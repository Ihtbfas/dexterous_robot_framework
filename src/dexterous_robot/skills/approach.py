from __future__ import annotations

from dataclasses import dataclass

from dexterous_robot.core import Command, FailureReason, JointPositionCommand, SkillResult, SkillStatus
from dexterous_robot.runtime import RuntimeSnapshot

from ._common import positive_finite, snapshot_joint_state


@dataclass(frozen=True)
class ApproachGoal:
    arm_target: JointPositionCommand
    joint_tolerance_rad: float
    stable_duration_s: float
    timeout_s: float

    def __post_init__(self) -> None:
        if not isinstance(self.arm_target, JointPositionCommand):
            raise ValueError("APPROACH_ARM_TARGET_INVALID")
        if self.arm_target.device_id != "arm":
            raise ValueError("APPROACH_ARM_DEVICE_INVALID")
        tolerance = positive_finite(self.joint_tolerance_rad, error="APPROACH_TOLERANCE_INVALID")
        stable = positive_finite(self.stable_duration_s, error="APPROACH_STABLE_DURATION_INVALID", allow_zero=True)
        timeout = positive_finite(self.timeout_s, error="APPROACH_TIMEOUT_INVALID")
        if stable > timeout:
            raise ValueError("APPROACH_STABLE_DURATION_EXCEEDS_TIMEOUT")
        object.__setattr__(self, "joint_tolerance_rad", tolerance)
        object.__setattr__(self, "stable_duration_s", stable)
        object.__setattr__(self, "timeout_s", timeout)


class ApproachSkill:
    def __init__(self, goal: ApproachGoal) -> None:
        if not isinstance(goal, ApproachGoal):
            raise ValueError("APPROACH_GOAL_INVALID")
        self._goal = goal
        self.reset()

    def reset(self) -> None:
        self._started_at_s: float | None = None
        self._stable_since_s: float | None = None

    def step(self, snapshot: RuntimeSnapshot) -> tuple[SkillResult, tuple[Command, ...]]:
        if self._started_at_s is None:
            self._started_at_s = snapshot.time_s
        elapsed = snapshot.time_s - self._started_at_s
        command = self._goal.arm_target
        if elapsed > self._goal.timeout_s:
            return SkillResult(SkillStatus.FAILURE, FailureReason.TIMEOUT, "approach timeout"), (command,)

        try:
            state = snapshot_joint_state(snapshot, command.device_id)
        except (KeyError, ValueError) as exc:
            return SkillResult(SkillStatus.FAILURE, FailureReason.RUNTIME_ERROR, str(exc)), (command,)
        if state.names != command.joint_names:
            return SkillResult(
                SkillStatus.FAILURE,
                FailureReason.RUNTIME_ERROR,
                "approach joint-state order mismatch",
            ), (command,)
        max_error = max(abs(actual - target) for actual, target in zip(state.position_rad, command.position_rad, strict=True))
        if max_error <= self._goal.joint_tolerance_rad:
            if self._stable_since_s is None:
                self._stable_since_s = snapshot.time_s
            if snapshot.time_s - self._stable_since_s + 1.0e-12 >= self._goal.stable_duration_s:
                return SkillResult(SkillStatus.SUCCESS), (command,)
        else:
            self._stable_since_s = None
        return SkillResult(SkillStatus.RUNNING), (command,)


@dataclass(frozen=True)
class ArmWaypoint:
    name: str
    q_rad: tuple[float, ...]
    duration_s: float

    def __post_init__(self) -> None:
        from dexterous_robot.devices.arms.wam7 import WAM7_JOINT_NAMES
        if not isinstance(self.name, str) or not self.name:
            raise ValueError("ARM_WAYPOINT_NAME_INVALID")
        values = tuple(float(v) for v in self.q_rad)
        if len(values) != len(WAM7_JOINT_NAMES):
            raise ValueError("ARM_WAYPOINT_WIDTH_INVALID")
        object.__setattr__(self, "q_rad", values)
        object.__setattr__(self, "duration_s", positive_finite(self.duration_s, error="ARM_WAYPOINT_DURATION_INVALID"))


@dataclass(frozen=True)
class PreshapeApproachPlan:
    arm_waypoints: tuple[ArmWaypoint, ...]
    preshape_hand_q_rad: tuple[float, ...]
    preshape_duration_s: float
    grasp_waypoint: ArmWaypoint
    settle_duration_s: float
    joint_tolerance_rad: float

    def __post_init__(self) -> None:
        from dexterous_robot.devices.hands.linker_l20 import L20_PHYSICAL_JOINTS
        if not isinstance(self.arm_waypoints, tuple) or not self.arm_waypoints or not all(isinstance(v, ArmWaypoint) for v in self.arm_waypoints):
            raise ValueError("PRESHAPE_APPROACH_WAYPOINTS_INVALID")
        if not isinstance(self.grasp_waypoint, ArmWaypoint):
            raise ValueError("PRESHAPE_APPROACH_GRASP_WAYPOINT_INVALID")
        hand = tuple(float(v) for v in self.preshape_hand_q_rad)
        if len(hand) != len(L20_PHYSICAL_JOINTS):
            raise ValueError("PRESHAPE_APPROACH_HAND_WIDTH_INVALID")
        object.__setattr__(self, "preshape_hand_q_rad", hand)
        object.__setattr__(self, "preshape_duration_s", positive_finite(self.preshape_duration_s, error="PRESHAPE_APPROACH_DURATION_INVALID"))
        object.__setattr__(self, "settle_duration_s", positive_finite(self.settle_duration_s, error="PRESHAPE_APPROACH_SETTLE_INVALID", allow_zero=True))
        object.__setattr__(self, "joint_tolerance_rad", positive_finite(self.joint_tolerance_rad, error="PRESHAPE_APPROACH_TOLERANCE_INVALID"))


class PreshapeApproachSkill:
    """Timed collision-safe arm approach with hand preshape before the final grasp waypoint."""

    def __init__(self, *, plan: PreshapeApproachPlan, hand_open_command: JointPositionCommand) -> None:
        from dexterous_robot.control import JointTargetRampController
        if not isinstance(plan, PreshapeApproachPlan):
            raise ValueError("PRESHAPE_APPROACH_PLAN_INVALID")
        if not isinstance(hand_open_command, JointPositionCommand) or hand_open_command.device_id != "hand":
            raise ValueError("PRESHAPE_APPROACH_HAND_OPEN_INVALID")
        self._plan = plan
        self._hand_open = hand_open_command
        self._ramp = JointTargetRampController()
        self.reset()

    def reset(self) -> None:
        self._phase = "ARM"
        self._arm_index = 0
        self._phase_started_s: float | None = None
        self._segment_start_arm: tuple[float, ...] | None = None
        self._segment_start_hand: tuple[float, ...] | None = None

    @property
    def local_phase(self) -> str:
        return self._phase

    def _arm_hold(self, q_rad: tuple[float, ...]) -> JointPositionCommand:
        from dexterous_robot.devices.arms.wam7 import WAM7_JOINT_NAMES
        return JointPositionCommand("arm", WAM7_JOINT_NAMES, q_rad, profile="arm_carry_position_drive")

    def _begin(self, snapshot: RuntimeSnapshot) -> None:
        arm = snapshot_joint_state(snapshot, "arm")
        hand = snapshot_joint_state(snapshot, "hand")
        self._phase_started_s = snapshot.time_s
        self._segment_start_arm = arm.position_rad
        self._segment_start_hand = hand.position_rad

    def step(self, snapshot: RuntimeSnapshot) -> tuple[SkillResult, tuple[Command, ...]]:
        try:
            arm_state = snapshot_joint_state(snapshot, "arm")
            hand_state = snapshot_joint_state(snapshot, "hand")
        except (KeyError, ValueError) as exc:
            return SkillResult(SkillStatus.FAILURE, FailureReason.RUNTIME_ERROR, str(exc)), ()
        if self._phase_started_s is None:
            self._begin(snapshot)
        assert self._phase_started_s is not None and self._segment_start_arm is not None and self._segment_start_hand is not None
        elapsed = max(0.0, snapshot.time_s - self._phase_started_s)

        if self._phase == "ARM":
            waypoint = self._plan.arm_waypoints[self._arm_index]
            arm_cmd = self._ramp.compute(
                device_id="arm", joint_names=arm_state.names,
                start_rad=self._segment_start_arm, target_rad=waypoint.q_rad,
                elapsed_s=min(elapsed, waypoint.duration_s), duration_s=waypoint.duration_s,
                profile="arm_carry_position_drive",
            )
            if elapsed + 1.0e-12 >= waypoint.duration_s:
                self._arm_index += 1
                self._phase_started_s = snapshot.time_s
                self._segment_start_arm = arm_state.position_rad
                if self._arm_index >= len(self._plan.arm_waypoints):
                    self._phase = "PRESHAPE"
                    self._segment_start_hand = hand_state.position_rad
            return SkillResult(SkillStatus.RUNNING), (arm_cmd, self._hand_open)

        final_pregrasp = self._plan.arm_waypoints[-1].q_rad
        if self._phase == "PRESHAPE":
            hand_cmd = self._ramp.compute(
                device_id="hand", joint_names=hand_state.names,
                start_rad=self._segment_start_hand, target_rad=self._plan.preshape_hand_q_rad,
                elapsed_s=min(elapsed, self._plan.preshape_duration_s), duration_s=self._plan.preshape_duration_s,
                profile="hand_open_hold",
            )
            if elapsed + 1.0e-12 >= self._plan.preshape_duration_s:
                self._phase = "GRASP_WAYPOINT"
                self._phase_started_s = snapshot.time_s
                self._segment_start_arm = arm_state.position_rad
            return SkillResult(SkillStatus.RUNNING), (self._arm_hold(final_pregrasp), hand_cmd)

        preshape_cmd = JointPositionCommand("hand", hand_state.names, self._plan.preshape_hand_q_rad, profile="hand_open_hold")
        if self._phase == "GRASP_WAYPOINT":
            waypoint = self._plan.grasp_waypoint
            arm_cmd = self._ramp.compute(
                device_id="arm", joint_names=arm_state.names,
                start_rad=self._segment_start_arm, target_rad=waypoint.q_rad,
                elapsed_s=min(elapsed, waypoint.duration_s), duration_s=waypoint.duration_s,
                profile="arm_carry_position_drive",
            )
            if elapsed + 1.0e-12 >= waypoint.duration_s:
                self._phase = "SETTLE"
                self._phase_started_s = snapshot.time_s
            return SkillResult(SkillStatus.RUNNING), (arm_cmd, preshape_cmd)

        if self._phase == "SETTLE":
            target = self._plan.grasp_waypoint.q_rad
            max_error = max(abs(a - b) for a, b in zip(arm_state.position_rad, target, strict=True))
            commands = (self._arm_hold(target), preshape_cmd)
            if elapsed + 1.0e-12 < self._plan.settle_duration_s:
                return SkillResult(SkillStatus.RUNNING), commands
            if max_error > self._plan.joint_tolerance_rad:
                return SkillResult(SkillStatus.FAILURE, FailureReason.TARGET_UNREACHABLE, f"approach joint error {max_error:.6f} rad"), commands
            return SkillResult(SkillStatus.SUCCESS), commands

        return SkillResult(SkillStatus.FAILURE, FailureReason.RUNTIME_ERROR, "invalid approach local phase"), ()
