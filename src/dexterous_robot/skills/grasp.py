from __future__ import annotations

from dataclasses import dataclass

from dexterous_robot.control.hand import GraspLockController, GraspLockGoal
from dexterous_robot.core import Command, FailureReason, JointPositionCommand, SkillResult, SkillStatus
from dexterous_robot.runtime import RuntimeSnapshot

from ._common import positive_finite, snapshot_joint_state, snapshot_numeric_signal


@dataclass(frozen=True)
class GraspCriteria:
    minimum_squeeze_n: float
    stable_duration_s: float
    timeout_s: float
    squeeze_signal: str = "opposing_y_squeeze_n"

    def __post_init__(self) -> None:
        minimum = positive_finite(self.minimum_squeeze_n, error="GRASP_MINIMUM_SQUEEZE_INVALID", allow_zero=True)
        stable = positive_finite(self.stable_duration_s, error="GRASP_STABLE_DURATION_INVALID", allow_zero=True)
        timeout = positive_finite(self.timeout_s, error="GRASP_TIMEOUT_INVALID")
        if stable > timeout:
            raise ValueError("GRASP_STABLE_DURATION_EXCEEDS_TIMEOUT")
        if not isinstance(self.squeeze_signal, str) or not self.squeeze_signal:
            raise ValueError("GRASP_SQUEEZE_SIGNAL_INVALID")
        object.__setattr__(self, "minimum_squeeze_n", minimum)
        object.__setattr__(self, "stable_duration_s", stable)
        object.__setattr__(self, "timeout_s", timeout)


class GraspSkill:
    def __init__(self, *, controller: GraspLockController, goal: GraspLockGoal, criteria: GraspCriteria) -> None:
        if not isinstance(controller, GraspLockController):
            raise ValueError("GRASP_CONTROLLER_INVALID")
        if not isinstance(goal, GraspLockGoal):
            raise ValueError("GRASP_GOAL_INVALID")
        if not isinstance(criteria, GraspCriteria):
            raise ValueError("GRASP_CRITERIA_INVALID")
        self._controller = controller
        self._goal = goal
        self._criteria = criteria
        self.reset()

    def reset(self) -> None:
        self._started_at_s: float | None = None
        self._last_squeeze_n: float | None = None
        self._squeeze_quality_met = False
        self._controller.reset()

    def step(self, snapshot: RuntimeSnapshot) -> tuple[SkillResult, tuple[Command, ...]]:
        if self._started_at_s is None:
            self._started_at_s = snapshot.time_s
        command = self._controller.compute(self._goal)
        elapsed = snapshot.time_s - self._started_at_s
        if elapsed > self._criteria.timeout_s:
            return SkillResult(
                SkillStatus.FAILURE,
                FailureReason.GRASP_NOT_ESTABLISHED,
                "semantic squeeze did not stabilize before timeout",
            ), (command,)
        try:
            squeeze_n = snapshot_numeric_signal(snapshot, self._criteria.squeeze_signal)
        except (KeyError, ValueError) as exc:
            return SkillResult(SkillStatus.FAILURE, FailureReason.RUNTIME_ERROR, str(exc)), (command,)
        if squeeze_n >= self._criteria.minimum_squeeze_n:
            if self._stable_since_s is None:
                self._stable_since_s = snapshot.time_s
            if snapshot.time_s - self._stable_since_s + 1.0e-12 >= self._criteria.stable_duration_s:
                return SkillResult(SkillStatus.SUCCESS), (command,)
        else:
            self._stable_since_s = None
        return SkillResult(SkillStatus.RUNNING), (command,)


@dataclass(frozen=True)
class PreloadGraspCriteria:
    target_squeeze_n: float
    lock_hold_duration_s: float
    squeeze_signal: str = "opposing_y_squeeze_n"

    def __post_init__(self) -> None:
        object.__setattr__(self, "target_squeeze_n", positive_finite(self.target_squeeze_n, error="PRELOAD_GRASP_TARGET_SQUEEZE_INVALID", allow_zero=True))
        object.__setattr__(self, "lock_hold_duration_s", positive_finite(self.lock_hold_duration_s, error="PRELOAD_GRASP_LOCK_HOLD_INVALID"))
        if not isinstance(self.squeeze_signal, str) or not self.squeeze_signal:
            raise ValueError("PRELOAD_GRASP_SIGNAL_INVALID")


class PreloadGraspSkill:
    """Dynamic release -> preload -> fixed grasp-lock ramp/hold; squeeze remains telemetry-only."""

    def __init__(
        self,
        *,
        arm_hold_command: JointPositionCommand,
        preshape_hand_command: JointPositionCommand,
        controller: GraspLockController,
        goal: GraspLockGoal,
        release_settle_s: float,
        preload_duration_s: float,
        lock_ramp_duration_s: float,
        criteria: PreloadGraspCriteria,
    ) -> None:
        from dexterous_robot.control import JointTargetRampController
        if not isinstance(arm_hold_command, JointPositionCommand) or arm_hold_command.device_id != "arm":
            raise ValueError("PRELOAD_GRASP_ARM_HOLD_INVALID")
        if not isinstance(preshape_hand_command, JointPositionCommand) or preshape_hand_command.device_id != "hand":
            raise ValueError("PRELOAD_GRASP_PRESHAPE_INVALID")
        if not isinstance(controller, GraspLockController) or not isinstance(goal, GraspLockGoal):
            raise ValueError("PRELOAD_GRASP_CONTROLLER_INVALID")
        if not isinstance(criteria, PreloadGraspCriteria):
            raise ValueError("PRELOAD_GRASP_CRITERIA_INVALID")
        self._arm_hold = arm_hold_command
        self._preshape = preshape_hand_command
        self._controller = controller
        self._goal = goal
        self._release_settle_s = positive_finite(release_settle_s, error="PRELOAD_GRASP_RELEASE_SETTLE_INVALID", allow_zero=True)
        self._preload_duration_s = positive_finite(preload_duration_s, error="PRELOAD_GRASP_PRELOAD_DURATION_INVALID")
        self._lock_ramp_duration_s = positive_finite(lock_ramp_duration_s, error="PRELOAD_GRASP_LOCK_RAMP_DURATION_INVALID")
        self._criteria = criteria
        self._ramp = JointTargetRampController()
        self.reset()

    def reset(self) -> None:
        self._phase = "RELEASE"
        self._phase_started_s: float | None = None
        self._release_sent = False
        self._last_squeeze_n: float | None = None
        self._squeeze_quality_met = False
        self._controller.reset()

    @property
    def local_phase(self) -> str:
        return self._phase

    @property
    def last_squeeze_n(self) -> float | None:
        return self._last_squeeze_n

    @property
    def squeeze_quality_met(self) -> bool:
        return self._squeeze_quality_met

    def _elapsed(self, snapshot: RuntimeSnapshot) -> float:
        if self._phase_started_s is None:
            self._phase_started_s = snapshot.time_s
        return max(0.0, snapshot.time_s - self._phase_started_s)

    def _transition(self, phase: str, snapshot: RuntimeSnapshot) -> None:
        self._phase = phase
        self._phase_started_s = snapshot.time_s

    def step(self, snapshot: RuntimeSnapshot) -> tuple[SkillResult, tuple[Command, ...]]:
        from dexterous_robot.core import RigidBodyKinematicCommand
        try:
            hand_state = snapshot_joint_state(snapshot, "hand")
        except (KeyError, ValueError) as exc:
            return SkillResult(SkillStatus.FAILURE, FailureReason.RUNTIME_ERROR, str(exc)), (self._arm_hold, self._preshape)
        elapsed = self._elapsed(snapshot)

        if self._phase == "RELEASE":
            commands: list[Command] = [self._arm_hold, self._preshape]
            if not self._release_sent:
                commands.insert(0, RigidBodyKinematicCommand("object", False))
                self._release_sent = True
            if elapsed + 1.0e-12 < self._release_settle_s:
                return SkillResult(SkillStatus.RUNNING), tuple(commands)
            self._transition("PRELOAD", snapshot)
            elapsed = 0.0

        base = self._goal.base_target.positions_rad
        if self._phase == "PRELOAD":
            hand_cmd = self._ramp.compute(
                device_id="hand", joint_names=hand_state.names,
                start_rad=self._preshape.position_rad, target_rad=base,
                elapsed_s=min(elapsed, self._preload_duration_s), duration_s=self._preload_duration_s,
                profile="hand_grasp_lock",
            )
            if elapsed + 1.0e-12 >= self._preload_duration_s:
                self._transition("LOCK", snapshot)
            return SkillResult(SkillStatus.RUNNING), (self._arm_hold, hand_cmd)

        final = self._controller.compute(self._goal)
        if self._phase == "LOCK":
            hand_cmd = self._ramp.compute(
                device_id="hand", joint_names=hand_state.names,
                start_rad=base, target_rad=final.position_rad,
                elapsed_s=min(elapsed, self._lock_ramp_duration_s), duration_s=self._lock_ramp_duration_s,
                profile="hand_grasp_lock",
            )
            if elapsed + 1.0e-12 < self._lock_ramp_duration_s:
                return SkillResult(SkillStatus.RUNNING), (self._arm_hold, hand_cmd)
            self._transition("HOLD", snapshot)
            elapsed = 0.0

        if self._phase == "HOLD":
            try:
                squeeze = snapshot_numeric_signal(snapshot, self._criteria.squeeze_signal)
            except (KeyError, ValueError) as exc:
                return SkillResult(SkillStatus.FAILURE, FailureReason.RUNTIME_ERROR, str(exc)), (self._arm_hold, final)
            self._last_squeeze_n = squeeze
            self._squeeze_quality_met = squeeze >= self._criteria.target_squeeze_n
            if elapsed + 1.0e-12 >= self._criteria.lock_hold_duration_s:
                message = (
                    "fixed grasp-lock hold complete"
                    if self._squeeze_quality_met
                    else "fixed grasp-lock hold complete; squeeze target not met (telemetry only)"
                )
                return SkillResult(SkillStatus.SUCCESS, FailureReason.NONE, message), (self._arm_hold, final)
            return SkillResult(SkillStatus.RUNNING), (self._arm_hold, final)

        return SkillResult(SkillStatus.FAILURE, FailureReason.RUNTIME_ERROR, "invalid grasp local phase"), (self._arm_hold,)
