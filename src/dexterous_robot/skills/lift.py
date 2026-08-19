from __future__ import annotations

import math
from dataclasses import dataclass

from dexterous_robot.control.arm import CartesianCarryController, CartesianCarryGoal
from dexterous_robot.core import Command, FailureReason, JointPositionCommand, SkillResult, SkillStatus
from dexterous_robot.motion.limits import ResolvedCartesianKinematicLimits
from dexterous_robot.motion.timing import ScalarTimingResult, minimum_jerk_duration
from dexterous_robot.runtime import RuntimeSnapshot

from ._common import positive_finite, relative_xyz, snapshot_joint_state, snapshot_numeric_signal, snapshot_pose, xyz_distance


@dataclass(frozen=True)
class LiftCriteria:
    max_relative_drift_m: float
    minimum_object_rise_m: float
    max_table_normal_n: float
    object_body_id: str = "object"
    hand_body_id: str = "hand_tcp"
    table_normal_signal: str = "object_table_normal_n"

    def __post_init__(self) -> None:
        drift = positive_finite(self.max_relative_drift_m, error="LIFT_MAX_RELATIVE_DRIFT_INVALID")
        rise = positive_finite(self.minimum_object_rise_m, error="LIFT_MINIMUM_OBJECT_RISE_INVALID", allow_zero=True)
        normal = positive_finite(self.max_table_normal_n, error="LIFT_MAX_TABLE_NORMAL_INVALID", allow_zero=True)
        for value, error in (
            (self.object_body_id, "LIFT_OBJECT_BODY_ID_INVALID"),
            (self.hand_body_id, "LIFT_HAND_BODY_ID_INVALID"),
            (self.table_normal_signal, "LIFT_TABLE_NORMAL_SIGNAL_INVALID"),
        ):
            if not isinstance(value, str) or not value:
                raise ValueError(error)
        object.__setattr__(self, "max_relative_drift_m", drift)
        object.__setattr__(self, "minimum_object_rise_m", rise)
        object.__setattr__(self, "max_table_normal_n", normal)


class LiftSkill:
    def __init__(
        self,
        *,
        controller: CartesianCarryController,
        carry_goal: CartesianCarryGoal,
        hand_hold_command: JointPositionCommand,
        criteria: LiftCriteria,
    ) -> None:
        if not isinstance(controller, CartesianCarryController):
            raise ValueError("LIFT_CONTROLLER_INVALID")
        if not isinstance(carry_goal, CartesianCarryGoal):
            raise ValueError("LIFT_CARRY_GOAL_INVALID")
        if not isinstance(hand_hold_command, JointPositionCommand) or hand_hold_command.device_id != "hand":
            raise ValueError("LIFT_HAND_HOLD_COMMAND_INVALID")
        if not isinstance(criteria, LiftCriteria):
            raise ValueError("LIFT_CRITERIA_INVALID")
        self._controller = controller
        self._carry_goal = carry_goal
        self._hand_hold_command = hand_hold_command
        self._criteria = criteria
        self.reset()

    def reset(self) -> None:
        self._started_at_s: float | None = None
        self._initial_relative_xyz: tuple[float, float, float] | None = None
        self._initial_object_z_m: float | None = None
        self._last_arm_command: JointPositionCommand | None = None

    @property
    def last_arm_command(self) -> JointPositionCommand | None:
        return self._last_arm_command

    def _hand_only(self, result: SkillResult) -> tuple[SkillResult, tuple[Command, ...]]:
        return result, (self._hand_hold_command,)

    def step(self, snapshot: RuntimeSnapshot) -> tuple[SkillResult, tuple[Command, ...]]:
        try:
            arm_state = snapshot_joint_state(snapshot, "arm")
            object_pose = snapshot_pose(snapshot, self._criteria.object_body_id)
            hand_pose = snapshot_pose(snapshot, self._criteria.hand_body_id)
            table_normal_n = snapshot_numeric_signal(snapshot, self._criteria.table_normal_signal)
        except (KeyError, ValueError) as exc:
            return self._hand_only(SkillResult(SkillStatus.FAILURE, FailureReason.RUNTIME_ERROR, str(exc)))

        if self._started_at_s is None:
            self._started_at_s = snapshot.time_s
            self._initial_relative_xyz = relative_xyz(object_pose, hand_pose)
            self._initial_object_z_m = object_pose.position_xyz_m[2]
        assert self._initial_relative_xyz is not None and self._initial_object_z_m is not None
        current_relative = relative_xyz(object_pose, hand_pose)
        if xyz_distance(current_relative, self._initial_relative_xyz) > self._criteria.max_relative_drift_m:
            return self._hand_only(
                SkillResult(SkillStatus.FAILURE, FailureReason.OBJECT_SLIPPED, "object-to-hand relative pose drift exceeded bound")
            )

        elapsed = max(0.0, snapshot.time_s - self._started_at_s)
        try:
            arm_command = self._controller.compute(
                elapsed_s=min(elapsed, self._carry_goal.duration_s),
                current_q_rad=arm_state.position_rad,
                goal=self._carry_goal,
            )
        except (RuntimeError, ValueError) as exc:
            return self._hand_only(SkillResult(SkillStatus.FAILURE, FailureReason.TARGET_UNREACHABLE, str(exc)))
        self._last_arm_command = arm_command
        commands: tuple[Command, ...] = (arm_command, self._hand_hold_command)

        if elapsed < self._carry_goal.duration_s:
            return SkillResult(SkillStatus.RUNNING), commands

        rise_m = object_pose.position_xyz_m[2] - self._initial_object_z_m
        if rise_m < self._criteria.minimum_object_rise_m:
            return SkillResult(
                SkillStatus.FAILURE,
                FailureReason.TARGET_UNREACHABLE,
                "object did not achieve minimum lift rise",
            ), commands
        if table_normal_n > self._criteria.max_table_normal_n:
            return SkillResult(
                SkillStatus.FAILURE,
                FailureReason.TARGET_UNREACHABLE,
                "object remained in table contact after carry",
            ), commands
        return SkillResult(SkillStatus.SUCCESS), commands


class LivePoseLiftSkill:
    """Fixed-orientation Cartesian lift auto-timed from resolved linear limits."""

    def __init__(
        self,
        *,
        controller: CartesianCarryController,
        hand_hold_command: JointPositionCommand,
        delta_world_m: tuple[float, float, float],
        cartesian_limits: ResolvedCartesianKinematicLimits,
        criteria: LiftCriteria,
    ) -> None:
        if not isinstance(controller, CartesianCarryController):
            raise ValueError("LIVE_LIFT_CONTROLLER_INVALID")
        if not isinstance(hand_hold_command, JointPositionCommand) or hand_hold_command.device_id != "hand":
            raise ValueError("LIVE_LIFT_HAND_HOLD_INVALID")
        if not isinstance(criteria, LiftCriteria):
            raise ValueError("LIVE_LIFT_CRITERIA_INVALID")
        if not isinstance(cartesian_limits, ResolvedCartesianKinematicLimits):
            raise ValueError("LIVE_LIFT_CARTESIAN_LIMITS_INVALID")
        try:
            delta = tuple(float(v) for v in delta_world_m)
        except (TypeError, ValueError) as exc:
            raise ValueError("LIVE_LIFT_DELTA_INVALID") from exc
        if len(delta) != 3 or not all(math.isfinite(v) for v in delta):
            raise ValueError("LIVE_LIFT_DELTA_INVALID")
        self._controller = controller
        self._hand_hold_command = hand_hold_command
        self._delta = delta
        self._cartesian_limits = cartesian_limits
        self._criteria = criteria
        self.reset()

    def reset(self) -> None:
        self._started_at_s: float | None = None
        self._initial_relative_xyz: tuple[float, float, float] | None = None
        self._initial_object_z_m: float | None = None
        self._carry_goal: CartesianCarryGoal | None = None
        self._timing_result: ScalarTimingResult | None = None
        self._last_arm_command: JointPositionCommand | None = None

    @property
    def carry_goal(self) -> CartesianCarryGoal | None:
        return self._carry_goal

    @property
    def timing_result(self) -> ScalarTimingResult | None:
        return self._timing_result

    @property
    def last_arm_command(self) -> JointPositionCommand | None:
        return self._last_arm_command

    def step(self, snapshot: RuntimeSnapshot) -> tuple[SkillResult, tuple[Command, ...]]:
        try:
            arm_state = snapshot_joint_state(snapshot, "arm")
            object_pose = snapshot_pose(snapshot, self._criteria.object_body_id)
            hand_pose = snapshot_pose(snapshot, self._criteria.hand_body_id)
            table_normal_n = snapshot_numeric_signal(snapshot, self._criteria.table_normal_signal)
        except (KeyError, ValueError) as exc:
            return SkillResult(SkillStatus.FAILURE, FailureReason.RUNTIME_ERROR, str(exc)), (self._hand_hold_command,)
        if self._started_at_s is None:
            self._started_at_s = snapshot.time_s
            self._initial_relative_xyz = relative_xyz(object_pose, hand_pose)
            self._initial_object_z_m = object_pose.position_xyz_m[2]
            distance_m = math.sqrt(sum(value * value for value in self._delta))
            self._timing_result = minimum_jerk_duration(
                distance_m,
                max_velocity=self._cartesian_limits.linear_velocity_m_s,
                max_acceleration=self._cartesian_limits.linear_acceleration_m_s2,
                max_jerk=self._cartesian_limits.linear_jerk_m_s3,
                minimum_duration_s=snapshot.dt_s,
            )
            self._carry_goal = CartesianCarryGoal(hand_pose, self._delta, self._timing_result.duration_s)
        assert self._initial_relative_xyz is not None and self._initial_object_z_m is not None and self._carry_goal is not None
        if xyz_distance(relative_xyz(object_pose, hand_pose), self._initial_relative_xyz) > self._criteria.max_relative_drift_m:
            return SkillResult(SkillStatus.FAILURE, FailureReason.OBJECT_SLIPPED, "object-to-hand relative pose drift exceeded lift bound"), (self._hand_hold_command,)
        elapsed = max(0.0, snapshot.time_s - self._started_at_s)
        duration_s = self._carry_goal.duration_s
        try:
            arm_command = self._controller.compute(
                elapsed_s=min(elapsed, duration_s), current_q_rad=arm_state.position_rad, goal=self._carry_goal
            )
        except (RuntimeError, ValueError) as exc:
            return SkillResult(SkillStatus.FAILURE, FailureReason.TARGET_UNREACHABLE, str(exc)), (self._hand_hold_command,)
        self._last_arm_command = arm_command
        commands: tuple[Command, ...] = (arm_command, self._hand_hold_command)
        if elapsed < duration_s:
            return SkillResult(SkillStatus.RUNNING), commands
        rise = object_pose.position_xyz_m[2] - self._initial_object_z_m
        if rise < self._criteria.minimum_object_rise_m:
            return SkillResult(SkillStatus.FAILURE, FailureReason.TARGET_UNREACHABLE, "object did not achieve minimum lift rise"), commands
        if table_normal_n > self._criteria.max_table_normal_n:
            return SkillResult(SkillStatus.FAILURE, FailureReason.TARGET_UNREACHABLE, "object remained in table contact after carry"), commands
        return SkillResult(SkillStatus.SUCCESS), commands
