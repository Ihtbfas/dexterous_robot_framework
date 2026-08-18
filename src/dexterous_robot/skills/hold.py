from __future__ import annotations

from dataclasses import dataclass

from dexterous_robot.control.arm import CartesianCarryController, CartesianCarryGoal
from dexterous_robot.core import Command, FailureReason, JointPositionCommand, SkillResult, SkillStatus
from dexterous_robot.runtime import RuntimeSnapshot

from ._common import positive_finite, relative_xyz, snapshot_joint_state, snapshot_numeric_signal, snapshot_pose, xyz_distance


@dataclass(frozen=True)
class HoldCriteria:
    hold_duration_s: float
    table_top_z_m: float
    object_half_height_m: float
    minimum_clearance_m: float
    max_table_normal_n: float
    max_relative_drift_m: float
    object_body_id: str = "object"
    hand_body_id: str = "hand_tcp"
    table_normal_signal: str = "object_table_normal_n"

    def __post_init__(self) -> None:
        hold = positive_finite(self.hold_duration_s, error="HOLD_DURATION_INVALID")
        if hold < 0.5:
            raise ValueError("HOLD_DURATION_TOO_SHORT")
        half_height = positive_finite(self.object_half_height_m, error="HOLD_OBJECT_HALF_HEIGHT_INVALID")
        clearance = positive_finite(self.minimum_clearance_m, error="HOLD_CLEARANCE_INVALID", allow_zero=True)
        normal = positive_finite(self.max_table_normal_n, error="HOLD_MAX_TABLE_NORMAL_INVALID", allow_zero=True)
        drift = positive_finite(self.max_relative_drift_m, error="HOLD_MAX_RELATIVE_DRIFT_INVALID")
        try:
            table_z = float(self.table_top_z_m)
        except (TypeError, ValueError) as exc:
            raise ValueError("HOLD_TABLE_TOP_Z_INVALID") from exc
        for value, error in (
            (self.object_body_id, "HOLD_OBJECT_BODY_ID_INVALID"),
            (self.hand_body_id, "HOLD_HAND_BODY_ID_INVALID"),
            (self.table_normal_signal, "HOLD_TABLE_NORMAL_SIGNAL_INVALID"),
        ):
            if not isinstance(value, str) or not value:
                raise ValueError(error)
        object.__setattr__(self, "hold_duration_s", hold)
        object.__setattr__(self, "table_top_z_m", table_z)
        object.__setattr__(self, "object_half_height_m", half_height)
        object.__setattr__(self, "minimum_clearance_m", clearance)
        object.__setattr__(self, "max_table_normal_n", normal)
        object.__setattr__(self, "max_relative_drift_m", drift)


class HoldSkill:
    def __init__(
        self,
        *,
        controller: CartesianCarryController,
        carry_goal: CartesianCarryGoal,
        hand_hold_command: JointPositionCommand,
        criteria: HoldCriteria,
    ) -> None:
        if not isinstance(controller, CartesianCarryController):
            raise ValueError("HOLD_CONTROLLER_INVALID")
        if not isinstance(carry_goal, CartesianCarryGoal):
            raise ValueError("HOLD_CARRY_GOAL_INVALID")
        if not isinstance(hand_hold_command, JointPositionCommand) or hand_hold_command.device_id != "hand":
            raise ValueError("HOLD_HAND_HOLD_COMMAND_INVALID")
        if not isinstance(criteria, HoldCriteria):
            raise ValueError("HOLD_CRITERIA_INVALID")
        self._controller = controller
        self._carry_goal = carry_goal
        self._hand_hold_command = hand_hold_command
        self._criteria = criteria
        self.reset()

    def reset(self) -> None:
        self._started_at_s: float | None = None
        self._initial_relative_xyz: tuple[float, float, float] | None = None

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

        relative = relative_xyz(object_pose, hand_pose)
        if self._started_at_s is None:
            self._started_at_s = snapshot.time_s
            self._initial_relative_xyz = relative
        assert self._initial_relative_xyz is not None
        if xyz_distance(relative, self._initial_relative_xyz) > self._criteria.max_relative_drift_m:
            return self._hand_only(
                SkillResult(SkillStatus.FAILURE, FailureReason.OBJECT_SLIPPED, "object-to-hand relative pose drift exceeded hold bound")
            )

        object_bottom_z_m = object_pose.position_xyz_m[2] - self._criteria.object_half_height_m
        minimum_bottom_z_m = self._criteria.table_top_z_m + self._criteria.minimum_clearance_m
        if object_bottom_z_m < minimum_bottom_z_m or table_normal_n > self._criteria.max_table_normal_n:
            return self._hand_only(
                SkillResult(SkillStatus.FAILURE, FailureReason.OBJECT_SLIPPED, "object no longer remains off table")
            )

        try:
            arm_command = self._controller.compute(
                elapsed_s=self._carry_goal.duration_s,
                current_q_rad=arm_state.position_rad,
                goal=self._carry_goal,
            )
        except (RuntimeError, ValueError) as exc:
            return self._hand_only(SkillResult(SkillStatus.FAILURE, FailureReason.TARGET_UNREACHABLE, str(exc)))
        commands: tuple[Command, ...] = (arm_command, self._hand_hold_command)
        if snapshot.time_s - self._started_at_s >= self._criteria.hold_duration_s:
            return SkillResult(SkillStatus.SUCCESS), commands
        return SkillResult(SkillStatus.RUNNING), commands
