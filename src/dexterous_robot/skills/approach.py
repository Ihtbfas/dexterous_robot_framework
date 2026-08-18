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
