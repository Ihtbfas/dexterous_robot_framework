from __future__ import annotations

from dataclasses import dataclass

from dexterous_robot.control.hand import GraspLockController, GraspLockGoal
from dexterous_robot.core import Command, FailureReason, SkillResult, SkillStatus
from dexterous_robot.runtime import RuntimeSnapshot

from ._common import positive_finite, snapshot_numeric_signal


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
        self._stable_since_s: float | None = None
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
