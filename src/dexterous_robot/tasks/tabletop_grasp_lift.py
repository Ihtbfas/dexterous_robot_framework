from __future__ import annotations

from enum import Enum
from typing import Protocol

from dexterous_robot.core import Command, SkillResult, SkillStatus
from dexterous_robot.runtime import RuntimeSnapshot


class TaskPhase(Enum):
    APPROACH = "APPROACH"
    GRASP = "GRASP"
    LIFT = "LIFT"
    HOLD = "HOLD"
    SUCCESS = "SUCCESS"
    FAILURE = "FAILURE"


class _Skill(Protocol):
    def step(self, snapshot: RuntimeSnapshot) -> tuple[SkillResult, tuple[Command, ...]]: ...
    def reset(self) -> None: ...


class TabletopGraspLiftTask:
    """M1 sequence orchestrator with fail/abort recovery policy.

    The task observes only SkillResult. Contact thresholds, relative-pose slip
    bounds, control convergence, and controller state remain inside Skills.
    """

    def __init__(self, *, approach: _Skill, grasp: _Skill, lift: _Skill, hold: _Skill) -> None:
        self._skills = {
            TaskPhase.APPROACH: approach,
            TaskPhase.GRASP: grasp,
            TaskPhase.LIFT: lift,
            TaskPhase.HOLD: hold,
        }
        for phase, skill in self._skills.items():
            if not callable(getattr(skill, "step", None)) or not callable(getattr(skill, "reset", None)):
                raise TypeError(f"TABLETOP_TASK_SKILL_INVALID:{phase.value}")
        self._phase = TaskPhase.APPROACH
        self._terminal_result: SkillResult | None = None

    @property
    def phase(self) -> TaskPhase:
        return self._phase

    def reset(self) -> None:
        for skill in self._skills.values():
            skill.reset()
        self._phase = TaskPhase.APPROACH
        self._terminal_result = None

    def step(self, snapshot: RuntimeSnapshot) -> tuple[SkillResult, tuple[Command, ...]]:
        if self._phase is TaskPhase.SUCCESS:
            assert self._terminal_result is not None
            return self._terminal_result, ()
        if self._phase is TaskPhase.FAILURE:
            assert self._terminal_result is not None
            return self._terminal_result, ()

        skill = self._skills[self._phase]
        result, commands = skill.step(snapshot)
        if not isinstance(result, SkillResult):
            raise TypeError("TABLETOP_TASK_SKILL_RESULT_INVALID")
        commands = tuple(commands)

        if result.status is SkillStatus.RUNNING:
            return result, commands
        if result.status is SkillStatus.FAILURE:
            self._phase = TaskPhase.FAILURE
            self._terminal_result = result
            return result, commands

        next_phase = {
            TaskPhase.APPROACH: TaskPhase.GRASP,
            TaskPhase.GRASP: TaskPhase.LIFT,
            TaskPhase.LIFT: TaskPhase.HOLD,
            TaskPhase.HOLD: TaskPhase.SUCCESS,
        }[self._phase]
        self._phase = next_phase
        if next_phase is TaskPhase.SUCCESS:
            self._terminal_result = SkillResult(SkillStatus.SUCCESS)
            return self._terminal_result, commands
        return SkillResult(SkillStatus.RUNNING), commands
