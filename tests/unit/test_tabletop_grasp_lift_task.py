from __future__ import annotations

from dataclasses import dataclass

from dexterous_robot.core import FailureReason, JointPositionCommand, SkillResult, SkillStatus
from dexterous_robot.runtime import RuntimeSnapshot
from dexterous_robot.tasks.tabletop_grasp_lift import TabletopGraspLiftTask, TaskPhase


@dataclass
class ScriptedSkill:
    results: list[SkillResult]
    reset_count: int = 0

    def step(self, snapshot: RuntimeSnapshot):
        del snapshot
        result = self.results.pop(0) if len(self.results) > 1 else self.results[0]
        command = JointPositionCommand("arm", ("j",), (0.0,))
        return result, (command,)

    def reset(self) -> None:
        self.reset_count += 1


def _snapshot(time_s: float) -> RuntimeSnapshot:
    return RuntimeSnapshot(time_s=time_s, dt_s=0.1, device_states={}, body_poses={}, signals={})


def test_task_drives_sequence_using_only_skill_results() -> None:
    task = TabletopGraspLiftTask(
        approach=ScriptedSkill([SkillResult(SkillStatus.SUCCESS)]),
        grasp=ScriptedSkill([SkillResult(SkillStatus.SUCCESS)]),
        lift=ScriptedSkill([SkillResult(SkillStatus.SUCCESS)]),
        hold=ScriptedSkill([SkillResult(SkillStatus.RUNNING), SkillResult(SkillStatus.SUCCESS)]),
    )

    expected = [TaskPhase.GRASP, TaskPhase.LIFT, TaskPhase.HOLD, TaskPhase.HOLD, TaskPhase.SUCCESS]
    statuses = []
    for index, phase in enumerate(expected):
        result, commands = task.step(_snapshot(index * 0.1))
        statuses.append(result.status)
        assert task.phase is phase
        if phase is not TaskPhase.SUCCESS:
            assert commands

    assert statuses == [
        SkillStatus.RUNNING,
        SkillStatus.RUNNING,
        SkillStatus.RUNNING,
        SkillStatus.RUNNING,
        SkillStatus.SUCCESS,
    ]


def test_task_aborts_on_skill_failure_without_retrying_or_inspecting_backend() -> None:
    failing = ScriptedSkill([SkillResult(SkillStatus.FAILURE, FailureReason.OBJECT_SLIPPED, "semantic slip")])
    untouched = ScriptedSkill([SkillResult(SkillStatus.SUCCESS)])
    task = TabletopGraspLiftTask(
        approach=ScriptedSkill([SkillResult(SkillStatus.SUCCESS)]),
        grasp=ScriptedSkill([SkillResult(SkillStatus.SUCCESS)]),
        lift=failing,
        hold=untouched,
    )
    task.step(_snapshot(0.0))
    task.step(_snapshot(0.1))
    result, commands = task.step(_snapshot(0.2))

    assert result.status is SkillStatus.FAILURE
    assert result.reason is FailureReason.OBJECT_SLIPPED
    assert task.phase is TaskPhase.FAILURE
    assert commands

    terminal, terminal_commands = task.step(_snapshot(0.3))
    assert terminal == result
    assert terminal_commands == ()
    assert untouched.results == [SkillResult(SkillStatus.SUCCESS)]


def test_task_reset_resets_all_skills_and_phase() -> None:
    skills = [ScriptedSkill([SkillResult(SkillStatus.RUNNING)]) for _ in range(4)]
    task = TabletopGraspLiftTask(approach=skills[0], grasp=skills[1], lift=skills[2], hold=skills[3])
    task.reset()
    assert task.phase is TaskPhase.APPROACH
    assert [skill.reset_count for skill in skills] == [1, 1, 1, 1]
