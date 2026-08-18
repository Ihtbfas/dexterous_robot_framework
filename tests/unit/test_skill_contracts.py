from __future__ import annotations

from dataclasses import replace

import pytest

from dexterous_robot.control.arm import CartesianCarryController, CartesianCarryGoal, Wam7Kinematics
from dexterous_robot.control.hand import GraspLockController, GraspLockGoal
from dexterous_robot.core import JointPositionCommand, JointState, Pose, SkillStatus, FailureReason
from dexterous_robot.devices.arms.wam7 import WAM7_JOINT_NAMES
from dexterous_robot.devices.hands.linker_l20 import L20_PHYSICAL_JOINTS, L20PhysicalTarget21
from dexterous_robot.runtime import RuntimeSnapshot
from dexterous_robot.skills.approach import ApproachGoal, ApproachSkill
from dexterous_robot.skills.grasp import GraspCriteria, GraspSkill
from dexterous_robot.skills.hold import HoldCriteria, HoldSkill
from dexterous_robot.skills.lift import LiftCriteria, LiftSkill


def _snapshot(
    *,
    time_s: float,
    arm_q: tuple[float, ...] | None = None,
    hand_q: tuple[float, ...] | None = None,
    object_xyz: tuple[float, float, float] = (0.68, -0.14, 1.0125),
    hand_xyz: tuple[float, float, float] = (0.68, -0.11, 1.0125),
    squeeze_n: float = 0.0,
    table_normal_n: float = 1.0,
) -> RuntimeSnapshot:
    arm_q = arm_q or (0.0,) * 7
    hand_q = hand_q or (0.0,) * 21
    return RuntimeSnapshot(
        time_s=time_s,
        dt_s=0.1,
        device_states={
            "arm": JointState(WAM7_JOINT_NAMES, arm_q, (0.0,) * 7),
            "hand": JointState(L20_PHYSICAL_JOINTS, hand_q, (0.0,) * 21),
        },
        body_poses={
            "object": Pose(object_xyz, (0.0, 0.0, 0.0, 1.0), "world"),
            "hand_tcp": Pose(hand_xyz, (0.0, 0.0, 0.0, 1.0), "world"),
        },
        signals={
            "opposing_y_squeeze_n": squeeze_n,
            "object_table_normal_n": table_normal_n,
        },
    )


def _hand_base_target() -> L20PhysicalTarget21:
    return L20PhysicalTarget21((0.2,) * 21, "mujoco_equal_v1")


def test_approach_skill_requires_stable_joint_convergence() -> None:
    target = JointPositionCommand("arm", WAM7_JOINT_NAMES, (0.1,) * 7, profile="arm_carry_position_drive")
    skill = ApproachSkill(ApproachGoal(target, joint_tolerance_rad=0.01, stable_duration_s=0.2, timeout_s=1.0))

    result, commands = skill.step(_snapshot(time_s=0.0, arm_q=(0.0,) * 7))
    assert result.status is SkillStatus.RUNNING
    assert commands == (target,)

    result, _ = skill.step(_snapshot(time_s=0.1, arm_q=(0.1,) * 7))
    assert result.status is SkillStatus.RUNNING
    result, _ = skill.step(_snapshot(time_s=0.3, arm_q=(0.1,) * 7))
    assert result.status is SkillStatus.SUCCESS


def test_grasp_skill_uses_semantic_squeeze_and_latched_grasp_lock() -> None:
    controller = GraspLockController()
    skill = GraspSkill(
        controller=controller,
        goal=GraspLockGoal(_hand_base_target()),
        criteria=GraspCriteria(minimum_squeeze_n=0.25, stable_duration_s=0.2, timeout_s=1.0),
    )

    result, first_commands = skill.step(_snapshot(time_s=0.0, squeeze_n=0.1))
    assert result.status is SkillStatus.RUNNING
    assert first_commands[0].profile == "hand_grasp_lock"
    result, second_commands = skill.step(_snapshot(time_s=0.1, squeeze_n=0.30))
    assert result.status is SkillStatus.RUNNING
    result, third_commands = skill.step(_snapshot(time_s=0.3, squeeze_n=0.30))
    assert result.status is SkillStatus.SUCCESS
    assert first_commands == second_commands == third_commands


def test_grasp_skill_reports_semantic_failure_without_raw_backend_details() -> None:
    skill = GraspSkill(
        controller=GraspLockController(),
        goal=GraspLockGoal(_hand_base_target()),
        criteria=GraspCriteria(minimum_squeeze_n=0.5, stable_duration_s=0.1, timeout_s=0.25),
    )
    skill.step(_snapshot(time_s=0.0, squeeze_n=0.0))
    result, _ = skill.step(_snapshot(time_s=0.3, squeeze_n=0.0))
    assert result.status is SkillStatus.FAILURE
    assert result.reason is FailureReason.GRASP_NOT_ESTABLISHED
    assert "PhysX" not in result.message


def test_lift_skill_reports_object_slipped_on_relative_pose_drift() -> None:
    kin = Wam7Kinematics()
    q = (0.0, 0.15, 0.0, -0.15, -1.5, 0.1, 0.0)
    locked_pose = kin.forward(q)
    carry_goal = CartesianCarryGoal(locked_pose, (0.0, 0.0, 0.05), 1.0)
    hand_hold = JointPositionCommand("hand", L20_PHYSICAL_JOINTS, (0.2,) * 21)
    skill = LiftSkill(
        controller=CartesianCarryController(kinematics=kin),
        carry_goal=carry_goal,
        hand_hold_command=hand_hold,
        criteria=LiftCriteria(max_relative_drift_m=0.01, minimum_object_rise_m=0.025, max_table_normal_n=0.1),
    )

    start = _snapshot(
        time_s=0.0,
        arm_q=q,
        object_xyz=(0.68, -0.14, 1.0125),
        hand_xyz=(0.68, -0.11, 1.0125),
        table_normal_n=0.0,
    )
    result, commands = skill.step(start)
    assert result.status is SkillStatus.RUNNING
    assert {command.device_id for command in commands} == {"arm", "hand"}

    slipped = _snapshot(
        time_s=0.1,
        arm_q=q,
        object_xyz=(0.68, -0.18, 1.0125),
        hand_xyz=(0.68, -0.11, 1.0125),
        table_normal_n=0.0,
    )
    result, commands = skill.step(slipped)
    assert result.status is SkillStatus.FAILURE
    assert result.reason is FailureReason.OBJECT_SLIPPED
    assert commands == (hand_hold,)


def test_hold_skill_requires_at_least_half_second_off_table() -> None:
    with pytest.raises(ValueError, match="HOLD_DURATION_TOO_SHORT"):
        HoldCriteria(
            hold_duration_s=0.49,
            table_top_z_m=0.98,
            object_half_height_m=0.0325,
            minimum_clearance_m=0.001,
            max_table_normal_n=0.1,
            max_relative_drift_m=0.01,
        )


def test_hold_skill_succeeds_only_after_continuous_off_table_duration() -> None:
    import json
    from pathlib import Path

    kin = Wam7Kinematics()
    fixture = json.loads((Path(__file__).resolve().parents[1] / "fixtures" / "wam7_legacy_ik_golden_vectors.json").read_text(encoding="utf-8"))
    q = tuple(next(row for row in fixture["vectors"] if row["name"] == "grasp")["solved_q_rad"])
    carry_goal = CartesianCarryGoal(kin.forward(q), (0.0, 0.0, 0.05), 1.0)
    hand_hold = JointPositionCommand("hand", L20_PHYSICAL_JOINTS, (0.2,) * 21)
    skill = HoldSkill(
        controller=CartesianCarryController(kinematics=kin),
        carry_goal=carry_goal,
        hand_hold_command=hand_hold,
        criteria=HoldCriteria(
            hold_duration_s=0.5,
            table_top_z_m=0.98,
            object_half_height_m=0.0325,
            minimum_clearance_m=0.001,
            max_table_normal_n=0.1,
            max_relative_drift_m=0.01,
        ),
    )
    kwargs = dict(
        arm_q=q,
        object_xyz=(0.68, -0.14, 1.0465),
        hand_xyz=(0.68, -0.11, 1.0465),
        table_normal_n=0.0,
    )
    result, _ = skill.step(_snapshot(time_s=2.0, **kwargs))
    assert result.status is SkillStatus.RUNNING
    result, _ = skill.step(_snapshot(time_s=2.49, **kwargs))
    assert result.status is SkillStatus.RUNNING
    result, _ = skill.step(_snapshot(time_s=2.5, **kwargs))
    assert result.status is SkillStatus.SUCCESS


def test_skills_and_tasks_do_not_import_backend_or_transport_namespaces() -> None:
    from pathlib import Path

    root = Path(__file__).resolve().parents[2] / "src" / "dexterous_robot"
    forbidden = ("omni", "isaacsim", "mujoco", "rclpy", "scripts.phase2", "dexterous_robot.backends")
    for package in (root / "skills", root / "tasks"):
        for path in package.rglob("*.py"):
            text = path.read_text(encoding="utf-8")
            for token in forbidden:
                assert token not in text, f"{path.relative_to(root)} imports forbidden token {token}"
