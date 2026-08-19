from __future__ import annotations

import pytest

from dexterous_robot.control.arm import CartesianCarryController, Wam7Kinematics
from dexterous_robot.core import JointPositionCommand, JointState, Pose, SkillStatus
from dexterous_robot.devices.arms.wam7 import WAM7_JOINT_NAMES
from dexterous_robot.devices.hands.linker_l20 import L20_PHYSICAL_JOINTS
from dexterous_robot.motion.limits import ResolvedCartesianKinematicLimits
from dexterous_robot.runtime import RuntimeSnapshot
from dexterous_robot.skills.lift import LiftCriteria, LivePoseLiftSkill

_Q = (0.6409612055, 0.6602303863, -0.7924558436, -0.7563616005, -1.7493225115, 0.5366375803, 0.4366375803)


def _snapshot(*, dt_s: float = 0.1) -> RuntimeSnapshot:
    tcp = Wam7Kinematics().forward(_Q)
    return RuntimeSnapshot(
        time_s=1.0,
        dt_s=dt_s,
        device_states={
            "arm": JointState(WAM7_JOINT_NAMES, _Q, (0.0,) * 7),
            "hand": JointState(L20_PHYSICAL_JOINTS, (0.2,) * 21, (0.0,) * 21),
        },
        body_poses={
            "object": Pose((0.68, -0.14, 1.0125), (0.0, 0.0, 0.0, 1.0), "world"),
            "hand_tcp": tcp,
        },
        signals={"object_table_normal_n": 0.0},
    )


def _hand_hold() -> JointPositionCommand:
    return JointPositionCommand("hand", L20_PHYSICAL_JOINTS, (0.2,) * 21, profile="hand_grasp_lock")


def test_live_lift_computes_3_5_s_for_m17_carry_limits() -> None:
    limits = ResolvedCartesianKinematicLimits(
        linear_velocity_m_s=0.04285714285714286,
        linear_acceleration_m_s2=0.03770450737564903,
        linear_jerk_m_s3=0.1119533527696793,
    )
    skill = LivePoseLiftSkill(
        controller=CartesianCarryController(kinematics=Wam7Kinematics()),
        hand_hold_command=_hand_hold(),
        delta_world_m=(0.0, 0.0, 0.08),
        cartesian_limits=limits,
        criteria=LiftCriteria(max_relative_drift_m=0.03, minimum_object_rise_m=0.025, max_table_normal_n=0.1),
    )
    result, _ = skill.step(_snapshot())
    assert result.status is SkillStatus.RUNNING
    assert skill.timing_result is not None
    assert skill.timing_result.duration_s == pytest.approx(3.5, abs=1.0e-12)
    assert skill.carry_goal is not None
    assert skill.carry_goal.duration_s == pytest.approx(3.5, abs=1.0e-12)


def test_zero_distance_uses_snapshot_dt_floor() -> None:
    limits = ResolvedCartesianKinematicLimits(1.0, 1.0, 1.0)
    skill = LivePoseLiftSkill(
        controller=CartesianCarryController(kinematics=Wam7Kinematics()),
        hand_hold_command=_hand_hold(),
        delta_world_m=(0.0, 0.0, 0.0),
        cartesian_limits=limits,
        criteria=LiftCriteria(max_relative_drift_m=0.03, minimum_object_rise_m=0.0, max_table_normal_n=0.1),
    )
    skill.step(_snapshot(dt_s=0.125))
    assert skill.timing_result is not None
    assert skill.timing_result.duration_s == pytest.approx(0.125)
