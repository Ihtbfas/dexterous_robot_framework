from __future__ import annotations

import pytest

from dexterous_robot.core import JointPositionCommand, JointState, Pose, SkillStatus
from dexterous_robot.devices.arms.wam7 import WAM7_JOINT_NAMES
from dexterous_robot.devices.hands.linker_l20 import L20_PHYSICAL_JOINTS
from dexterous_robot.motion.limits import ResolvedJointKinematicLimits
from dexterous_robot.runtime import RuntimeSnapshot
from dexterous_robot.skills.approach import ArmWaypoint, PreshapeApproachPlan, PreshapeApproachSkill


def _limits() -> ResolvedJointKinematicLimits:
    return ResolvedJointKinematicLimits(
        joint_names=WAM7_JOINT_NAMES,
        velocity_rad_s=(1.875,) * 7,
        acceleration_rad_s2=(1.0e9,) * 7,
        jerk_rad_s3=(1.0e9,) * 7,
    )


def _snapshot(time_s: float, arm_q: tuple[float, ...]) -> RuntimeSnapshot:
    return RuntimeSnapshot(
        time_s=time_s,
        dt_s=0.1,
        device_states={
            "arm": JointState(WAM7_JOINT_NAMES, arm_q, (0.0,) * 7),
            "hand": JointState(L20_PHYSICAL_JOINTS, (0.0,) * 21, (0.0,) * 21),
        },
        body_poses={"object": Pose((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), "world")},
        signals={},
    )


def _hand_open() -> JointPositionCommand:
    return JointPositionCommand("hand", L20_PHYSICAL_JOINTS, (0.0,) * 21, profile="hand_open_hold")


def test_arm_waypoint_has_no_duration_authority() -> None:
    waypoint = ArmWaypoint("ready", (0.1,) * 7)
    assert waypoint.name == "ready"
    assert not hasattr(waypoint, "duration_s")


def test_approach_times_from_actual_segment_start_not_previous_nominal_target() -> None:
    plan = PreshapeApproachPlan(
        arm_waypoints=(ArmWaypoint("first", (0.2,) * 7), ArmWaypoint("second", (1.0,) * 7)),
        preshape_hand_q_rad=(0.0,) * 21,
        preshape_duration_s=0.1,
        grasp_waypoint=ArmWaypoint("grasp", (1.0,) * 7),
        settle_duration_s=0.1,
        joint_tolerance_rad=0.02,
    )
    skill = PreshapeApproachSkill(plan=plan, hand_open_command=_hand_open(), joint_limits=_limits())

    result, _ = skill.step(_snapshot(0.0, (0.0,) * 7))
    assert result.status is SkillStatus.RUNNING
    assert skill.segment_timings[0][0] == "first"
    assert skill.segment_timings[0][1].duration_s == pytest.approx(0.2)

    # First nominal target is 0.2 rad, but measured state reaches only 0.1 rad
    # when the timed segment completes. The next duration must therefore use
    # actual 0.1 -> 1.0 displacement (0.9), not nominal 0.2 -> 1.0 (0.8).
    skill.step(_snapshot(0.2, (0.1,) * 7))
    result, _ = skill.step(_snapshot(0.2, (0.1,) * 7))
    assert result.status is SkillStatus.RUNNING
    assert skill.segment_timings[1][0] == "second"
    assert skill.segment_timings[1][1].duration_s == pytest.approx(0.9)
