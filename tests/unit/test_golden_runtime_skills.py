from __future__ import annotations

from dexterous_robot.control.arm import CartesianCarryController, Wam7Kinematics
from dexterous_robot.control.hand import GraspLockController, GraspLockGoal
from dexterous_robot.core import (
    JointPositionCommand,
    JointState,
    Pose,
    RigidBodyKinematicCommand,
    FailureReason,
    SkillStatus,
)
from dexterous_robot.devices.arms.wam7 import WAM7_JOINT_NAMES
from dexterous_robot.devices.hands.linker_l20 import L20_PHYSICAL_JOINTS, L20PhysicalTarget21
from dexterous_robot.motion.limits import ResolvedJointKinematicLimits
from dexterous_robot.runtime import RuntimeSnapshot
from dexterous_robot.skills.approach import ArmWaypoint, PreshapeApproachPlan, PreshapeApproachSkill
from dexterous_robot.skills.grasp import PreloadGraspCriteria, PreloadGraspSkill
from dexterous_robot.skills.hold import HoldCriteria, SuspendedHoldSkill
from dexterous_robot.skills.lift import LiftCriteria, LivePoseLiftSkill


def _snapshot(
    *,
    time_s: float,
    arm_q=(0.0,) * 7,
    hand_q=(0.0,) * 21,
    object_xyz=(0.68, -0.14, 1.0125),
    hand_xyz=(0.49, -0.18, 1.017),
    hand_quat=(0.0, 0.0, 0.0, 1.0),
    squeeze_n=0.0,
    table_normal_n=0.0,
) -> RuntimeSnapshot:
    return RuntimeSnapshot(
        time_s=time_s,
        dt_s=0.1,
        device_states={
            "arm": JointState(WAM7_JOINT_NAMES, tuple(arm_q), (0.0,) * 7),
            "hand": JointState(L20_PHYSICAL_JOINTS, tuple(hand_q), (0.0,) * 21),
        },
        body_poses={
            "object": Pose(tuple(object_xyz), (0.0, 0.0, 0.0, 1.0), "world"),
            "hand_tcp": Pose(tuple(hand_xyz), tuple(hand_quat), "world"),
        },
        signals={
            "opposing_y_squeeze_n": squeeze_n,
            "object_table_normal_n": table_normal_n,
        },
    )


def _hand_command(values=(0.1,) * 21, profile="hand_open_hold") -> JointPositionCommand:
    return JointPositionCommand("hand", L20_PHYSICAL_JOINTS, tuple(values), profile=profile)


def _approach_limits() -> ResolvedJointKinematicLimits:
    return ResolvedJointKinematicLimits(
        joint_names=WAM7_JOINT_NAMES,
        velocity_rad_s=(1.875,) * 7,
        acceleration_rad_s2=(1.0e9,) * 7,
        jerk_rad_s3=(1.0e9,) * 7,
    )


def test_rigid_body_kinematic_command_is_typed_and_immutable() -> None:
    command = RigidBodyKinematicCommand("object", False)
    assert command.body_id == "object"
    assert command.kinematic_enabled is False


def test_preshape_approach_keeps_hand_open_then_preshapes_before_final_grasp_waypoint() -> None:
    plan = PreshapeApproachPlan(
        arm_waypoints=(
            ArmWaypoint("ready", (0.1,) * 7),
            ArmWaypoint("transit", (0.2,) * 7),
            ArmWaypoint("pregrasp", (0.3,) * 7),
        ),
        preshape_hand_q_rad=(0.25,) * 21,
        preshape_duration_s=0.1,
        grasp_waypoint=ArmWaypoint("grasp", (0.4,) * 7),
        settle_duration_s=0.1,
        joint_tolerance_rad=0.02,
    )
    open_cmd = _hand_command()
    skill = PreshapeApproachSkill(plan=plan, hand_open_command=open_cmd, joint_limits=_approach_limits())

    result, commands = skill.step(_snapshot(time_s=0.0))
    assert result.status is SkillStatus.RUNNING
    assert commands[1] == open_cmd
    for time_s, q in ((0.1, (0.1,) * 7), (0.2, (0.2,) * 7), (0.3, (0.3,) * 7)):
        result, commands = skill.step(_snapshot(time_s=time_s, arm_q=q))
        assert result.status is SkillStatus.RUNNING
    result, commands = skill.step(_snapshot(time_s=0.4, arm_q=(0.3,) * 7, hand_q=(0.25,) * 21))
    assert result.status is SkillStatus.RUNNING
    assert commands[1].device_id == "hand"
    result, commands = skill.step(_snapshot(time_s=0.5, arm_q=(0.4,) * 7, hand_q=(0.25,) * 21))
    assert result.status is SkillStatus.RUNNING
    result, _ = skill.step(_snapshot(time_s=0.6, arm_q=(0.4,) * 7, hand_q=(0.25,) * 21))
    assert result.status is SkillStatus.SUCCESS


def test_preload_grasp_releases_object_before_preload_and_locks_fixed_target() -> None:
    base = L20PhysicalTarget21((0.2,) * 21, "mujoco_equal_v1")
    controller = GraspLockController()
    final = controller.compute(GraspLockGoal(base))
    controller.reset()
    arm_hold = JointPositionCommand("arm", WAM7_JOINT_NAMES, (0.4,) * 7, profile="arm_carry_position_drive")
    preshape = _hand_command((0.1,) * 21)
    skill = PreloadGraspSkill(
        arm_hold_command=arm_hold,
        preshape_hand_command=preshape,
        controller=controller,
        goal=GraspLockGoal(base),
        release_settle_s=0.1,
        preload_duration_s=0.2,
        lock_ramp_duration_s=0.2,
        criteria=PreloadGraspCriteria(target_squeeze_n=0.3, lock_hold_duration_s=0.1),
    )

    result, commands = skill.step(_snapshot(time_s=0.0, hand_q=(0.1,) * 21))
    assert result.status is SkillStatus.RUNNING
    assert any(isinstance(command, RigidBodyKinematicCommand) for command in commands)
    assert commands[-1].profile == "hand_open_hold"
    result, commands = skill.step(_snapshot(time_s=0.1, hand_q=(0.1,) * 21))
    assert not any(isinstance(command, RigidBodyKinematicCommand) for command in commands)
    assert commands[-1].profile == "hand_grasp_lock"
    skill.step(_snapshot(time_s=0.3, hand_q=(0.2,) * 21))
    result, commands = skill.step(_snapshot(time_s=0.5, hand_q=final.position_rad, squeeze_n=0.4))
    assert commands[-1].position_rad == final.position_rad
    result, commands = skill.step(_snapshot(time_s=0.6, hand_q=final.position_rad, squeeze_n=0.4))
    assert result.status is SkillStatus.SUCCESS
    assert commands[-1].position_rad == final.position_rad


def test_live_pose_lift_locks_actual_tcp_pose_on_first_sample() -> None:
    kin = Wam7Kinematics()
    q = (0.6409612055, 0.6602303863, -0.7924558436, -0.7563616005, -1.7493225115, 0.5366375803, 0.4366375803)
    actual_tcp = kin.forward(q)
    hand_hold = _hand_command((0.2,) * 21, profile="hand_grasp_lock")
    skill = LivePoseLiftSkill(
        controller=CartesianCarryController(kinematics=kin),
        hand_hold_command=hand_hold,
        delta_world_m=(0.0, 0.0, 0.05),
        duration_s=0.2,
        criteria=LiftCriteria(max_relative_drift_m=0.03, minimum_object_rise_m=0.025, max_table_normal_n=0.1),
    )
    start = _snapshot(time_s=1.0, arm_q=q, hand_xyz=actual_tcp.position_xyz_m, hand_quat=actual_tcp.quaternion_xyzw)
    result, _ = skill.step(start)
    assert result.status is SkillStatus.RUNNING
    assert skill.carry_goal is not None
    assert skill.carry_goal.locked_tcp_pose == actual_tcp


def test_suspended_hold_captures_arm_joint_hold_and_requires_continuous_half_second() -> None:
    q = (0.1,) * 7
    hand_hold = _hand_command((0.2,) * 21, profile="hand_grasp_lock")
    skill = SuspendedHoldSkill(
        hand_hold_command=hand_hold,
        criteria=HoldCriteria(
            hold_duration_s=0.5,
            table_top_z_m=0.98,
            object_half_height_m=0.0325,
            minimum_clearance_m=0.001,
            max_table_normal_n=0.1,
            max_relative_drift_m=0.03,
        ),
    )
    kwargs = dict(arm_q=q, object_xyz=(0.68, -0.14, 1.0465), hand_xyz=(0.49, -0.18, 1.05))
    result, commands = skill.step(_snapshot(time_s=2.0, **kwargs))
    assert result.status is SkillStatus.RUNNING
    assert commands[0].position_rad == q
    assert commands[0].profile == "arm_carry_position_drive"
    result, _ = skill.step(_snapshot(time_s=2.5, **kwargs))
    assert result.status is SkillStatus.SUCCESS


def test_preload_grasp_fixed_lock_hold_is_not_hard_gated_by_squeeze_telemetry() -> None:
    base = L20PhysicalTarget21((0.2,) * 21, "mujoco_equal_v1")
    controller = GraspLockController()
    final = controller.compute(GraspLockGoal(base))
    controller.reset()
    arm_hold = JointPositionCommand("arm", WAM7_JOINT_NAMES, (0.4,) * 7, profile="arm_carry_position_drive")
    preshape = _hand_command((0.1,) * 21)
    skill = PreloadGraspSkill(
        arm_hold_command=arm_hold,
        preshape_hand_command=preshape,
        controller=controller,
        goal=GraspLockGoal(base),
        release_settle_s=0.1,
        preload_duration_s=0.1,
        lock_ramp_duration_s=0.1,
        criteria=PreloadGraspCriteria(target_squeeze_n=0.45, lock_hold_duration_s=0.2),
    )

    skill.step(_snapshot(time_s=0.0, hand_q=(0.1,) * 21, squeeze_n=0.0))
    skill.step(_snapshot(time_s=0.1, hand_q=(0.1,) * 21, squeeze_n=0.0))
    skill.step(_snapshot(time_s=0.2, hand_q=(0.2,) * 21, squeeze_n=0.0))
    skill.step(_snapshot(time_s=0.3, hand_q=final.position_rad, squeeze_n=0.0))
    result, commands = skill.step(_snapshot(time_s=0.5, hand_q=final.position_rad, squeeze_n=0.0))

    assert result.status is SkillStatus.SUCCESS
    assert result.reason is FailureReason.NONE
    assert commands[-1].position_rad == final.position_rad
    assert skill.last_squeeze_n == 0.0
    assert skill.squeeze_quality_met is False
