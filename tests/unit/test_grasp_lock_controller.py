from __future__ import annotations

import pytest

from dexterous_robot.control.hand.grasp_lock import GraspLockController, GraspLockGoal
from dexterous_robot.devices.hands.linker_l20 import L20_PHYSICAL_JOINTS, L20PhysicalTarget21


def _r15u_capture_target() -> L20PhysicalTarget21:
    values = {
        "thumb_joint0": 0.67,
        "thumb_joint1": 0.65,
        "thumb_joint2": 0.2427783333333289,
        "thumb_joint3": 0.3810874999999892,
        "thumb_joint4": 0.3810874999999892,
        "index_joint0": 0.0,
        "index_joint1": 0.3954999999999889,
        "index_joint2": 0.32183999999999585,
        "index_joint3": 0.32183999999999585,
        "middle_joint0": 0.0,
        "middle_joint1": 0.4135833333333216,
        "middle_joint2": 0.3367199999999956,
        "middle_joint3": 0.3367199999999956,
        "ring_joint0": 0.0,
        "ring_joint1": 0.3269999999999926,
        "ring_joint2": 0.2725599999999972,
        "ring_joint3": 0.2725599999999972,
        "little_joint0": 0.0,
        "little_joint1": 0.25083333333332715,
        "little_joint2": 0.2027999999999977,
        "little_joint3": 0.2027999999999977,
    }
    return L20PhysicalTarget21(
        tuple(values[name] for name in L20_PHYSICAL_JOINTS),
        "mujoco_equal_v1",
        source_timestamp_s=12.5,
        sequence_id=42,
    )


def test_grasp_lock_matches_frozen_successful_locked_target() -> None:
    controller = GraspLockController()
    command = controller.compute(GraspLockGoal(_r15u_capture_target()))
    actual = dict(zip(command.joint_names, command.position_rad, strict=True))
    expected = {
        "thumb_joint0": 0.67,
        "thumb_joint1": 0.65,
        "thumb_joint2": 0.2827783333333289,
        "thumb_joint3": 0.42108749999998923,
        "thumb_joint4": 0.42108749999998923,
        "index_joint0": 0.0,
        "index_joint1": 0.3954999999999889,
        "index_joint2": 0.32183999999999585,
        "index_joint3": 0.32183999999999585,
        "middle_joint0": 0.0,
        "middle_joint1": 0.4135833333333216,
        "middle_joint2": 0.3367199999999956,
        "middle_joint3": 0.3367199999999956,
        "ring_joint0": 0.0,
        "ring_joint1": 0.4069999999999926,
        "ring_joint2": 0.3525599999999972,
        "ring_joint3": 0.3525599999999972,
        "little_joint0": 0.0,
        "little_joint1": 0.33083333333332715,
        "little_joint2": 0.2827999999999977,
        "little_joint3": 0.2827999999999977,
    }
    assert command.device_id == "hand"
    assert command.profile == "hand_grasp_lock"
    assert command.joint_names == L20_PHYSICAL_JOINTS
    assert actual == pytest.approx(expected)


def test_grasp_lock_latches_first_target_and_disables_online_reshaping() -> None:
    controller = GraspLockController()
    first = controller.compute(GraspLockGoal(_r15u_capture_target()))
    different = L20PhysicalTarget21((0.0,) * 21, "mujoco_equal_v1")
    second = controller.compute(GraspLockGoal(different))
    assert second is first
    controller.reset()
    third = controller.compute(GraspLockGoal(different))
    assert third is not first
    assert third.position_rad != first.position_rad


def test_grasp_lock_enforces_all_follower_relations_for_selected_profile() -> None:
    controller = GraspLockController()
    base = _r15u_capture_target()
    urdf = L20PhysicalTarget21(base.positions_rad, "urdf_mimic_v1")
    cmd = controller.compute(GraspLockGoal(urdf))
    values = dict(zip(cmd.joint_names, cmd.position_rad, strict=True))
    assert values["thumb_joint4"] == pytest.approx(values["thumb_joint3"])
    assert values["index_joint3"] == pytest.approx(values["index_joint2"] * 1.06399)
    assert values["middle_joint3"] == pytest.approx(values["middle_joint2"] * 1.06399)
    assert values["ring_joint3"] == pytest.approx(values["ring_joint2"] * 1.06399)
    assert values["little_joint3"] == pytest.approx(values["little_joint2"] * 1.06399)
