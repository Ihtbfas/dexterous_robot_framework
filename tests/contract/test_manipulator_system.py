from __future__ import annotations

import dataclasses
from pathlib import Path

import pytest
import yaml

from dexterous_robot.core import Pose
from dexterous_robot.devices.hands.linker_l20 import LinkerL20Model

ROOT = Path(__file__).resolve().parents[2]
EXPECTED_MOUNT_Z_M = 0.00491000205278397 + 0.005


def _robot():
    from dexterous_robot.devices.arms.wam7 import Wam7Model
    from dexterous_robot.robots import ManipulatorSystem, MountTransform

    arm = Wam7Model()
    hand = LinkerL20Model(coupling_profile="mujoco_equal_v1")
    mount = MountTransform(
        parent_frame="wam_j7",
        child_frame="l20_base",
        pose=Pose(
            position_xyz_m=(0.0, 0.0, EXPECTED_MOUNT_Z_M),
            quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
            frame_id="wam_j7",
        ),
    )
    return ManipulatorSystem(
        system_id="wam7_linker_l20",
        arm=arm,
        hand=hand,
        hand_mount=mount,
        tcp_frame="l20_tcp",
    )


def test_manipulator_system_is_composition_only() -> None:
    robot = _robot()
    assert robot.arm.device_id == "arm"
    assert robot.hand.device_id == "hand"
    assert robot.hand_mount.parent_frame == robot.arm.flange_frame
    assert robot.hand_mount.child_frame == "l20_base"
    assert robot.hand_mount.pose.position_xyz_m == (0.0, 0.0, EXPECTED_MOUNT_Z_M)
    assert robot.tcp_frame == "l20_tcp"
    for forbidden in ("move_ee", "grasp", "close_hand", "lift"):
        assert not hasattr(robot, forbidden)


def test_mount_and_composition_are_frozen_and_fail_closed() -> None:
    from dexterous_robot.devices.arms.wam7 import Wam7Model
    from dexterous_robot.robots import ManipulatorSystem, MountTransform

    robot = _robot()
    with pytest.raises(dataclasses.FrozenInstanceError):
        robot.tcp_frame = "changed"  # type: ignore[misc]

    with pytest.raises(ValueError, match="MOUNT_POSE_PARENT_FRAME_MISMATCH"):
        MountTransform(
            parent_frame="wam_j7",
            child_frame="l20_base",
            pose=Pose((0.0, 0.0, 0.01), (0.0, 0.0, 0.0, 1.0), "world"),
        )
    with pytest.raises(ValueError, match="MANIPULATOR_MOUNT_PARENT_MISMATCH"):
        ManipulatorSystem(
            system_id="bad",
            arm=Wam7Model(),
            hand=LinkerL20Model(),
            hand_mount=MountTransform(
                parent_frame="some_other_frame",
                child_frame="l20_base",
                pose=Pose((0.0, 0.0, 0.01), (0.0, 0.0, 0.0, 1.0), "some_other_frame"),
            ),
            tcp_frame="l20_tcp",
        )
    with pytest.raises(ValueError, match="MANIPULATOR_DEVICE_ID_COLLISION"):
        ManipulatorSystem(
            system_id="bad",
            arm=Wam7Model(device_id="hand"),
            hand=LinkerL20Model(device_id="hand"),
            hand_mount=MountTransform(
                parent_frame="wam_j7",
                child_frame="l20_base",
                pose=Pose((0.0, 0.0, 0.01), (0.0, 0.0, 0.0, 1.0), "wam_j7"),
            ),
            tcp_frame="l20_tcp",
        )


def test_tracked_m1_composition_config_preserves_accepted_mount_candidate() -> None:
    wam_cfg = yaml.safe_load((ROOT / "configs/devices/arms/wam7.yaml").read_text(encoding="utf-8"))
    robot_cfg = yaml.safe_load((ROOT / "configs/robots/wam7_linker_l20.yaml").read_text(encoding="utf-8"))

    assert wam_cfg == {
        "schema_version": 1,
        "kind": "Wam7Model",
        "device_id": "arm",
        "base_frame": "wam_base",
        "flange_frame": "wam_j7",
    }
    assert robot_cfg["schema_version"] == 1
    assert robot_cfg["kind"] == "ManipulatorSystem"
    assert robot_cfg["system_id"] == "wam7_linker_l20"
    assert robot_cfg["arm_config"] == "../devices/arms/wam7.yaml"
    assert robot_cfg["hand_config"] == "../devices/hands/linker_l20.yaml"
    assert robot_cfg["tcp_frame"] == "l20_tcp"
    assert robot_cfg["hand_mount"]["parent_frame"] == "wam_j7"
    assert robot_cfg["hand_mount"]["child_frame"] == "l20_base"
    assert robot_cfg["hand_mount"]["position_xyz_m"] == [0.0, 0.0, EXPECTED_MOUNT_Z_M]
    assert robot_cfg["hand_mount"]["quaternion_xyzw"] == [0.0, 0.0, 0.0, 1.0]
    assert robot_cfg["hand_mount"]["selection"] == {
        "candidate_id": "M_R000_D005",
        "rotation_deg": 0,
        "standoff_m": 0.005,
        "flange_offset_m": 0.00491000205278397,
    }
    text = (ROOT / "configs/robots/wam7_linker_l20.yaml").read_text(encoding="utf-8")
    assert "/home/lyf/" not in text
