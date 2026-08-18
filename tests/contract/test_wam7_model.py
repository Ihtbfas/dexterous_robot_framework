from __future__ import annotations

import ast
import dataclasses
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
EXPECTED_WAM7_JOINTS = (
    "wam_j1_joint",
    "wam_j2_joint",
    "wam_j3_joint",
    "wam_j4_joint",
    "wam_j5_joint",
    "wam_j6_joint",
    "wam_j7_joint",
)


def test_wam7_joint_order_and_frames_are_frozen() -> None:
    from dexterous_robot.devices.arms.wam7 import WAM7_JOINT_NAMES, Wam7Model

    model = Wam7Model()
    assert WAM7_JOINT_NAMES == EXPECTED_WAM7_JOINTS
    assert model.joint_names == EXPECTED_WAM7_JOINTS
    assert model.device_id == "arm"
    assert model.base_frame == "wam_base"
    assert model.flange_frame == "wam_j7"


def test_wam7_model_is_frozen_and_rejects_semantic_drift() -> None:
    from dexterous_robot.devices.arms.wam7 import Wam7Model

    model = Wam7Model()
    with pytest.raises(dataclasses.FrozenInstanceError):
        model.device_id = "changed"  # type: ignore[misc]
    with pytest.raises(ValueError, match="WAM7_JOINT_NAMES_INVALID"):
        Wam7Model(joint_names=tuple(reversed(EXPECTED_WAM7_JOINTS)))
    with pytest.raises(ValueError, match="WAM7_DEVICE_ID_INVALID"):
        Wam7Model(device_id="")
    with pytest.raises(ValueError, match="WAM7_BASE_FRAME_INVALID"):
        Wam7Model(base_frame="")
    with pytest.raises(ValueError, match="WAM7_FLANGE_FRAME_INVALID"):
        Wam7Model(flange_frame="")


def test_wam7_device_module_has_no_backend_runtime_dependencies() -> None:
    forbidden = {"isaacsim", "omni", "pxr", "mujoco", "rclpy", "can", "socket", "subprocess"}
    module_dir = ROOT / "src/dexterous_robot/devices/arms/wam7"
    for module in module_dir.glob("*.py"):
        tree = ast.parse(module.read_text(encoding="utf-8"), filename=str(module))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                assert not any(alias.name.split(".")[0] in forbidden for alias in node.names), module
            elif isinstance(node, ast.ImportFrom) and node.module:
                assert node.module.split(".")[0] not in forbidden, module
