from __future__ import annotations

import ast
import dataclasses
import json
from pathlib import Path

import pytest

FIXTURE = Path(__file__).resolve().parents[1] / "fixtures" / "l20_legacy_golden_vectors.json"
ROOT = Path(__file__).resolve().parents[2]


def _golden() -> dict[str, object]:
    return json.loads(FIXTURE.read_text(encoding="utf-8"))


def test_l20_canonical_spaces_match_frozen_legacy_authority() -> None:
    from dexterous_robot.devices.hands.linker_l20.types import (
        L20_ACTIVE_CHANNELS,
        L20_PHYSICAL_JOINTS,
    )

    golden = _golden()
    assert list(L20_ACTIVE_CHANNELS) == golden["active_channels"]
    assert len(L20_ACTIVE_CHANNELS) == 16
    assert list(L20_PHYSICAL_JOINTS) == golden["physical_joints"]
    assert len(L20_PHYSICAL_JOINTS) == 21


def test_active_command_is_frozen_ordered_and_range_checked() -> None:
    from dexterous_robot.devices.hands.linker_l20.types import (
        L20_ACTIVE_CHANNELS,
        L20ActiveCommand16,
    )

    values = {name: 0.25 for name in L20_ACTIVE_CHANNELS}
    command = L20ActiveCommand16.from_mapping(values, timestamp_s=1.25, sequence_id=7)
    assert tuple(name for name, _ in command.values) == L20_ACTIVE_CHANNELS
    assert command.as_mapping()["thumb_roll"] == 0.25
    with pytest.raises(dataclasses.FrozenInstanceError):
        command.sequence_id = 8  # type: ignore[misc]
    bad = dict(values)
    bad["thumb_roll"] = 1.01
    with pytest.raises(ValueError, match="L20_ACTIVE_COMMAND_VALUE_INVALID"):
        L20ActiveCommand16.from_mapping(bad, timestamp_s=1.25, sequence_id=7)
    with pytest.raises(ValueError, match="L20_ACTIVE_COMMAND_CHANNEL_MISMATCH"):
        L20ActiveCommand16.from_mapping({"thumb_roll": 0.2}, timestamp_s=1.25, sequence_id=7)


def test_protocol_and_physical_types_fail_closed_on_width_and_finiteness() -> None:
    from dexterous_robot.devices.hands.linker_l20.types import (
        L20PhysicalState21,
        L20PhysicalTarget21,
        L20ProtocolCommand20,
    )

    with pytest.raises(ValueError, match="L20_PROTOCOL_COMMAND20_INVALID"):
        L20ProtocolCommand20((0.0,) * 19)
    with pytest.raises(ValueError, match="L20_PHYSICAL_TARGET21_INVALID"):
        L20PhysicalTarget21((0.0,) * 20, "mujoco_equal_v1", 0.0, 0)
    with pytest.raises(ValueError, match="L20_PHYSICAL_STATE21_INVALID"):
        L20PhysicalState21((0.0,) * 21, (0.0,) * 20, None, 0.0)


def test_device_modules_have_no_runtime_backend_dependencies() -> None:
    forbidden = {"isaacsim", "omni", "pxr", "mujoco", "rclpy", "can", "socket", "subprocess"}
    module_dir = ROOT / "src/dexterous_robot/devices/hands/linker_l20"
    for module in module_dir.glob("*.py"):
        tree = ast.parse(module.read_text(encoding="utf-8"), filename=str(module))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                assert not any(alias.name.split(".")[0] in forbidden for alias in node.names), module
            elif isinstance(node, ast.ImportFrom) and node.module:
                assert node.module.split(".")[0] not in forbidden, module
