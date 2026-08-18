from __future__ import annotations

import json
from pathlib import Path

import pytest
import yaml

FIXTURE = Path(__file__).resolve().parents[1] / "fixtures" / "l20_legacy_golden_vectors.json"
ROOT = Path(__file__).resolve().parents[2]


def _golden() -> dict[str, object]:
    return json.loads(FIXTURE.read_text(encoding="utf-8"))


def _command(values: list[float]):
    from dexterous_robot.devices.hands.linker_l20.types import L20_ACTIVE_CHANNELS, L20ActiveCommand16

    return L20ActiveCommand16.from_mapping(
        dict(zip(L20_ACTIVE_CHANNELS, values, strict=True)), timestamp_s=1.25, sequence_id=7
    )


def test_protocol_slot_contract_is_exact_and_reserved_slots_are_fixed_zero() -> None:
    from dexterous_robot.devices.hands.linker_l20.protocol import REVERSED_SLOTS, SLOT_SOURCES

    golden = _golden()
    assert list(SLOT_SOURCES) == golden["protocol_slot_sources"]
    assert len(SLOT_SOURCES) == 20
    assert set(REVERSED_SLOTS) == {0, 1, 2, 3, 4, 5, 10, 15, 16, 17, 18, 19}
    assert SLOT_SOURCES[11:15] == (None, None, None, None)


@pytest.mark.parametrize("vector_name", ["all_open", "all_min", "all_max", "mixed_direction_probe"])
def test_protocol20_normalized_and_encoded_bytes_match_frozen_legacy_vectors(vector_name: str) -> None:
    from dexterous_robot.devices.hands.linker_l20.protocol import adapt_active_to_protocol20, encode_official20

    vector = _golden()["vectors"][vector_name]
    command = _command(vector["active16"])
    protocol = adapt_active_to_protocol20(command)
    assert protocol.values == pytest.approx(vector["protocol20_normalized"], abs=0.0, rel=0.0)
    assert protocol.values[11:15] == (0.0, 0.0, 0.0, 0.0)
    assert encode_official20(command) == tuple(vector["official20_bytes"])


def test_linker_l20_model_and_yaml_select_a_profile_without_backend_dependency() -> None:
    from dexterous_robot.devices.hands.linker_l20.model import LinkerL20Model
    from dexterous_robot.devices.hands.linker_l20.types import L20_ACTIVE_CHANNELS, L20_PHYSICAL_JOINTS

    raw = yaml.safe_load((ROOT / "configs/devices/hands/linker_l20.yaml").read_text(encoding="utf-8"))
    model = LinkerL20Model(device_id=raw["device_id"], coupling_profile=raw["coupling_profile"])
    assert model.active_channels == L20_ACTIVE_CHANNELS
    assert model.physical_joints == L20_PHYSICAL_JOINTS
    assert model.coupling_profile == "mujoco_equal_v1"


def test_legacy_fixture_records_authority_sha256s() -> None:
    authority = _golden()["legacy_authority_sha256"]
    assert set(authority) == {
        "scripts/phase2/l20_dynamic_types.py",
        "scripts/phase2/l20_active_mapper.py",
        "scripts/phase2/l20_official20_codec.py",
        "configs/phase2/l20_command_mapping_v1.json",
    }
    assert all(len(value) == 64 for value in authority.values())
