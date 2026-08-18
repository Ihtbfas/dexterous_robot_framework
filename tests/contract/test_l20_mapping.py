from __future__ import annotations

import json
from pathlib import Path

import pytest

FIXTURE = Path(__file__).resolve().parents[1] / "fixtures" / "l20_legacy_golden_vectors.json"


def _golden() -> dict[str, object]:
    return json.loads(FIXTURE.read_text(encoding="utf-8"))


def _command(values: list[float]):
    from dexterous_robot.devices.hands.linker_l20.types import L20_ACTIVE_CHANNELS, L20ActiveCommand16

    return L20ActiveCommand16.from_mapping(
        dict(zip(L20_ACTIVE_CHANNELS, values, strict=True)), timestamp_s=1.25, sequence_id=7
    )


@pytest.mark.parametrize("vector_name", ["all_open", "all_min", "all_max", "mixed_direction_probe"])
@pytest.mark.parametrize("profile", ["mujoco_equal_v1", "urdf_mimic_v1"])
def test_active16_to_physical21_matches_frozen_legacy_vectors(vector_name: str, profile: str) -> None:
    from dexterous_robot.devices.hands.linker_l20.mapping import map_active_to_physical

    vector = _golden()["vectors"][vector_name]
    target = map_active_to_physical(_command(vector["active16"]), coupling_profile=profile)
    expected = vector["physical21_rad"][profile]
    assert len(target.positions_rad) == 21
    assert target.coupling_profile == profile
    assert target.source_timestamp_s == 1.25
    assert target.sequence_id == 7
    assert target.positions_rad == pytest.approx(expected, abs=1e-12, rel=0.0)


def test_coupling_profiles_preserve_frozen_follower_contract() -> None:
    from dexterous_robot.devices.hands.linker_l20.mapping import COUPLING_MULTIPLIERS

    assert COUPLING_MULTIPLIERS["urdf_mimic_v1"]["thumb_joint4"] == 1.0
    for joint in ("index_joint3", "middle_joint3", "ring_joint3", "little_joint3"):
        assert COUPLING_MULTIPLIERS["urdf_mimic_v1"][joint] == 1.06399
        assert COUPLING_MULTIPLIERS["mujoco_equal_v1"][joint] == 1.0


def test_unknown_coupling_profile_fails_closed() -> None:
    from dexterous_robot.devices.hands.linker_l20.mapping import map_active_to_physical

    vector = _golden()["vectors"]["all_min"]
    with pytest.raises(ValueError, match="L20_COUPLING_PROFILE_INVALID"):
        map_active_to_physical(_command(vector["active16"]), coupling_profile="unknown")
