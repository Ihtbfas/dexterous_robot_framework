import dataclasses
from pathlib import Path

import pytest

from dexterous_robot.config.loader import ConfigError, load_local_asset_config


def test_local_asset_config_expands_environment(tmp_path, monkeypatch):
    monkeypatch.setenv("LEGACY_WAM_RUNTIME", "/tmp/wam.usda")
    p = tmp_path / "assets.yaml"
    p.write_text(
        "wam_runtime: ${LEGACY_WAM_RUNTIME}\n"
        "l20_runtime: /tmp/l20.usda\n",
        encoding="utf-8",
    )
    cfg = load_local_asset_config(p)
    assert cfg.wam_runtime.as_posix() == "/tmp/wam.usda"
    assert cfg.l20_runtime.as_posix() == "/tmp/l20.usda"
    with pytest.raises(dataclasses.FrozenInstanceError):
        cfg.wam_runtime = Path("/tmp/other.usda")


def test_local_asset_config_requires_exact_keys(tmp_path):
    p = tmp_path / "assets.yaml"
    p.write_text(
        "wam_runtime: /tmp/wam.usda\n"
        "l20_runtime: /tmp/l20.usda\n"
        "extra: forbidden\n",
        encoding="utf-8",
    )
    with pytest.raises(ConfigError, match="LOCAL_ASSET_CONFIG_KEYS_INVALID"):
        load_local_asset_config(p)


def test_local_asset_config_rejects_missing_key(tmp_path):
    p = tmp_path / "assets.yaml"
    p.write_text("wam_runtime: /tmp/wam.usda\n", encoding="utf-8")
    with pytest.raises(ConfigError, match="LOCAL_ASSET_CONFIG_KEYS_INVALID"):
        load_local_asset_config(p)


def test_local_asset_config_rejects_unexpanded_environment_variable(tmp_path, monkeypatch):
    monkeypatch.delenv("NOT_DEFINED_FOR_DRF_TEST", raising=False)
    p = tmp_path / "assets.yaml"
    p.write_text(
        "wam_runtime: ${NOT_DEFINED_FOR_DRF_TEST}/wam.usda\n"
        "l20_runtime: /tmp/l20.usda\n",
        encoding="utf-8",
    )
    with pytest.raises(ConfigError, match="LOCAL_ASSET_CONFIG_ENV_UNEXPANDED"):
        load_local_asset_config(p)


def test_local_asset_config_rejects_non_mapping_yaml(tmp_path):
    p = tmp_path / "assets.yaml"
    p.write_text("- /tmp/wam.usda\n- /tmp/l20.usda\n", encoding="utf-8")
    with pytest.raises(ConfigError, match="LOCAL_ASSET_CONFIG_ROOT_INVALID"):
        load_local_asset_config(p)


def test_local_asset_config_rejects_non_string_path(tmp_path):
    p = tmp_path / "assets.yaml"
    p.write_text("wam_runtime: 123\nl20_runtime: /tmp/l20.usda\n", encoding="utf-8")
    with pytest.raises(ConfigError, match="LOCAL_ASSET_CONFIG_PATH_INVALID"):
        load_local_asset_config(p)


def test_tabletop_task_config_contains_typed_m1_control_contract() -> None:
    from dexterous_robot.backends.isaac import load_tabletop_grasp_lift_config

    cfg = load_tabletop_grasp_lift_config(Path(__file__).resolve().parents[2] / "configs" / "tasks" / "tabletop_grasp_lift.yaml")
    assert cfg.control.approach.waypoint_duration_s == pytest.approx(1.5)
    assert len(cfg.control.approach.preshape_hand_q_rad) == 21
    assert cfg.control.grasp.release_settle_s == pytest.approx(0.2)
    assert cfg.control.grasp.preload_duration_s == pytest.approx(5.0)
    assert cfg.control.grasp.lock_ramp_duration_s == pytest.approx(3.0)
    assert cfg.control.grasp.lock_hold_duration_s == pytest.approx(1.0)
    assert cfg.control.grasp.target_squeeze_n == pytest.approx(0.45)
    assert cfg.control.lift.delta_world_z_m == pytest.approx(0.05)
    assert cfg.control.lift.minimum_object_rise_m == pytest.approx(0.025)
    assert cfg.control.hold.duration_s >= 0.5
