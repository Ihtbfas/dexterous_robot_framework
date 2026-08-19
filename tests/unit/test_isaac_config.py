from __future__ import annotations

from pathlib import Path

import pytest


def test_tracked_isaac_backend_config_is_typed_and_contains_no_local_asset_paths():
    from dexterous_robot.backends.isaac.config import load_isaac_backend_config

    root = Path(__file__).resolve().parents[2]
    cfg = load_isaac_backend_config(root / "configs/backends/isaac.yaml")

    assert cfg.physics_dt_s == pytest.approx(1.0 / 120.0)
    assert cfg.stage_load_timeout_s == 30.0
    assert cfg.transform_sync.update_to_fast_cache is True
    assert cfg.transform_sync.update_to_usd is True
    assert cfg.transform_sync.position_tolerance_m == 0.001
    assert not hasattr(cfg.transform_sync, "probe_delta_z_m")
    assert len(cfg.arm_carry_position_drive.stiffness) == 7
    assert len(cfg.arm_carry_position_drive.damping) == 7
    assert len(cfg.arm_carry_position_drive.max_force) == 7
    assert len(cfg.hand_open_hold.stiffness_usd_per_degree) == 21
    assert len(cfg.hand_open_hold.damping_usd_per_degree_per_second) == 21
    assert len(cfg.hand_open_hold.max_force_nm) == 21
    assert len(cfg.hand_open_hold.max_joint_velocity_deg_s) == 21
    text = (root / "configs/backends/isaac.yaml").read_text(encoding="utf-8")
    assert "/home/lyf/" not in text


def test_isaac_config_loader_rejects_unknown_schema(tmp_path: Path):
    from dexterous_robot.backends.isaac.config import IsaacConfigError, load_isaac_backend_config

    path = tmp_path / "bad.yaml"
    path.write_text("schema_version: 99\nkind: IsaacBackend\n", encoding="utf-8")
    with pytest.raises(IsaacConfigError):
        load_isaac_backend_config(path)


def test_r8_backend_config_has_distinct_open_and_grasp_lock_hand_profiles() -> None:
    from dexterous_robot.backends.isaac.config import load_isaac_backend_config

    cfg = load_isaac_backend_config(Path(__file__).resolve().parents[2] / "configs" / "backends" / "isaac.yaml")
    assert cfg.hand_open_hold.drive_type == "force"
    assert cfg.hand_grasp_lock.drive_type == "force"
    assert len(cfg.hand_grasp_lock.max_force_nm) == 21
    assert cfg.hand_grasp_lock.max_force_nm[0] == pytest.approx(0.12)
    assert cfg.hand_grasp_lock.max_force_nm[1] == pytest.approx(0.1087)
    assert cfg.hand_grasp_lock.max_force_nm[13] == pytest.approx(0.04291625)
    assert cfg.hand_grasp_lock.max_force_nm[14] == pytest.approx(0.02271125)
    assert cfg.hand_grasp_lock.max_force_nm[15] == pytest.approx(0.00879875)
    assert cfg.hand_grasp_lock.damping_usd_per_degree_per_second[3] == pytest.approx(
        2.0 * cfg.hand_open_hold.damping_usd_per_degree_per_second[3]
    )
    assert cfg.hand_grasp_lock.damping_usd_per_degree_per_second[4] == pytest.approx(
        2.0 * cfg.hand_open_hold.damping_usd_per_degree_per_second[4]
    )
