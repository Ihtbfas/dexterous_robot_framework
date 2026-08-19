from pathlib import Path
import pytest

from dexterous_robot.config.tasks import (
    LegacyTabletopGraspLiftConfigV1,
    TabletopGraspLiftConfig,
    TaskConfigError,
    load_tabletop_grasp_lift_config,
    load_tabletop_grasp_lift_document,
)
from dexterous_robot.motion.profiles import load_motion_profiles

ROOT = Path(__file__).resolve().parents[2]
PROFILES = load_motion_profiles(ROOT / "configs/motion/profiles.yaml")


def test_current_task_is_v2_and_profile_based() -> None:
    cfg = load_tabletop_grasp_lift_config(
        ROOT / "configs/tasks/tabletop_grasp_lift.yaml",
        motion_profiles=PROFILES,
    )
    assert isinstance(cfg, TabletopGraspLiftConfig)
    assert cfg.schema_version == 2
    assert cfg.control.approach.motion_profile == "approach_precise"
    assert cfg.control.lift.motion_profile == "carry"
    assert not hasattr(cfg.control.approach, "waypoint_duration_s")
    assert not hasattr(cfg.control.lift, "duration_s")
    assert cfg.table_top_world_z_m == pytest.approx(0.98)
    assert cfg.table_dimensions_xyz_m == pytest.approx((0.45, 0.5, 0.05))
    assert cfg.object_dimensions_xyz_m == pytest.approx((0.05, 0.05, 0.065))
    assert cfg.object_mass_kg == pytest.approx(0.05)
    assert cfg.object_position_world_m == pytest.approx((0.68, -0.14, 1.0125))
    assert cfg.object_static_friction == pytest.approx(1.0)
    assert cfg.object_dynamic_friction == pytest.approx(1.0)
    assert len(cfg.initial_wam_q_rad) == 7
    assert len(cfg.initial_hand_q_rad) == 21
    assert cfg.initial_l20_root_position_world_m == pytest.approx((0.3714205479287327, 0.14, 1.1880446148173085))


def test_legacy_v1_fixture_remains_explicitly_readable() -> None:
    doc = load_tabletop_grasp_lift_document(ROOT / "tests/fixtures/tabletop_grasp_lift_v1.yaml")
    assert isinstance(doc, LegacyTabletopGraspLiftConfigV1)
    assert doc.control.approach.waypoint_duration_s == pytest.approx(1.0)
    assert doc.control.lift.duration_s == pytest.approx(3.5)


def test_current_loader_rejects_legacy_authority() -> None:
    with pytest.raises(TaskConfigError, match="TASK_CONFIG_CURRENT_SCHEMA_REQUIRED"):
        load_tabletop_grasp_lift_config(
            ROOT / "tests/fixtures/tabletop_grasp_lift_v1.yaml",
            motion_profiles=PROFILES,
        )


def test_v2_rejects_duration_override(tmp_path: Path) -> None:
    source = (ROOT / "configs/tasks/tabletop_grasp_lift.yaml").read_text(encoding="utf-8")
    source = source.replace("    motion_profile: approach_precise\n", "    motion_profile: approach_precise\n    waypoint_duration_s: 1.0\n")
    path = tmp_path / "bad.yaml"
    path.write_text(source, encoding="utf-8")
    with pytest.raises(TaskConfigError, match="TABLETOP_APPROACH_KEYS_INVALID"):
        load_tabletop_grasp_lift_document(path)


def test_v2_rejects_lift_duration_override(tmp_path: Path) -> None:
    source = (ROOT / "configs/tasks/tabletop_grasp_lift.yaml").read_text(encoding="utf-8")
    source = source.replace("    motion_profile: carry\n", "    motion_profile: carry\n    duration_s: 3.5\n")
    path = tmp_path / "bad_lift.yaml"
    path.write_text(source, encoding="utf-8")
    with pytest.raises(TaskConfigError, match="TABLETOP_LIFT_KEYS_INVALID"):
        load_tabletop_grasp_lift_document(path)


def test_v2_rejects_unknown_profile(tmp_path: Path) -> None:
    source = (ROOT / "configs/tasks/tabletop_grasp_lift.yaml").read_text(encoding="utf-8")
    source = source.replace("motion_profile: carry", "motion_profile: missing")
    path = tmp_path / "bad.yaml"
    path.write_text(source, encoding="utf-8")
    with pytest.raises(ValueError, match="MOTION_PROFILE_NOT_FOUND:missing"):
        load_tabletop_grasp_lift_config(path, motion_profiles=PROFILES)
