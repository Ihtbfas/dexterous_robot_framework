from __future__ import annotations

from pathlib import Path

from dexterous_robot.config.tasks import LegacyTabletopGraspLiftConfigV1, load_tabletop_grasp_lift_document


def test_m1_6_r1_motion_pacing_height_profile_remains_readable_as_frozen_v1_fixture() -> None:
    root = Path(__file__).resolve().parents[2]
    cfg = load_tabletop_grasp_lift_document(root / "tests" / "fixtures" / "tabletop_grasp_lift_v1.yaml")
    assert isinstance(cfg, LegacyTabletopGraspLiftConfigV1)

    assert cfg.control.approach.waypoint_duration_s == 1.0
    assert cfg.control.approach.preshape_duration_s == 1.0
    assert cfg.control.approach.settle_duration_s == 0.2

    assert cfg.control.grasp.release_settle_s == 0.2
    assert cfg.control.grasp.preload_duration_s == 3.0
    assert cfg.control.grasp.lock_ramp_duration_s == 1.5
    assert cfg.control.grasp.lock_hold_duration_s == 0.5

    assert cfg.control.lift.delta_world_z_m == 0.08
    assert cfg.control.lift.duration_s == 3.5
    assert cfg.control.hold.duration_s == 1.0

    assert cfg.control.lift.minimum_object_rise_m == 0.025
    assert cfg.control.lift.max_table_normal_n == 0.1
    assert cfg.control.lift.max_relative_drift_m == 0.03
    assert cfg.control.hold.minimum_clearance_m == 0.001
    assert cfg.control.hold.max_table_normal_n == 0.1
    assert cfg.control.hold.max_relative_drift_m == 0.03
