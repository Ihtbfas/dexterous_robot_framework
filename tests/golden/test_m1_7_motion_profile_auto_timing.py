from __future__ import annotations

from pathlib import Path

import pytest

from dexterous_robot.config.tasks import load_tabletop_grasp_lift_config
from dexterous_robot.devices.arms.wam7 import WAM7_JOINT_NAMES
from dexterous_robot.motion import (
    load_cartesian_kinematic_limits,
    load_joint_kinematic_limits,
    load_motion_profiles,
    minimum_jerk_duration,
    minimum_jerk_joint_duration,
    resolve_cartesian_limits,
    resolve_joint_limits,
)


def test_m1_7_motion_profile_auto_timing_contract_is_deterministic() -> None:
    root = Path(__file__).resolve().parents[2]
    profiles = load_motion_profiles(root / "configs/motion/profiles.yaml")
    cfg = load_tabletop_grasp_lift_config(
        root / "configs/tasks/tabletop_grasp_lift.yaml",
        motion_profiles=profiles,
    )
    assert cfg.schema_version == 2
    assert cfg.control.approach.motion_profile == "approach_precise"
    assert cfg.control.lift.motion_profile == "carry"
    assert not hasattr(cfg.control.approach, "waypoint_duration_s")
    assert not hasattr(cfg.control.lift, "duration_s")

    base_joint = load_joint_kinematic_limits(
        root / "configs/devices/arms/wam7_kinematic_limits.yaml",
        expected_joint_names=WAM7_JOINT_NAMES,
    )
    assert all(row.velocity.provenance.authority == "project_software" for row in base_joint.limits)
    assert all(row.acceleration.provenance.authority == "project_software" for row in base_joint.limits)
    assert all(row.jerk.provenance.authority == "project_software" for row in base_joint.limits)
    assert all(row.velocity.provenance.derived_from == "m1.6-motion-pacing-height-v1" for row in base_joint.limits)
    assert all(row.acceleration.provenance.derived_from == "m1.6-motion-pacing-height-v1" for row in base_joint.limits)
    assert all(row.jerk.provenance.derived_from == "m1.6-motion-pacing-height-v1" for row in base_joint.limits)

    effective_joint = resolve_joint_limits(base_joint, profiles.joint("approach_precise"))
    assert effective_joint.velocity_rad_s == pytest.approx(
        (
            1.5976937735625,
            0.6174378930000001,
            1.7595628340625,
            0.8461084608749999,
            0.5582643524999998,
            1.2435469010625,
            1.2435469010625,
        ),
        abs=1.0e-12,
    )

    starts = (
        cfg.initial_wam_q_rad,
        cfg.control.approach.lateral_ready_q_rad,
        cfg.control.approach.transit_q_rad,
        cfg.control.approach.pregrasp_q_rad,
    )
    targets = (
        cfg.control.approach.lateral_ready_q_rad,
        cfg.control.approach.transit_q_rad,
        cfg.control.approach.pregrasp_q_rad,
        cfg.control.approach.grasp_q_rad,
    )
    durations = tuple(
        minimum_jerk_joint_duration(start, target, effective_joint).duration_s
        for start, target in zip(starts, targets, strict=True)
    )
    assert durations == pytest.approx(
        (1.0, 1.0, 0.7851896449895320, 0.6786636214802383),
        abs=1.0e-12,
    )

    base_cartesian = load_cartesian_kinematic_limits(root / "configs/motion/cartesian_limits.yaml")
    assert base_cartesian.linear_velocity.provenance.authority == "project_software"
    assert base_cartesian.linear_acceleration.provenance.authority == "project_software"
    assert base_cartesian.linear_jerk.provenance.authority == "project_software"
    assert base_cartesian.linear_velocity.provenance.derived_from == "m1.6-motion-pacing-height-v1"
    effective_cartesian = resolve_cartesian_limits(base_cartesian, profiles.cartesian("carry"))
    lift_timing = minimum_jerk_duration(
        cfg.control.lift.delta_world_z_m,
        max_velocity=effective_cartesian.linear_velocity_m_s,
        max_acceleration=effective_cartesian.linear_acceleration_m_s2,
        max_jerk=effective_cartesian.linear_jerk_m_s3,
    )
    assert lift_timing.duration_s == pytest.approx(3.5, abs=1.0e-12)
