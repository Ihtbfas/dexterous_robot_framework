import pytest

from dexterous_robot.motion.limits import ResolvedJointKinematicLimits
from dexterous_robot.motion.timing import minimum_jerk_duration, minimum_jerk_joint_duration


def test_minimum_jerk_duration_selects_each_constraint() -> None:
    velocity = minimum_jerk_duration(
        1.0, max_velocity=1.0, max_acceleration=100.0, max_jerk=1000.0, minimum_duration_s=0.0
    )
    assert velocity.duration_s == pytest.approx(1.875)
    assert velocity.limiting_constraint == "velocity"

    acceleration = minimum_jerk_duration(
        1.0, max_velocity=100.0, max_acceleration=1.0, max_jerk=1000.0, minimum_duration_s=0.0
    )
    assert acceleration.duration_s == pytest.approx((5.773502691896257) ** 0.5)
    assert acceleration.limiting_constraint == "acceleration"

    jerk = minimum_jerk_duration(
        1.0, max_velocity=100.0, max_acceleration=100.0, max_jerk=1.0, minimum_duration_s=0.0
    )
    assert jerk.duration_s == pytest.approx(60.0 ** (1.0 / 3.0))
    assert jerk.limiting_constraint == "jerk"


def test_zero_motion_uses_explicit_minimum_duration() -> None:
    result = minimum_jerk_duration(
        0.0, max_velocity=1.0, max_acceleration=1.0, max_jerk=1.0, minimum_duration_s=1.0 / 120.0
    )
    assert result.duration_s == pytest.approx(1.0 / 120.0)
    assert result.limiting_constraint == "minimum_duration"


def test_joint_timing_uses_slowest_joint_and_same_segment_duration() -> None:
    limits = ResolvedJointKinematicLimits(
        joint_names=("j1", "j2"),
        velocity_rad_s=(1.0, 0.5),
        acceleration_rad_s2=(10.0, 10.0),
        jerk_rad_s3=(100.0, 100.0),
    )
    result = minimum_jerk_joint_duration(
        (0.0, 0.0), (0.2, 0.2), limits, minimum_duration_s=0.01
    )
    assert result.limiting_joint == "j2"
    assert result.duration_s >= 1.875 * 0.2 / 0.5


def test_zero_joint_motion_reports_minimum_duration_and_no_joint() -> None:
    limits = ResolvedJointKinematicLimits(
        joint_names=("j1", "j2"),
        velocity_rad_s=(1.0, 1.0),
        acceleration_rad_s2=(1.0, 1.0),
        jerk_rad_s3=(1.0, 1.0),
    )
    result = minimum_jerk_joint_duration((0.0, 0.0), (0.0, 0.0), limits, minimum_duration_s=0.01)
    assert result.duration_s == pytest.approx(0.01)
    assert result.limiting_joint == "none"
    assert result.limiting_constraint == "minimum_duration"
