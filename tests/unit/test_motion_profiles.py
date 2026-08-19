from pathlib import Path
import pytest

from dexterous_robot.motion.profiles import load_motion_profiles

ROOT = Path(__file__).resolve().parents[2]


def test_tracked_profiles_have_only_m17_required_profiles() -> None:
    profiles = load_motion_profiles(ROOT / "configs/motion/profiles.yaml")
    assert profiles.joint("approach_precise").velocity_scale == pytest.approx(0.8)
    assert profiles.cartesian("carry").linear_velocity_scale == pytest.approx(0.8)
    assert {p.name for p in profiles.joint_profiles} == {"approach_precise"}
    assert {p.name for p in profiles.cartesian_profiles} == {"carry"}


def test_profile_scale_above_one_is_rejected(tmp_path: Path) -> None:
    path = tmp_path / "profiles.yaml"
    path.write_text(
        "schema_version: 1\nkind: MotionProfiles\n"
        "joint_profiles:\n  approach_precise:\n    velocity_scale: 1.01\n    acceleration_scale: 0.8\n    jerk_scale: 0.8\n"
        "cartesian_profiles:\n  carry:\n    linear_velocity_scale: 0.8\n    linear_acceleration_scale: 0.8\n    linear_jerk_scale: 0.8\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="MOTION_PROFILE_SCALE_INVALID"):
        load_motion_profiles(path)
