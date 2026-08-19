from pathlib import Path
import pytest

from dexterous_robot.devices.arms.wam7 import WAM7_JOINT_NAMES
from dexterous_robot.motion.limits import (
    CartesianKinematicLimits,
    JointKinematicLimits,
    load_cartesian_kinematic_limits,
    load_joint_kinematic_limits,
)

ROOT = Path(__file__).resolve().parents[2]


def test_tracked_wam7_limits_are_project_software_and_exact_joint_set() -> None:
    limits = load_joint_kinematic_limits(
        ROOT / "configs/devices/arms/wam7_kinematic_limits.yaml",
        expected_joint_names=WAM7_JOINT_NAMES,
    )
    assert isinstance(limits, JointKinematicLimits)
    assert limits.joint_names == WAM7_JOINT_NAMES
    assert all(row.velocity.provenance.authority == "project_software" for row in limits.limits)
    assert all(row.acceleration.provenance.derived_from == "m1.6-motion-pacing-height-v1" for row in limits.limits)
    assert all(row.jerk.provenance.derived_from == "m1.6-motion-pacing-height-v1" for row in limits.limits)


def test_joint_limits_reject_missing_joint(tmp_path: Path) -> None:
    source = (ROOT / "configs/devices/arms/wam7_kinematic_limits.yaml").read_text(encoding="utf-8")
    source = source.replace("  wam_j7_joint:\n", "  removed_j7_joint:\n", 1)
    path = tmp_path / "bad.yaml"
    path.write_text(source, encoding="utf-8")
    with pytest.raises(ValueError, match="KINEMATIC_LIMITS_JOINT_SET_INVALID"):
        load_joint_kinematic_limits(path, expected_joint_names=WAM7_JOINT_NAMES)


def test_tracked_cartesian_limits_are_project_software() -> None:
    limits = load_cartesian_kinematic_limits(ROOT / "configs/motion/cartesian_limits.yaml")
    assert isinstance(limits, CartesianKinematicLimits)
    assert limits.linear_velocity.provenance.authority == "project_software"
