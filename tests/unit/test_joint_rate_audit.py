from __future__ import annotations

import math

import pytest

from dexterous_robot.core import JointPositionCommand
from dexterous_robot.motion.audit import JointRateAudit
from dexterous_robot.motion.limits import ResolvedJointKinematicLimits


def test_joint_rate_audit_computes_velocity_and_acceleration_utilization() -> None:
    limits = ResolvedJointKinematicLimits(("j1",), (2.0,), (4.0,), (100.0,))
    audit = JointRateAudit(limits)
    audit.observe(time_s=0.0, command=JointPositionCommand("arm", ("j1",), (0.0,)))
    audit.observe(time_s=0.5, command=JointPositionCommand("arm", ("j1",), (0.5,)))
    audit.observe(time_s=1.0, command=JointPositionCommand("arm", ("j1",), (1.5,)))
    summary = audit.summary()
    assert summary.max_velocity_utilization == pytest.approx(1.0)
    assert summary.max_acceleration_utilization == pytest.approx(0.5)
    assert summary.classification == "MOTION_LIMIT_AUDIT_PASS"
    assert summary.per_joint[0].peak_velocity_rad_s == pytest.approx(2.0)
    assert summary.per_joint[0].peak_acceleration_rad_s2 == pytest.approx(2.0)


def test_joint_rate_audit_marks_project_limit_exceedance_review_required() -> None:
    limits = ResolvedJointKinematicLimits(("j1",), (1.0,), (10.0,), (100.0,))
    audit = JointRateAudit(limits)
    audit.observe(time_s=0.0, command=JointPositionCommand("arm", ("j1",), (0.0,)))
    audit.observe(time_s=0.5, command=JointPositionCommand("arm", ("j1",), (1.0,)))
    summary = audit.summary()
    assert summary.classification == "MOTION_LIMIT_AUDIT_REVIEW_REQUIRED"
    assert summary.max_velocity_utilization == pytest.approx(2.0)


def test_joint_rate_audit_rejects_non_increasing_time_and_order_mismatch() -> None:
    limits = ResolvedJointKinematicLimits(("j1", "j2"), (1.0, 1.0), (1.0, 1.0), (1.0, 1.0))
    audit = JointRateAudit(limits)
    audit.observe(time_s=0.0, command=JointPositionCommand("arm", ("j1", "j2"), (0.0, 0.0)))
    with pytest.raises(ValueError, match="MOTION_AUDIT_TIME_INVALID"):
        audit.observe(time_s=0.0, command=JointPositionCommand("arm", ("j1", "j2"), (0.0, 0.0)))
    audit = JointRateAudit(limits)
    with pytest.raises(ValueError, match="MOTION_AUDIT_JOINT_ORDER_INVALID"):
        audit.observe(time_s=0.0, command=JointPositionCommand("arm", ("j2", "j1"), (0.0, 0.0)))


def test_joint_rate_audit_rejects_non_arm_and_nonfinite_time() -> None:
    limits = ResolvedJointKinematicLimits(("j1",), (1.0,), (1.0,), (1.0,))
    audit = JointRateAudit(limits)
    with pytest.raises(ValueError, match="MOTION_AUDIT_DEVICE_INVALID"):
        audit.observe(time_s=0.0, command=JointPositionCommand("hand", ("j1",), (0.0,)))
    with pytest.raises(ValueError, match="MOTION_AUDIT_TIME_INVALID"):
        audit.observe(time_s=math.nan, command=JointPositionCommand("arm", ("j1",), (0.0,)))
