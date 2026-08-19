"""Backend-neutral motion limits, profiles, timing, and audit utilities."""

from .audit import JointRateAudit, JointRateAuditSummary, JointRateEvidence
from .limits import (
    CartesianKinematicLimits,
    JointKinematicLimits,
    LimitProvenance,
    ResolvedCartesianKinematicLimits,
    ResolvedJointKinematicLimits,
    ScalarLimit,
    load_cartesian_kinematic_limits,
    load_joint_kinematic_limits,
)
from .profiles import (
    CartesianMotionProfile,
    JointMotionProfile,
    MotionProfiles,
    load_motion_profiles,
    resolve_cartesian_limits,
    resolve_joint_limits,
)
from .timing import (
    JointTimingResult,
    ScalarTimingResult,
    minimum_jerk_duration,
    minimum_jerk_joint_duration,
)

__all__ = [
    "JointRateAudit", "JointRateAuditSummary", "JointRateEvidence",
    "CartesianKinematicLimits", "JointKinematicLimits", "LimitProvenance",
    "ResolvedCartesianKinematicLimits", "ResolvedJointKinematicLimits", "ScalarLimit",
    "load_cartesian_kinematic_limits", "load_joint_kinematic_limits",
    "CartesianMotionProfile", "JointMotionProfile", "MotionProfiles",
    "load_motion_profiles", "resolve_cartesian_limits", "resolve_joint_limits",
    "JointTimingResult", "ScalarTimingResult", "minimum_jerk_duration", "minimum_jerk_joint_duration",
]
