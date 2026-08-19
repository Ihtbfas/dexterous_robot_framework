from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from pathlib import Path
from typing import Any

import yaml

from .limits import (
    CartesianKinematicLimits,
    JointKinematicLimits,
    ResolvedCartesianKinematicLimits,
    ResolvedJointKinematicLimits,
)


@dataclass(frozen=True)
class JointMotionProfile:
    name: str
    velocity_scale: float
    acceleration_scale: float
    jerk_scale: float


@dataclass(frozen=True)
class CartesianMotionProfile:
    name: str
    linear_velocity_scale: float
    linear_acceleration_scale: float
    linear_jerk_scale: float


@dataclass(frozen=True)
class MotionProfiles:
    joint_profiles: tuple[JointMotionProfile, ...]
    cartesian_profiles: tuple[CartesianMotionProfile, ...]

    def joint(self, name: str) -> JointMotionProfile:
        for profile in self.joint_profiles:
            if profile.name == name:
                return profile
        raise ValueError(f"MOTION_PROFILE_NOT_FOUND:{name}")

    def cartesian(self, name: str) -> CartesianMotionProfile:
        for profile in self.cartesian_profiles:
            if profile.name == name:
                return profile
        raise ValueError(f"MOTION_PROFILE_NOT_FOUND:{name}")


def _scale(value: Any) -> float:
    if isinstance(value, bool):
        raise ValueError("MOTION_PROFILE_SCALE_INVALID")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError("MOTION_PROFILE_SCALE_INVALID") from exc
    if not isfinite(result) or result <= 0.0 or result > 1.0:
        raise ValueError("MOTION_PROFILE_SCALE_INVALID")
    return result


def load_motion_profiles(path: str | Path) -> MotionProfiles:
    try:
        raw = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError("MOTION_PROFILE_CONFIG_INVALID") from exc
    if not isinstance(raw, dict) or set(raw) != {"schema_version", "kind", "joint_profiles", "cartesian_profiles"}:
        raise ValueError("MOTION_PROFILE_SCHEMA_INVALID")
    if raw["schema_version"] != 1 or raw["kind"] != "MotionProfiles":
        raise ValueError("MOTION_PROFILE_SCHEMA_INVALID")
    joint_raw = raw["joint_profiles"]
    cart_raw = raw["cartesian_profiles"]
    if not isinstance(joint_raw, dict) or not isinstance(cart_raw, dict):
        raise ValueError("MOTION_PROFILE_SCHEMA_INVALID")
    joints: list[JointMotionProfile] = []
    for name, values in joint_raw.items():
        if not isinstance(name, str) or not name or not isinstance(values, dict) or set(values) != {"velocity_scale", "acceleration_scale", "jerk_scale"}:
            raise ValueError("MOTION_PROFILE_SCHEMA_INVALID")
        joints.append(
            JointMotionProfile(
                name,
                _scale(values["velocity_scale"]),
                _scale(values["acceleration_scale"]),
                _scale(values["jerk_scale"]),
            )
        )
    carts: list[CartesianMotionProfile] = []
    for name, values in cart_raw.items():
        if not isinstance(name, str) or not name or not isinstance(values, dict) or set(values) != {"linear_velocity_scale", "linear_acceleration_scale", "linear_jerk_scale"}:
            raise ValueError("MOTION_PROFILE_SCHEMA_INVALID")
        carts.append(
            CartesianMotionProfile(
                name,
                _scale(values["linear_velocity_scale"]),
                _scale(values["linear_acceleration_scale"]),
                _scale(values["linear_jerk_scale"]),
            )
        )
    return MotionProfiles(tuple(joints), tuple(carts))


def resolve_joint_limits(limits: JointKinematicLimits, profile: JointMotionProfile) -> ResolvedJointKinematicLimits:
    return ResolvedJointKinematicLimits(
        joint_names=limits.joint_names,
        velocity_rad_s=tuple(row.velocity.value * profile.velocity_scale for row in limits.limits),
        acceleration_rad_s2=tuple(row.acceleration.value * profile.acceleration_scale for row in limits.limits),
        jerk_rad_s3=tuple(row.jerk.value * profile.jerk_scale for row in limits.limits),
    )


def resolve_cartesian_limits(limits: CartesianKinematicLimits, profile: CartesianMotionProfile) -> ResolvedCartesianKinematicLimits:
    return ResolvedCartesianKinematicLimits(
        linear_velocity_m_s=limits.linear_velocity.value * profile.linear_velocity_scale,
        linear_acceleration_m_s2=limits.linear_acceleration.value * profile.linear_acceleration_scale,
        linear_jerk_m_s3=limits.linear_jerk.value * profile.linear_jerk_scale,
    )
