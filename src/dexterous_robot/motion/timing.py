from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass
import math

from .limits import ResolvedJointKinematicLimits

_MIN_JERK_PEAK_V = 1.875
_MIN_JERK_PEAK_A = 5.773502691896257
_MIN_JERK_PEAK_J = 60.0


@dataclass(frozen=True)
class ScalarTimingResult:
    duration_s: float
    limiting_constraint: str


@dataclass(frozen=True)
class JointTimingResult:
    duration_s: float
    limiting_joint: str
    limiting_constraint: str


def minimum_jerk_duration(
    displacement: float,
    *,
    max_velocity: float,
    max_acceleration: float,
    max_jerk: float,
    minimum_duration_s: float = 0.0,
) -> ScalarTimingResult:
    try:
        d = abs(float(displacement))
        v = float(max_velocity)
        a = float(max_acceleration)
        j = float(max_jerk)
        floor = float(minimum_duration_s)
    except (TypeError, ValueError) as exc:
        raise ValueError("MOTION_TIMING_INPUT_INVALID") from exc
    if not all(math.isfinite(x) for x in (d, v, a, j, floor)) or v <= 0.0 or a <= 0.0 or j <= 0.0 or floor < 0.0:
        raise ValueError("MOTION_TIMING_INPUT_INVALID")
    candidates = (
        ("velocity", _MIN_JERK_PEAK_V * d / v),
        ("acceleration", math.sqrt(_MIN_JERK_PEAK_A * d / a)),
        ("jerk", (_MIN_JERK_PEAK_J * d / j) ** (1.0 / 3.0)),
        ("minimum_duration", floor),
    )
    constraint, duration = max(candidates, key=lambda row: row[1])
    return ScalarTimingResult(float(duration), constraint)


def minimum_jerk_joint_duration(
    start_rad: Sequence[float],
    target_rad: Sequence[float],
    limits: ResolvedJointKinematicLimits,
    *,
    minimum_duration_s: float = 0.0,
) -> JointTimingResult:
    if not isinstance(limits, ResolvedJointKinematicLimits):
        raise ValueError("MOTION_JOINT_TIMING_LIMITS_INVALID")
    try:
        start = tuple(float(v) for v in start_rad)
        target = tuple(float(v) for v in target_rad)
    except (TypeError, ValueError) as exc:
        raise ValueError("MOTION_JOINT_TIMING_WIDTH_INVALID") from exc
    width = len(limits.joint_names)
    if width == 0 or len(start) != width or len(target) != width:
        raise ValueError("MOTION_JOINT_TIMING_WIDTH_INVALID")
    if not (
        len(limits.velocity_rad_s) == width
        and len(limits.acceleration_rad_s2) == width
        and len(limits.jerk_rad_s3) == width
    ):
        raise ValueError("MOTION_JOINT_TIMING_WIDTH_INVALID")
    if not all(math.isfinite(v) for v in start + target):
        raise ValueError("MOTION_TIMING_INPUT_INVALID")

    if all(abs(b - a) == 0.0 for a, b in zip(start, target, strict=True)):
        floor = minimum_jerk_duration(
            0.0,
            max_velocity=limits.velocity_rad_s[0],
            max_acceleration=limits.acceleration_rad_s2[0],
            max_jerk=limits.jerk_rad_s3[0],
            minimum_duration_s=minimum_duration_s,
        )
        return JointTimingResult(floor.duration_s, "none", "minimum_duration")

    best: tuple[float, str, str] | None = None
    for name, a0, a1, vmax, amax, jmax in zip(
        limits.joint_names,
        start,
        target,
        limits.velocity_rad_s,
        limits.acceleration_rad_s2,
        limits.jerk_rad_s3,
        strict=True,
    ):
        result = minimum_jerk_duration(
            a1 - a0,
            max_velocity=vmax,
            max_acceleration=amax,
            max_jerk=jmax,
            minimum_duration_s=minimum_duration_s,
        )
        candidate = (result.duration_s, name, result.limiting_constraint)
        if best is None or candidate[0] > best[0]:
            best = candidate
    assert best is not None
    return JointTimingResult(best[0], best[1], best[2])
