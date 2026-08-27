from __future__ import annotations

import math
from dataclasses import dataclass


class MuJoCoTimingError(ValueError):
    """Raised when Runtime dt cannot be represented by exact MuJoCo substeps."""


def _positive_finite(value: object, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise MuJoCoTimingError(f"{label}_INVALID") from exc
    if not math.isfinite(result) or result <= 0.0:
        raise MuJoCoTimingError(f"{label}_INVALID")
    return result


def _nonnegative_finite(value: object, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise MuJoCoTimingError(f"{label}_INVALID") from exc
    if not math.isfinite(result) or result < 0.0:
        raise MuJoCoTimingError(f"{label}_INVALID")
    return result


@dataclass(frozen=True)
class MuJoCoTimingResolution:
    runtime_dt_s: float
    physics_timestep_s: float
    tolerance_s: float
    substeps: int
    represented_dt_s: float
    representation_error_s: float


def resolve_substeps(
    runtime_dt_s: float,
    physics_timestep_s: float,
    tolerance_s: float,
) -> int:
    """Resolve one Runtime cycle to an exact integer MuJoCo step count.

    No fractional accumulator is permitted in M2-B1. If a Runtime period cannot
    be represented within the configured absolute tolerance, fail closed.
    """
    runtime_dt = _positive_finite(runtime_dt_s, "MUJOCO_RUNTIME_DT")
    physics_dt = _positive_finite(
        physics_timestep_s, "MUJOCO_PHYSICS_TIMESTEP"
    )
    tolerance = _nonnegative_finite(
        tolerance_s, "MUJOCO_RUNTIME_DT_TOLERANCE"
    )

    nearest = int(round(runtime_dt / physics_dt))
    if nearest < 1:
        raise MuJoCoTimingError(
            "MUJOCO_RUNTIME_DT_BELOW_PHYSICS_TIMESTEP:"
            f"runtime={runtime_dt}:physics={physics_dt}"
        )

    represented = nearest * physics_dt
    error = abs(runtime_dt - represented)
    if error > tolerance:
        raise MuJoCoTimingError(
            "MUJOCO_RUNTIME_DT_NOT_INTEGER_MULTIPLE:"
            f"runtime={runtime_dt}:physics={physics_dt}:"
            f"nearest_substeps={nearest}:error={error}:tolerance={tolerance}"
        )
    return nearest


def describe_substeps(
    runtime_dt_s: float,
    physics_timestep_s: float,
    tolerance_s: float,
) -> MuJoCoTimingResolution:
    runtime_dt = _positive_finite(runtime_dt_s, "MUJOCO_RUNTIME_DT")
    physics_dt = _positive_finite(
        physics_timestep_s, "MUJOCO_PHYSICS_TIMESTEP"
    )
    tolerance = _nonnegative_finite(
        tolerance_s, "MUJOCO_RUNTIME_DT_TOLERANCE"
    )
    substeps = resolve_substeps(runtime_dt, physics_dt, tolerance)
    represented = substeps * physics_dt
    return MuJoCoTimingResolution(
        runtime_dt_s=runtime_dt,
        physics_timestep_s=physics_dt,
        tolerance_s=tolerance,
        substeps=substeps,
        represented_dt_s=represented,
        representation_error_s=abs(runtime_dt - represented),
    )
