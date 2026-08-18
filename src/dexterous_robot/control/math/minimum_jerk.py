from __future__ import annotations

import math
from numbers import Real


def _finite(value: object, error: str) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(error)
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(error)
    return result


def minimum_jerk_fraction(u: float) -> float:
    """Return the clamped quintic minimum-jerk blend for normalized time."""
    normalized = _finite(u, "MINIMUM_JERK_INPUT_INVALID")
    normalized = min(1.0, max(0.0, normalized))
    return float(10.0 * normalized**3 - 15.0 * normalized**4 + 6.0 * normalized**5)


def minimum_jerk_position(start: float, target: float, elapsed_s: float, duration_s: float) -> float:
    """Interpolate one scalar with endpoint-clamped minimum-jerk timing."""
    start_value = _finite(start, "MINIMUM_JERK_INPUT_INVALID")
    target_value = _finite(target, "MINIMUM_JERK_INPUT_INVALID")
    elapsed = _finite(elapsed_s, "MINIMUM_JERK_INPUT_INVALID")
    duration = _finite(duration_s, "MINIMUM_JERK_DURATION_INVALID")
    if duration <= 0.0:
        raise ValueError("MINIMUM_JERK_DURATION_INVALID")
    blend = minimum_jerk_fraction(elapsed / duration)
    return float(start_value + (target_value - start_value) * blend)
