from __future__ import annotations

import math
from collections.abc import Sequence

from dexterous_robot.control.math.minimum_jerk import minimum_jerk_fraction
from dexterous_robot.core import JointPositionCommand


class JointTargetRampController:
    """Pure minimum-jerk interpolation between two joint-space targets."""

    def compute(
        self,
        *,
        device_id: str,
        joint_names: Sequence[str],
        start_rad: Sequence[float],
        target_rad: Sequence[float],
        elapsed_s: float,
        duration_s: float,
        profile: str | None = None,
    ) -> JointPositionCommand:
        names = tuple(joint_names)
        start = tuple(float(v) for v in start_rad)
        target = tuple(float(v) for v in target_rad)
        if len(names) == 0 or len(start) != len(names) or len(target) != len(names):
            raise ValueError("JOINT_RAMP_WIDTH_INVALID")
        if not all(math.isfinite(v) for v in start + target):
            raise ValueError("JOINT_RAMP_TARGET_INVALID")
        elapsed = float(elapsed_s)
        duration = float(duration_s)
        if not math.isfinite(elapsed) or not math.isfinite(duration) or duration <= 0.0:
            raise ValueError("JOINT_RAMP_TIME_INVALID")
        blend = minimum_jerk_fraction(elapsed / duration)
        values = tuple(a + (b - a) * blend for a, b in zip(start, target, strict=True))
        return JointPositionCommand(device_id, names, values, profile=profile)
