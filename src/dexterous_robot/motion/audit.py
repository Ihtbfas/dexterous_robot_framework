from __future__ import annotations

from dataclasses import asdict, dataclass
import math

from dexterous_robot.core import JointPositionCommand

from .limits import ResolvedJointKinematicLimits


@dataclass(frozen=True)
class JointRateEvidence:
    joint_name: str
    peak_velocity_rad_s: float
    peak_acceleration_rad_s2: float
    max_velocity_utilization: float
    max_acceleration_utilization: float


@dataclass(frozen=True)
class JointRateAuditSummary:
    per_joint: tuple[JointRateEvidence, ...]
    max_velocity_utilization: float
    max_acceleration_utilization: float
    classification: str


class JointRateAudit:
    """Backend-neutral finite-difference audit of commanded arm joint targets."""

    def __init__(self, limits: ResolvedJointKinematicLimits) -> None:
        if not isinstance(limits, ResolvedJointKinematicLimits):
            raise ValueError("MOTION_AUDIT_LIMITS_INVALID")
        width = len(limits.joint_names)
        if width == 0 or not (
            len(limits.velocity_rad_s) == width
            and len(limits.acceleration_rad_s2) == width
            and len(limits.jerk_rad_s3) == width
        ):
            raise ValueError("MOTION_AUDIT_LIMITS_INVALID")
        numeric = limits.velocity_rad_s + limits.acceleration_rad_s2 + limits.jerk_rad_s3
        if not all(math.isfinite(value) and value > 0.0 for value in numeric):
            raise ValueError("MOTION_AUDIT_LIMITS_INVALID")
        self._limits = limits
        self._last_time_s: float | None = None
        self._last_q: tuple[float, ...] | None = None
        self._last_velocity: tuple[float, ...] | None = None
        self._peak_velocity = [0.0] * width
        self._peak_acceleration = [0.0] * width

    def observe(self, *, time_s: float, command: JointPositionCommand) -> None:
        if not isinstance(command, JointPositionCommand) or command.device_id != "arm":
            raise ValueError("MOTION_AUDIT_DEVICE_INVALID")
        if tuple(command.joint_names) != self._limits.joint_names:
            raise ValueError("MOTION_AUDIT_JOINT_ORDER_INVALID")
        try:
            time_value = float(time_s)
            q = tuple(float(value) for value in command.position_rad)
        except (TypeError, ValueError) as exc:
            raise ValueError("MOTION_AUDIT_COMMAND_INVALID") from exc
        if not math.isfinite(time_value):
            raise ValueError("MOTION_AUDIT_TIME_INVALID")
        if len(q) != len(self._limits.joint_names) or not all(math.isfinite(value) for value in q):
            raise ValueError("MOTION_AUDIT_COMMAND_INVALID")

        if self._last_time_s is not None:
            assert self._last_q is not None
            dt_s = time_value - self._last_time_s
            if not math.isfinite(dt_s) or dt_s <= 0.0:
                raise ValueError("MOTION_AUDIT_TIME_INVALID")
            velocity = tuple(
                (current - previous) / dt_s
                for current, previous in zip(q, self._last_q, strict=True)
            )
            for index, value in enumerate(velocity):
                self._peak_velocity[index] = max(self._peak_velocity[index], abs(value))
            if self._last_velocity is not None:
                acceleration = tuple(
                    (current - previous) / dt_s
                    for current, previous in zip(velocity, self._last_velocity, strict=True)
                )
                for index, value in enumerate(acceleration):
                    self._peak_acceleration[index] = max(self._peak_acceleration[index], abs(value))
            self._last_velocity = velocity

        self._last_time_s = time_value
        self._last_q = q

    def summary(self) -> JointRateAuditSummary:
        rows: list[JointRateEvidence] = []
        max_velocity_utilization = 0.0
        max_acceleration_utilization = 0.0
        for index, name in enumerate(self._limits.joint_names):
            velocity_utilization = self._peak_velocity[index] / self._limits.velocity_rad_s[index]
            acceleration_utilization = self._peak_acceleration[index] / self._limits.acceleration_rad_s2[index]
            rows.append(
                JointRateEvidence(
                    joint_name=name,
                    peak_velocity_rad_s=self._peak_velocity[index],
                    peak_acceleration_rad_s2=self._peak_acceleration[index],
                    max_velocity_utilization=velocity_utilization,
                    max_acceleration_utilization=acceleration_utilization,
                )
            )
            max_velocity_utilization = max(max_velocity_utilization, velocity_utilization)
            max_acceleration_utilization = max(max_acceleration_utilization, acceleration_utilization)
        classification = (
            "MOTION_LIMIT_AUDIT_PASS"
            if max_velocity_utilization <= 1.0 and max_acceleration_utilization <= 1.0
            else "MOTION_LIMIT_AUDIT_REVIEW_REQUIRED"
        )
        return JointRateAuditSummary(
            per_joint=tuple(rows),
            max_velocity_utilization=max_velocity_utilization,
            max_acceleration_utilization=max_acceleration_utilization,
            classification=classification,
        )

    def summary_as_dict(self) -> dict[str, object]:
        return asdict(self.summary())
