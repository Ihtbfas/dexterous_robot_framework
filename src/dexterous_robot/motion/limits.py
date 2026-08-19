from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from pathlib import Path
from typing import Any, Literal, Sequence

import yaml

Authority = Literal["vendor", "project_software"]


@dataclass(frozen=True)
class LimitProvenance:
    authority: Authority
    source: str
    derived_from: str | None = None


@dataclass(frozen=True)
class ScalarLimit:
    value: float
    provenance: LimitProvenance


@dataclass(frozen=True)
class JointLimit:
    velocity: ScalarLimit
    acceleration: ScalarLimit
    jerk: ScalarLimit


@dataclass(frozen=True)
class JointKinematicLimits:
    device_model: str
    joint_names: tuple[str, ...]
    limits: tuple[JointLimit, ...]


@dataclass(frozen=True)
class CartesianKinematicLimits:
    system_id: str
    linear_velocity: ScalarLimit
    linear_acceleration: ScalarLimit
    linear_jerk: ScalarLimit


@dataclass(frozen=True)
class ResolvedJointKinematicLimits:
    joint_names: tuple[str, ...]
    velocity_rad_s: tuple[float, ...]
    acceleration_rad_s2: tuple[float, ...]
    jerk_rad_s3: tuple[float, ...]


@dataclass(frozen=True)
class ResolvedCartesianKinematicLimits:
    linear_velocity_m_s: float
    linear_acceleration_m_s2: float
    linear_jerk_m_s3: float


def _load_yaml(path: str | Path) -> dict[str, Any]:
    config_path = Path(path)
    try:
        raw = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ValueError("KINEMATIC_LIMITS_CONFIG_INVALID") from exc
    if not isinstance(raw, dict):
        raise ValueError("KINEMATIC_LIMITS_SCHEMA_INVALID")
    return raw


def _exact_keys(raw: Any, expected: set[str], error: str) -> dict[str, Any]:
    if not isinstance(raw, dict) or set(raw) != expected:
        raise ValueError(error)
    return raw


def _positive(value: Any) -> float:
    if isinstance(value, bool):
        raise ValueError("KINEMATIC_LIMIT_VALUE_INVALID")
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError("KINEMATIC_LIMIT_VALUE_INVALID") from exc
    if not isfinite(result) or result <= 0.0:
        raise ValueError("KINEMATIC_LIMIT_VALUE_INVALID")
    return result


def _provenance(raw: dict[str, Any]) -> LimitProvenance:
    authority = raw.get("authority")
    if authority not in ("vendor", "project_software"):
        raise ValueError("KINEMATIC_LIMIT_AUTHORITY_INVALID")
    source = raw.get("source")
    if not isinstance(source, str) or not source.strip():
        raise ValueError("KINEMATIC_LIMIT_SOURCE_INVALID")
    derived_from = raw.get("derived_from")
    if authority == "project_software":
        if not isinstance(derived_from, str) or not derived_from.strip():
            raise ValueError("KINEMATIC_LIMIT_DERIVATION_INVALID")
    elif derived_from is not None and (not isinstance(derived_from, str) or not derived_from.strip()):
        raise ValueError("KINEMATIC_LIMIT_DERIVATION_INVALID")
    return LimitProvenance(authority, source.strip(), None if derived_from is None else derived_from.strip())


def _scalar(raw: Any, *, value_key: str) -> ScalarLimit:
    if not isinstance(raw, dict):
        raise ValueError("KINEMATIC_LIMIT_VALUE_INVALID")
    expected = {value_key, "authority", "source"}
    if "derived_from" in raw:
        expected.add("derived_from")
    _exact_keys(raw, expected, "KINEMATIC_LIMITS_SCHEMA_INVALID")
    return ScalarLimit(_positive(raw[value_key]), _provenance(raw))


def load_joint_kinematic_limits(
    path: str | Path,
    *,
    expected_joint_names: Sequence[str],
) -> JointKinematicLimits:
    raw = _load_yaml(path)
    _exact_keys(raw, {"schema_version", "kind", "device_model", "joints"}, "KINEMATIC_LIMITS_SCHEMA_INVALID")
    if raw["schema_version"] != 1 or raw["kind"] != "JointKinematicLimits":
        raise ValueError("KINEMATIC_LIMITS_SCHEMA_INVALID")
    if not isinstance(raw["device_model"], str) or not raw["device_model"]:
        raise ValueError("KINEMATIC_LIMITS_SCHEMA_INVALID")
    expected = tuple(expected_joint_names)
    if len(expected) == 0 or len(set(expected)) != len(expected):
        raise ValueError("KINEMATIC_LIMITS_JOINT_SET_INVALID")
    joints = raw["joints"]
    if not isinstance(joints, dict) or set(joints) != set(expected):
        raise ValueError("KINEMATIC_LIMITS_JOINT_SET_INVALID")
    rows: list[JointLimit] = []
    for name in expected:
        row = _exact_keys(joints[name], {"velocity", "acceleration", "jerk"}, "KINEMATIC_LIMITS_SCHEMA_INVALID")
        rows.append(
            JointLimit(
                velocity=_scalar(row["velocity"], value_key="max_rad_s"),
                acceleration=_scalar(row["acceleration"], value_key="max_rad_s2"),
                jerk=_scalar(row["jerk"], value_key="max_rad_s3"),
            )
        )
    return JointKinematicLimits(str(raw["device_model"]), expected, tuple(rows))


def load_cartesian_kinematic_limits(path: str | Path) -> CartesianKinematicLimits:
    raw = _load_yaml(path)
    _exact_keys(raw, {"schema_version", "kind", "system_id", "linear"}, "KINEMATIC_LIMITS_SCHEMA_INVALID")
    if raw["schema_version"] != 1 or raw["kind"] != "CartesianKinematicLimits":
        raise ValueError("KINEMATIC_LIMITS_SCHEMA_INVALID")
    if not isinstance(raw["system_id"], str) or not raw["system_id"]:
        raise ValueError("KINEMATIC_LIMITS_SCHEMA_INVALID")
    linear = _exact_keys(raw["linear"], {"velocity", "acceleration", "jerk"}, "KINEMATIC_LIMITS_SCHEMA_INVALID")
    return CartesianKinematicLimits(
        system_id=raw["system_id"],
        linear_velocity=_scalar(linear["velocity"], value_key="max_m_s"),
        linear_acceleration=_scalar(linear["acceleration"], value_key="max_m_s2"),
        linear_jerk=_scalar(linear["jerk"], value_key="max_m_s3"),
    )
