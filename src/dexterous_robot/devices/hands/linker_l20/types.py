from __future__ import annotations

import math
from collections.abc import Mapping
from dataclasses import dataclass
from numbers import Real
from types import MappingProxyType

L20_ACTIVE_CHANNELS: tuple[str, ...] = (
    "thumb_roll",
    "thumb_yaw",
    "thumb_root_flex",
    "thumb_tip_flex",
    "index_yaw",
    "index_root_flex",
    "index_tip_flex",
    "middle_yaw",
    "middle_root_flex",
    "middle_tip_flex",
    "ring_yaw",
    "ring_root_flex",
    "ring_tip_flex",
    "little_yaw",
    "little_root_flex",
    "little_tip_flex",
)

L20_PHYSICAL_JOINTS: tuple[str, ...] = (
    "thumb_joint0",
    "thumb_joint1",
    "thumb_joint2",
    "thumb_joint3",
    "thumb_joint4",
    "index_joint0",
    "index_joint1",
    "index_joint2",
    "index_joint3",
    "middle_joint0",
    "middle_joint1",
    "middle_joint2",
    "middle_joint3",
    "ring_joint0",
    "ring_joint1",
    "ring_joint2",
    "ring_joint3",
    "little_joint0",
    "little_joint1",
    "little_joint2",
    "little_joint3",
)

_MAX_SEQUENCE_ID = 2**63 - 1


def _finite_real(value: object, error: str, *, minimum: float | None = None) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise ValueError(error)
    try:
        result = float(value)
    except (OverflowError, TypeError, ValueError) as exc:
        raise ValueError(error) from exc
    if not math.isfinite(result) or (minimum is not None and result < minimum):
        raise ValueError(error)
    return result


def _sequence_id(value: object) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= _MAX_SEQUENCE_ID:
        raise ValueError("L20_SEQUENCE_ID_INVALID")
    return value


def _fixed_numeric_tuple(values: object, width: int, error: str) -> tuple[float, ...]:
    if not isinstance(values, (tuple, list)) or len(values) != width:
        raise ValueError(error)
    return tuple(_finite_real(value, error) for value in values)


@dataclass(frozen=True)
class L20ActiveCommand16:
    """Normalized command in the L20's canonical 16 logical active channels."""

    values: tuple[tuple[str, float], ...]
    timestamp_s: float = 0.0
    sequence_id: int = 0

    def __post_init__(self) -> None:
        if not isinstance(self.values, tuple) or len(self.values) != len(L20_ACTIVE_CHANNELS):
            raise ValueError("L20_ACTIVE_COMMAND_CHANNEL_MISMATCH")
        canonical: list[tuple[str, float]] = []
        seen: set[str] = set()
        for expected_name, item in zip(L20_ACTIVE_CHANNELS, self.values, strict=True):
            if not isinstance(item, tuple) or len(item) != 2:
                raise ValueError("L20_ACTIVE_COMMAND_CHANNEL_MISMATCH")
            name, raw_value = item
            if name != expected_name or not isinstance(name, str) or name in seen:
                raise ValueError("L20_ACTIVE_COMMAND_CHANNEL_MISMATCH")
            seen.add(name)
            value = _finite_real(raw_value, "L20_ACTIVE_COMMAND_VALUE_INVALID")
            if not 0.0 <= value <= 1.0:
                raise ValueError("L20_ACTIVE_COMMAND_VALUE_INVALID")
            canonical.append((name, value))
        object.__setattr__(self, "values", tuple(canonical))
        object.__setattr__(self, "timestamp_s", _finite_real(self.timestamp_s, "L20_ACTIVE_COMMAND_TIMESTAMP_INVALID", minimum=0.0))
        object.__setattr__(self, "sequence_id", _sequence_id(self.sequence_id))

    @classmethod
    def from_mapping(
        cls,
        values: Mapping[str, object],
        *,
        timestamp_s: float = 0.0,
        sequence_id: int = 0,
    ) -> "L20ActiveCommand16":
        if not isinstance(values, Mapping) or set(values) != set(L20_ACTIVE_CHANNELS) or len(values) != len(L20_ACTIVE_CHANNELS):
            raise ValueError("L20_ACTIVE_COMMAND_CHANNEL_MISMATCH")
        return cls(tuple((name, values[name]) for name in L20_ACTIVE_CHANNELS), timestamp_s, sequence_id)  # type: ignore[arg-type]

    def as_mapping(self) -> Mapping[str, float]:
        return MappingProxyType(dict(self.values))


@dataclass(frozen=True)
class L20ProtocolCommand20:
    """Normalized 20-slot protocol space after direction mapping and reserved insertion."""

    values: tuple[float, ...]

    def __post_init__(self) -> None:
        canonical = _fixed_numeric_tuple(self.values, 20, "L20_PROTOCOL_COMMAND20_INVALID")
        if any(not 0.0 <= value <= 1.0 for value in canonical):
            raise ValueError("L20_PROTOCOL_COMMAND20_INVALID")
        object.__setattr__(self, "values", canonical)


@dataclass(frozen=True)
class L20PhysicalTarget21:
    """Radian targets in canonical L20 physical-joint order."""

    positions_rad: tuple[float, ...]
    coupling_profile: str
    source_timestamp_s: float = 0.0
    sequence_id: int = 0

    def __post_init__(self) -> None:
        object.__setattr__(self, "positions_rad", _fixed_numeric_tuple(self.positions_rad, 21, "L20_PHYSICAL_TARGET21_INVALID"))
        if not isinstance(self.coupling_profile, str) or not self.coupling_profile:
            raise ValueError("L20_PHYSICAL_TARGET21_INVALID")
        object.__setattr__(self, "source_timestamp_s", _finite_real(self.source_timestamp_s, "L20_PHYSICAL_TARGET21_INVALID", minimum=0.0))
        object.__setattr__(self, "sequence_id", _sequence_id(self.sequence_id))

    def as_mapping(self) -> Mapping[str, float]:
        return MappingProxyType(dict(zip(L20_PHYSICAL_JOINTS, self.positions_rad, strict=True)))


@dataclass(frozen=True)
class L20PhysicalState21:
    """Backend-neutral physical state in canonical L20 physical-joint order."""

    positions_rad: tuple[float, ...]
    velocities_rad_s: tuple[float, ...]
    efforts_nm: tuple[float | None, ...] | None = None
    timestamp_s: float = 0.0

    def __post_init__(self) -> None:
        object.__setattr__(self, "positions_rad", _fixed_numeric_tuple(self.positions_rad, 21, "L20_PHYSICAL_STATE21_INVALID"))
        object.__setattr__(self, "velocities_rad_s", _fixed_numeric_tuple(self.velocities_rad_s, 21, "L20_PHYSICAL_STATE21_INVALID"))
        if self.efforts_nm is not None:
            if not isinstance(self.efforts_nm, (tuple, list)) or len(self.efforts_nm) != 21:
                raise ValueError("L20_PHYSICAL_STATE21_INVALID")
            efforts: list[float | None] = []
            for value in self.efforts_nm:
                efforts.append(None if value is None else _finite_real(value, "L20_PHYSICAL_STATE21_INVALID"))
            object.__setattr__(self, "efforts_nm", tuple(efforts))
        object.__setattr__(self, "timestamp_s", _finite_real(self.timestamp_s, "L20_PHYSICAL_STATE21_INVALID", minimum=0.0))
