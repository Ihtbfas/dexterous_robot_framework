from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from types import MappingProxyType
from typing import Mapping

from dexterous_robot.backends.base import SignalValue
from dexterous_robot.core import JointState, Pose


def _positive_finite(value: float, *, error: str, allow_zero: bool) -> float:
    if isinstance(value, bool):
        raise ValueError(error)
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(error) from exc
    if not isfinite(result) or result < 0.0 or (not allow_zero and result == 0.0):
        raise ValueError(error)
    return result


def _freeze_mapping(mapping: Mapping[str, object], *, key_error: str, value_type: type, value_error: str):
    copied: dict[str, object] = {}
    for key, value in mapping.items():
        if not isinstance(key, str) or not key:
            raise ValueError(key_error)
        if not isinstance(value, value_type):
            raise ValueError(value_error)
        copied[key] = value
    return MappingProxyType(copied)


def _freeze_signals(signals: Mapping[str, SignalValue]) -> Mapping[str, SignalValue]:
    copied: dict[str, SignalValue] = {}
    for key, value in signals.items():
        if not isinstance(key, str) or not key:
            raise ValueError("RUNTIME_SNAPSHOT_SIGNAL_NAME_INVALID")
        if value is not None and not isinstance(value, (bool, int, float, str)):
            raise ValueError("RUNTIME_SNAPSHOT_SIGNAL_VALUE_INVALID")
        copied[key] = value
    return MappingProxyType(copied)


@dataclass(frozen=True)
class RuntimeSnapshot:
    time_s: float
    dt_s: float
    device_states: Mapping[str, JointState]
    body_poses: Mapping[str, Pose]
    signals: Mapping[str, SignalValue]

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "time_s",
            _positive_finite(self.time_s, error="RUNTIME_SNAPSHOT_TIME_INVALID", allow_zero=True),
        )
        object.__setattr__(
            self,
            "dt_s",
            _positive_finite(self.dt_s, error="RUNTIME_SNAPSHOT_DT_INVALID", allow_zero=False),
        )
        object.__setattr__(
            self,
            "device_states",
            _freeze_mapping(
                self.device_states,
                key_error="RUNTIME_SNAPSHOT_DEVICE_ID_INVALID",
                value_type=JointState,
                value_error="RUNTIME_SNAPSHOT_DEVICE_STATE_INVALID",
            ),
        )
        object.__setattr__(
            self,
            "body_poses",
            _freeze_mapping(
                self.body_poses,
                key_error="RUNTIME_SNAPSHOT_BODY_ID_INVALID",
                value_type=Pose,
                value_error="RUNTIME_SNAPSHOT_BODY_POSE_INVALID",
            ),
        )
        object.__setattr__(self, "signals", _freeze_signals(self.signals))
