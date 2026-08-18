from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from types import MappingProxyType
from typing import Mapping, Sequence

from dexterous_robot.core import Command, JointState, Pose

SignalValue = float | int | bool | str | None


def _freeze_named_mapping(mapping: Mapping[str, object], *, key_error: str, value_type: type, value_error: str):
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
            raise ValueError("BACKEND_STATE_SIGNAL_NAME_INVALID")
        if value is not None and not isinstance(value, (bool, int, float, str)):
            raise ValueError("BACKEND_STATE_SIGNAL_VALUE_INVALID")
        copied[key] = value
    return MappingProxyType(copied)


@dataclass(frozen=True)
class BackendState:
    device_states: Mapping[str, JointState]
    body_poses: Mapping[str, Pose]
    signals: Mapping[str, SignalValue]

    def __post_init__(self) -> None:
        device_states = _freeze_named_mapping(
            self.device_states,
            key_error="BACKEND_STATE_DEVICE_ID_INVALID",
            value_type=JointState,
            value_error="BACKEND_STATE_DEVICE_STATE_INVALID",
        )
        body_poses = _freeze_named_mapping(
            self.body_poses,
            key_error="BACKEND_STATE_BODY_ID_INVALID",
            value_type=Pose,
            value_error="BACKEND_STATE_BODY_POSE_INVALID",
        )
        signals = _freeze_signals(self.signals)
        object.__setattr__(self, "device_states", device_states)
        object.__setattr__(self, "body_poses", body_poses)
        object.__setattr__(self, "signals", signals)


class Backend(ABC):
    @abstractmethod
    def initialize(self) -> None:
        """Initialize backend resources without advancing simulation or wall time."""

    @abstractmethod
    def reset(self) -> None:
        """Reset backend state to its configured initial condition."""

    @abstractmethod
    def read_state(self) -> BackendState:
        """Read one coherent backend state sample."""

    @abstractmethod
    def apply(self, commands: Sequence[Command]) -> None:
        """Dispatch typed commands without advancing backend time."""

    @abstractmethod
    def step(self, dt_s: float) -> None:
        """Advance the backend by exactly one runtime-owned control period."""

    @abstractmethod
    def shutdown(self) -> None:
        """Release backend resources."""
