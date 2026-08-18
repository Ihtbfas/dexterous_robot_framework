from __future__ import annotations

from dataclasses import dataclass
from typing import TypeAlias

from ._validation import tuple_of_floats, tuple_of_names


def _validate_device_id(device_id: str) -> None:
    if not isinstance(device_id, str) or not device_id:
        raise ValueError("COMMAND_DEVICE_ID_INVALID")


@dataclass(frozen=True)
class JointPositionCommand:
    device_id: str
    joint_names: tuple[str, ...]
    position_rad: tuple[float, ...]
    profile: str | None = None

    def __post_init__(self) -> None:
        _validate_device_id(self.device_id)
        names = tuple_of_names(self.joint_names)
        position = tuple_of_floats(self.position_rad, error_prefix="JOINT_POSITION_COMMAND")
        if len(names) != len(position):
            raise ValueError("JOINT_POSITION_COMMAND_WIDTH_MISMATCH")
        if self.profile is not None and (not isinstance(self.profile, str) or not self.profile):
            raise ValueError("JOINT_POSITION_COMMAND_PROFILE_INVALID")
        object.__setattr__(self, "joint_names", names)
        object.__setattr__(self, "position_rad", position)


@dataclass(frozen=True)
class JointEffortCommand:
    device_id: str
    joint_names: tuple[str, ...]
    effort_nm: tuple[float, ...]

    def __post_init__(self) -> None:
        _validate_device_id(self.device_id)
        names = tuple_of_names(self.joint_names)
        effort = tuple_of_floats(self.effort_nm, error_prefix="JOINT_EFFORT_COMMAND")
        if len(names) != len(effort):
            raise ValueError("JOINT_EFFORT_COMMAND_WIDTH_MISMATCH")
        object.__setattr__(self, "joint_names", names)
        object.__setattr__(self, "effort_nm", effort)


@dataclass(frozen=True)
class RigidBodyKinematicCommand:
    """Backend-neutral request to switch one semantic rigid body dynamic/kinematic mode."""

    body_id: str
    kinematic_enabled: bool

    def __post_init__(self) -> None:
        if not isinstance(self.body_id, str) or not self.body_id:
            raise ValueError("RIGID_BODY_KINEMATIC_BODY_ID_INVALID")
        if not isinstance(self.kinematic_enabled, bool):
            raise ValueError("RIGID_BODY_KINEMATIC_VALUE_INVALID")


Command: TypeAlias = JointPositionCommand | JointEffortCommand | RigidBodyKinematicCommand
