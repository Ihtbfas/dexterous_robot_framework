from __future__ import annotations

from dataclasses import dataclass

from .mapping import SUPPORTED_COUPLING_PROFILES
from .types import L20_ACTIVE_CHANNELS, L20_PHYSICAL_JOINTS


@dataclass(frozen=True)
class LinkerL20Model:
    """Backend-independent semantic model for one Linker Hand L20 device."""

    device_id: str = "hand"
    active_channels: tuple[str, ...] = L20_ACTIVE_CHANNELS
    physical_joints: tuple[str, ...] = L20_PHYSICAL_JOINTS
    coupling_profile: str = "mujoco_equal_v1"

    def __post_init__(self) -> None:
        if not isinstance(self.device_id, str) or not self.device_id:
            raise ValueError("L20_DEVICE_ID_INVALID")
        if self.active_channels != L20_ACTIVE_CHANNELS:
            raise ValueError("L20_ACTIVE_CHANNELS_INVALID")
        if self.physical_joints != L20_PHYSICAL_JOINTS:
            raise ValueError("L20_PHYSICAL_JOINTS_INVALID")
        if self.coupling_profile not in SUPPORTED_COUPLING_PROFILES:
            raise ValueError("L20_COUPLING_PROFILE_INVALID")
