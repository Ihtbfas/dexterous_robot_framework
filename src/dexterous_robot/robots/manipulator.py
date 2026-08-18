from __future__ import annotations

from dataclasses import dataclass

from dexterous_robot.core import Pose
from dexterous_robot.devices.arms.wam7 import Wam7Model
from dexterous_robot.devices.hands.linker_l20 import LinkerL20Model


@dataclass(frozen=True)
class MountTransform:
    """Logical rigid transform that attaches one device frame to another."""

    parent_frame: str
    child_frame: str
    pose: Pose

    def __post_init__(self) -> None:
        if not isinstance(self.parent_frame, str) or not self.parent_frame:
            raise ValueError("MOUNT_PARENT_FRAME_INVALID")
        if not isinstance(self.child_frame, str) or not self.child_frame:
            raise ValueError("MOUNT_CHILD_FRAME_INVALID")
        if self.parent_frame == self.child_frame:
            raise ValueError("MOUNT_FRAME_COLLISION")
        if self.pose.frame_id != self.parent_frame:
            raise ValueError("MOUNT_POSE_PARENT_FRAME_MISMATCH")


@dataclass(frozen=True)
class ManipulatorSystem:
    """Composition metadata for an arm, hand, mount, and task-facing TCP.

    This type intentionally contains no motion or task action methods. Skills and
    controllers own behavior; backends own physical routing.
    """

    system_id: str
    arm: Wam7Model
    hand: LinkerL20Model
    hand_mount: MountTransform
    tcp_frame: str

    def __post_init__(self) -> None:
        if not isinstance(self.system_id, str) or not self.system_id:
            raise ValueError("MANIPULATOR_SYSTEM_ID_INVALID")
        if not isinstance(self.tcp_frame, str) or not self.tcp_frame:
            raise ValueError("MANIPULATOR_TCP_FRAME_INVALID")
        if self.arm.device_id == self.hand.device_id:
            raise ValueError("MANIPULATOR_DEVICE_ID_COLLISION")
        if self.hand_mount.parent_frame != self.arm.flange_frame:
            raise ValueError("MANIPULATOR_MOUNT_PARENT_MISMATCH")
