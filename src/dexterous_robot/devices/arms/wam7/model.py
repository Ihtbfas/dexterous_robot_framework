from __future__ import annotations

from dataclasses import dataclass

WAM7_JOINT_NAMES: tuple[str, ...] = (
    "wam_j1_joint",
    "wam_j2_joint",
    "wam_j3_joint",
    "wam_j4_joint",
    "wam_j5_joint",
    "wam_j6_joint",
    "wam_j7_joint",
)


@dataclass(frozen=True)
class Wam7Model:
    """Backend-independent logical model of the Barrett WAM 7-DOF arm."""

    device_id: str = "arm"
    joint_names: tuple[str, ...] = WAM7_JOINT_NAMES
    base_frame: str = "wam_base"
    flange_frame: str = "wam_j7"

    def __post_init__(self) -> None:
        if not isinstance(self.device_id, str) or not self.device_id:
            raise ValueError("WAM7_DEVICE_ID_INVALID")
        if self.joint_names != WAM7_JOINT_NAMES:
            raise ValueError("WAM7_JOINT_NAMES_INVALID")
        if not isinstance(self.base_frame, str) or not self.base_frame:
            raise ValueError("WAM7_BASE_FRAME_INVALID")
        if not isinstance(self.flange_frame, str) or not self.flange_frame:
            raise ValueError("WAM7_FLANGE_FRAME_INVALID")
