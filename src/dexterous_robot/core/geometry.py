from __future__ import annotations

from dataclasses import dataclass

from ._validation import tuple_of_floats


@dataclass(frozen=True)
class Pose:
    position_xyz_m: tuple[float, float, float]
    quaternion_xyzw: tuple[float, float, float, float]
    frame_id: str

    def __post_init__(self) -> None:
        position = tuple_of_floats(self.position_xyz_m, error_prefix="POSE_POSITION")
        quaternion = tuple_of_floats(self.quaternion_xyzw, error_prefix="POSE_QUATERNION")
        if len(position) != 3:
            raise ValueError("POSE_POSITION_INVALID")
        if len(quaternion) != 4:
            raise ValueError("POSE_QUATERNION_INVALID")
        if not isinstance(self.frame_id, str) or not self.frame_id:
            raise ValueError("POSE_FRAME_ID_INVALID")
        object.__setattr__(self, "position_xyz_m", position)
        object.__setattr__(self, "quaternion_xyzw", quaternion)
