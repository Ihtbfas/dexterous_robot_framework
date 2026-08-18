from __future__ import annotations

from dataclasses import dataclass

from ._validation import tuple_of_floats, tuple_of_names


@dataclass(frozen=True)
class JointState:
    names: tuple[str, ...]
    position_rad: tuple[float, ...]
    velocity_rad_s: tuple[float, ...]
    effort_nm: tuple[float, ...] | None = None

    def __post_init__(self) -> None:
        names = tuple_of_names(self.names)
        position = tuple_of_floats(self.position_rad, error_prefix="JOINT_STATE_POSITION")
        velocity = tuple_of_floats(self.velocity_rad_s, error_prefix="JOINT_STATE_VELOCITY")
        effort = None
        if self.effort_nm is not None:
            effort = tuple_of_floats(self.effort_nm, error_prefix="JOINT_STATE_EFFORT")
        width = len(names)
        if len(position) != width or len(velocity) != width or (effort is not None and len(effort) != width):
            raise ValueError("JOINT_STATE_WIDTH_MISMATCH")
        object.__setattr__(self, "names", names)
        object.__setattr__(self, "position_rad", position)
        object.__setattr__(self, "velocity_rad_s", velocity)
        object.__setattr__(self, "effort_nm", effort)
