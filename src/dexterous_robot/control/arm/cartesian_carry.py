from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass

from dexterous_robot.control.math.minimum_jerk import minimum_jerk_position
from dexterous_robot.core import JointPositionCommand, Pose
from dexterous_robot.devices.arms.wam7 import WAM7_JOINT_NAMES

from .kinematics import Wam7Kinematics


@dataclass(frozen=True)
class CartesianCarryGoal:
    locked_tcp_pose: Pose
    delta_world_m: tuple[float, float, float]
    duration_s: float

    def __post_init__(self) -> None:
        if not isinstance(self.locked_tcp_pose, Pose):
            raise ValueError("CARTESIAN_CARRY_POSE_INVALID")
        try:
            delta = tuple(float(v) for v in self.delta_world_m)
        except (TypeError, ValueError) as exc:
            raise ValueError("CARTESIAN_CARRY_DELTA_INVALID") from exc
        if len(delta) != 3 or not all(math.isfinite(v) for v in delta):
            raise ValueError("CARTESIAN_CARRY_DELTA_INVALID")
        if abs(delta[0]) > 1.0e-12 or abs(delta[1]) > 1.0e-12:
            raise ValueError("CARTESIAN_CARRY_WORLD_Z_ONLY")
        try:
            duration = float(self.duration_s)
        except (TypeError, ValueError) as exc:
            raise ValueError("CARTESIAN_CARRY_DURATION_INVALID") from exc
        if not math.isfinite(duration) or duration <= 0.0:
            raise ValueError("CARTESIAN_CARRY_DURATION_INVALID")
        object.__setattr__(self, "delta_world_m", delta)
        object.__setattr__(self, "duration_s", duration)


class CartesianCarryController:
    """Fixed-orientation, world-Z Cartesian carry solved into WAM joint targets."""

    def __init__(self, *, kinematics: Wam7Kinematics | None = None) -> None:
        self._kinematics = Wam7Kinematics() if kinematics is None else kinematics
        if not isinstance(self._kinematics, Wam7Kinematics):
            raise ValueError("CARTESIAN_CARRY_KINEMATICS_INVALID")

    def compute(
        self,
        *,
        elapsed_s: float,
        current_q_rad: Sequence[float],
        goal: CartesianCarryGoal,
    ) -> JointPositionCommand:
        if not isinstance(goal, CartesianCarryGoal):
            raise ValueError("CARTESIAN_CARRY_GOAL_INVALID")
        try:
            elapsed = float(elapsed_s)
        except (TypeError, ValueError) as exc:
            raise ValueError("CARTESIAN_CARRY_ELAPSED_INVALID") from exc
        if not math.isfinite(elapsed):
            raise ValueError("CARTESIAN_CARRY_ELAPSED_INVALID")

        start = goal.locked_tcp_pose.position_xyz_m
        target_z = start[2] + goal.delta_world_m[2]
        desired = Pose(
            (start[0], start[1], minimum_jerk_position(start[2], target_z, elapsed, goal.duration_s)),
            goal.locked_tcp_pose.quaternion_xyzw,
            goal.locked_tcp_pose.frame_id,
        )
        q_target = self._kinematics.solve_pose(desired, current_q_rad)
        return JointPositionCommand(
            device_id="arm",
            joint_names=WAM7_JOINT_NAMES,
            position_rad=q_target,
            profile="arm_carry_position_drive",
        )
