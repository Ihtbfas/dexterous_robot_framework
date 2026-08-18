from __future__ import annotations

from dataclasses import dataclass

from dexterous_robot.core import JointPositionCommand
from dexterous_robot.devices.hands.linker_l20 import (
    COUPLING_MULTIPLIERS,
    L20_JOINT_SPECS,
    L20_PHYSICAL_JOINTS,
    L20PhysicalTarget21,
)

_THUMB_TRIM_JOINTS = ("thumb_joint2", "thumb_joint3")
_FOUR_SIDE_TRIM_JOINTS = ("ring_joint1", "ring_joint2", "little_joint1", "little_joint2")
_THUMB_EXTRA_CLOSE_RAD = 0.04
_FOUR_SIDE_EXTRA_CLOSE_RAD = 0.08


@dataclass(frozen=True)
class GraspLockGoal:
    base_target: L20PhysicalTarget21

    def __post_init__(self) -> None:
        if not isinstance(self.base_target, L20PhysicalTarget21):
            raise ValueError("GRASP_LOCK_BASE_TARGET_INVALID")


class GraspLockController:
    """Latch one stronger preloaded hand target and keep it fixed until reset."""

    def __init__(self) -> None:
        self._locked_command: JointPositionCommand | None = None

    def reset(self) -> None:
        self._locked_command = None

    def compute(self, goal: GraspLockGoal) -> JointPositionCommand:
        if not isinstance(goal, GraspLockGoal):
            raise ValueError("GRASP_LOCK_GOAL_INVALID")
        if self._locked_command is not None:
            return self._locked_command

        base = goal.base_target
        multipliers = COUPLING_MULTIPLIERS.get(base.coupling_profile)
        if multipliers is None:
            raise ValueError("GRASP_LOCK_COUPLING_PROFILE_INVALID")
        values = dict(base.as_mapping())
        for name in _THUMB_TRIM_JOINTS:
            values[name] = float(values[name] + _THUMB_EXTRA_CLOSE_RAD)
        for name in _FOUR_SIDE_TRIM_JOINTS:
            values[name] = float(values[name] + _FOUR_SIDE_EXTRA_CLOSE_RAD)

        for spec in L20_JOINT_SPECS:
            if spec.coupled_from is not None:
                values[spec.name] = float(values[spec.coupled_from] * multipliers[spec.name])

        self._locked_command = JointPositionCommand(
            device_id="hand",
            joint_names=L20_PHYSICAL_JOINTS,
            position_rad=tuple(values[name] for name in L20_PHYSICAL_JOINTS),
            profile="hand_grasp_lock",
        )
        return self._locked_command
