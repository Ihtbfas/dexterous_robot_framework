from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

from dexterous_robot.devices.hands.linker_l20 import L20_PHYSICAL_JOINTS
from dexterous_robot.robots import ManipulatorSystem

# This is the observed physical lane order of the frozen Isaac/PhysX L20 asset.
# It is intentionally backend-specific and distinct from the framework's
# canonical Physical21 order.
ISAAC_L20_BACKEND_JOINT_ORDER: tuple[str, ...] = (
    "thumb_joint0",
    "index_joint0",
    "middle_joint0",
    "ring_joint0",
    "little_joint0",
    "thumb_joint1",
    "index_joint1",
    "middle_joint1",
    "ring_joint1",
    "little_joint1",
    "thumb_joint2",
    "index_joint2",
    "middle_joint2",
    "ring_joint2",
    "little_joint2",
    "thumb_joint3",
    "index_joint3",
    "middle_joint3",
    "ring_joint3",
    "little_joint3",
    "thumb_joint4",
)


def _float_tuple(values: Sequence[float], *, width: int, error: str) -> tuple[float, ...]:
    if len(values) != width:
        raise ValueError(error)
    return tuple(float(value) for value in values)


@dataclass(frozen=True)
class JointRouting:
    backend_joint_names: tuple[str, ...]
    arm_joint_names: tuple[str, ...]
    hand_joint_names: tuple[str, ...]
    arm_backend_indices: tuple[int, ...]
    hand_backend_indices: tuple[int, ...]

    def __post_init__(self) -> None:
        width = len(self.backend_joint_names)
        consumed = self.arm_backend_indices + self.hand_backend_indices
        if (
            width != 28
            or len(self.arm_joint_names) != 7
            or len(self.hand_joint_names) != 21
            or len(consumed) != width
            or len(set(consumed)) != width
            or set(consumed) != set(range(width))
        ):
            raise ValueError("ISAAC_COMBINED_TOPOLOGY_INVALID")

    def gather_arm(self, backend_values: Sequence[float]) -> tuple[float, ...]:
        values = _float_tuple(backend_values, width=len(self.backend_joint_names), error="ISAAC_BACKEND_VECTOR_WIDTH_INVALID")
        return tuple(values[index] for index in self.arm_backend_indices)

    def gather_hand(self, backend_values: Sequence[float]) -> tuple[float, ...]:
        values = _float_tuple(backend_values, width=len(self.backend_joint_names), error="ISAAC_BACKEND_VECTOR_WIDTH_INVALID")
        return tuple(values[index] for index in self.hand_backend_indices)

    def scatter(self, *, arm_values: Sequence[float], hand_values: Sequence[float]) -> tuple[float, ...]:
        arm = _float_tuple(arm_values, width=len(self.arm_joint_names), error="ISAAC_ARM_VECTOR_WIDTH_INVALID")
        hand = _float_tuple(hand_values, width=len(self.hand_joint_names), error="ISAAC_HAND_VECTOR_WIDTH_INVALID")
        out = [0.0] * len(self.backend_joint_names)
        for index, value in zip(self.arm_backend_indices, arm, strict=True):
            out[index] = value
        for index, value in zip(self.hand_backend_indices, hand, strict=True):
            out[index] = value
        return tuple(out)


def build_joint_routing(backend_joint_names: Sequence[str], robot: ManipulatorSystem) -> JointRouting:
    actual = tuple(str(name) for name in backend_joint_names)
    expected_names = tuple(robot.arm.joint_names) + tuple(robot.hand.physical_joints)
    if (
        len(actual) != len(expected_names)
        or len(set(actual)) != len(actual)
        or set(actual) != set(expected_names)
        or tuple(robot.hand.physical_joints) != L20_PHYSICAL_JOINTS
    ):
        raise ValueError("ISAAC_COMBINED_TOPOLOGY_INVALID")

    index_by_name = {name: index for index, name in enumerate(actual)}
    arm_indices = tuple(index_by_name[name] for name in robot.arm.joint_names)
    hand_indices = tuple(index_by_name[name] for name in robot.hand.physical_joints)
    return JointRouting(
        backend_joint_names=actual,
        arm_joint_names=tuple(robot.arm.joint_names),
        hand_joint_names=tuple(robot.hand.physical_joints),
        arm_backend_indices=arm_indices,
        hand_backend_indices=hand_indices,
    )
