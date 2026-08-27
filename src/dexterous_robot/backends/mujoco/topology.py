from __future__ import annotations

from dataclasses import dataclass
from types import MappingProxyType
from typing import Any, Mapping

from dexterous_robot.devices.arms.wam7.model import WAM7_JOINT_NAMES
from dexterous_robot.devices.hands.linker_l20.types import L20_PHYSICAL_JOINTS


class MuJoCoTopologyError(ValueError):
    """Raised when compiled MuJoCo topology cannot satisfy canonical routing."""


@dataclass(frozen=True)
class MuJoCoJointAddress:
    joint_name: str
    joint_id: int
    qpos_adr: int
    qvel_adr: int

    def __post_init__(self) -> None:
        if (
            not isinstance(self.joint_name, str)
            or not self.joint_name
            or isinstance(self.joint_id, bool)
            or not isinstance(self.joint_id, int)
            or self.joint_id < 0
            or isinstance(self.qpos_adr, bool)
            or not isinstance(self.qpos_adr, int)
            or self.qpos_adr < 0
            or isinstance(self.qvel_adr, bool)
            or not isinstance(self.qvel_adr, int)
            or self.qvel_adr < 0
        ):
            raise ValueError("MUJOCO_JOINT_ADDRESS_INVALID")


@dataclass(frozen=True)
class MuJoCoActuatorAddress:
    joint_name: str
    actuator_id: int
    actuator_name: str
    ctrl_adr: int
    ctrl_num: int

    def __post_init__(self) -> None:
        if (
            not isinstance(self.joint_name, str)
            or not self.joint_name
            or isinstance(self.actuator_id, bool)
            or not isinstance(self.actuator_id, int)
            or self.actuator_id < 0
            or not isinstance(self.actuator_name, str)
            or not self.actuator_name
            or isinstance(self.ctrl_adr, bool)
            or not isinstance(self.ctrl_adr, int)
            or self.ctrl_adr < 0
            or isinstance(self.ctrl_num, bool)
            or not isinstance(self.ctrl_num, int)
            or self.ctrl_num != 1
        ):
            raise ValueError("MUJOCO_ACTUATOR_ADDRESS_INVALID")


@dataclass(frozen=True)
class MuJoCoRouting:
    arm_joints: tuple[MuJoCoJointAddress, ...]
    hand_joints: tuple[MuJoCoJointAddress, ...]
    arm_actuators: tuple[MuJoCoActuatorAddress, ...]
    hand_actuators: tuple[MuJoCoActuatorAddress, ...]

    def __post_init__(self) -> None:
        if tuple(item.joint_name for item in self.arm_joints) != WAM7_JOINT_NAMES:
            raise ValueError("MUJOCO_ROUTING_ARM_JOINT_ORDER_INVALID")
        if tuple(item.joint_name for item in self.hand_joints) != L20_PHYSICAL_JOINTS:
            raise ValueError("MUJOCO_ROUTING_HAND_JOINT_ORDER_INVALID")
        if tuple(item.joint_name for item in self.hand_actuators) != L20_PHYSICAL_JOINTS:
            raise ValueError("MUJOCO_ROUTING_HAND_ACTUATOR_ORDER_INVALID")

        arm_actuator_names = tuple(item.joint_name for item in self.arm_actuators)
        if len(set(arm_actuator_names)) != len(arm_actuator_names):
            raise ValueError("MUJOCO_ROUTING_ARM_ACTUATOR_DUPLICATE")
        if any(name not in WAM7_JOINT_NAMES for name in arm_actuator_names):
            raise ValueError("MUJOCO_ROUTING_ARM_ACTUATOR_UNKNOWN_JOINT")

    @property
    def joint_by_name(self) -> Mapping[str, MuJoCoJointAddress]:
        return MappingProxyType(
            {
                item.joint_name: item
                for item in (*self.arm_joints, *self.hand_joints)
            }
        )

    @property
    def actuator_by_joint(self) -> Mapping[str, MuJoCoActuatorAddress]:
        return MappingProxyType(
            {
                item.joint_name: item
                for item in (*self.arm_actuators, *self.hand_actuators)
            }
        )


def _canonical_joint_address(
    mujoco: Any,
    model: Any,
    joint_name: str,
) -> MuJoCoJointAddress:
    joint_id = int(
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    )
    if joint_id < 0:
        raise MuJoCoTopologyError(f"MUJOCO_CANONICAL_JOINT_MISSING:{joint_name}")

    joint_type = int(model.jnt_type[joint_id])
    if joint_type != int(mujoco.mjtJoint.mjJNT_HINGE):
        raise MuJoCoTopologyError(
            f"MUJOCO_CANONICAL_JOINT_TYPE_INVALID:{joint_name}:{joint_type}"
        )

    qpos_adr = int(model.jnt_qposadr[joint_id])
    qvel_adr = int(model.jnt_dofadr[joint_id])
    if not (0 <= qpos_adr < int(model.nq)):
        raise MuJoCoTopologyError(
            f"MUJOCO_QPOS_ADDRESS_INVALID:{joint_name}:{qpos_adr}"
        )
    if not (0 <= qvel_adr < int(model.nv)):
        raise MuJoCoTopologyError(
            f"MUJOCO_QVEL_ADDRESS_INVALID:{joint_name}:{qvel_adr}"
        )

    return MuJoCoJointAddress(
        joint_name=joint_name,
        joint_id=joint_id,
        qpos_adr=qpos_adr,
        qvel_adr=qvel_adr,
    )


def _actuator_ctrl_address(model: Any, actuator_id: int) -> tuple[int, int]:
    if hasattr(model, "actuator_ctrladr") and hasattr(model, "actuator_ctrlnum"):
        ctrl_adr = int(model.actuator_ctrladr[actuator_id])
        ctrl_num = int(model.actuator_ctrlnum[actuator_id])
    else:
        # MuJoCo <=3.10 uses one ctrl slot per actuator.
        ctrl_adr = actuator_id
        ctrl_num = 1
    return ctrl_adr, ctrl_num


def _actuator_addresses_by_joint(
    mujoco: Any,
    model: Any,
) -> dict[str, MuJoCoActuatorAddress]:
    addresses: dict[str, MuJoCoActuatorAddress] = {}
    robot_names = set(WAM7_JOINT_NAMES) | set(L20_PHYSICAL_JOINTS)

    for actuator_id in range(int(model.nu)):
        transmission = int(model.actuator_trntype[actuator_id])
        if transmission != int(mujoco.mjtTrn.mjTRN_JOINT):
            raise MuJoCoTopologyError(
                "MUJOCO_ACTUATOR_TRANSMISSION_UNSUPPORTED:"
                f"{actuator_id}:{transmission}"
            )

        joint_id = int(model.actuator_trnid[actuator_id, 0])
        if not (0 <= joint_id < int(model.njnt)):
            raise MuJoCoTopologyError(
                f"MUJOCO_ACTUATOR_JOINT_ID_INVALID:{actuator_id}:{joint_id}"
            )

        joint_name = mujoco.mj_id2name(
            model, mujoco.mjtObj.mjOBJ_JOINT, joint_id
        )
        if not joint_name:
            raise MuJoCoTopologyError(
                f"MUJOCO_ACTUATOR_JOINT_NAME_MISSING:{actuator_id}"
            )
        if joint_name not in robot_names:
            raise MuJoCoTopologyError(
                f"MUJOCO_ACTUATOR_TARGET_OUTSIDE_ROBOT:{actuator_id}:{joint_name}"
            )
        if joint_name in addresses:
            raise MuJoCoTopologyError(
                f"MUJOCO_ACTUATOR_TARGET_DUPLICATE:{joint_name}"
            )

        actuator_name = mujoco.mj_id2name(
            model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_id
        )
        if not actuator_name:
            raise MuJoCoTopologyError(
                f"MUJOCO_ACTUATOR_NAME_MISSING:{actuator_id}"
            )

        ctrl_adr, ctrl_num = _actuator_ctrl_address(model, actuator_id)
        if ctrl_num != 1:
            raise MuJoCoTopologyError(
                f"MUJOCO_ACTUATOR_CTRL_WIDTH_UNSUPPORTED:"
                f"{actuator_name}:{ctrl_num}"
            )
        if not (0 <= ctrl_adr < int(model.nu)):
            raise MuJoCoTopologyError(
                f"MUJOCO_ACTUATOR_CTRL_ADDRESS_INVALID:"
                f"{actuator_name}:{ctrl_adr}"
            )

        addresses[joint_name] = MuJoCoActuatorAddress(
            joint_name=joint_name,
            actuator_id=actuator_id,
            actuator_name=actuator_name,
            ctrl_adr=ctrl_adr,
            ctrl_num=ctrl_num,
        )
    return addresses


def build_mujoco_routing(model: Any) -> MuJoCoRouting:
    """Build canonical name-based qpos/qvel/ctrl routing from a compiled model.

    Declaration order is never used as device semantics. WAM actuators are
    optional at B1.2/B1.3 entry because the qualified WAM runtime is passive;
    Task 9 will add backend-side WAM position actuation. L20 must already expose
    exactly one actuator for each canonical Physical21 joint.
    """
    import mujoco

    arm_joints = tuple(
        _canonical_joint_address(mujoco, model, name)
        for name in WAM7_JOINT_NAMES
    )
    hand_joints = tuple(
        _canonical_joint_address(mujoco, model, name)
        for name in L20_PHYSICAL_JOINTS
    )

    actuator_map = _actuator_addresses_by_joint(mujoco, model)

    missing_hand = [
        name for name in L20_PHYSICAL_JOINTS if name not in actuator_map
    ]
    if missing_hand:
        raise MuJoCoTopologyError(
            "MUJOCO_L20_ACTUATOR_ROUTING_INCOMPLETE:"
            + ",".join(missing_hand)
        )

    arm_actuators = tuple(
        actuator_map[name]
        for name in WAM7_JOINT_NAMES
        if name in actuator_map
    )
    hand_actuators = tuple(
        actuator_map[name] for name in L20_PHYSICAL_JOINTS
    )

    return MuJoCoRouting(
        arm_joints=arm_joints,
        hand_joints=hand_joints,
        arm_actuators=arm_actuators,
        hand_actuators=hand_actuators,
    )
