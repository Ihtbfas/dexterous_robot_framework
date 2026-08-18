from __future__ import annotations

import pytest

from dexterous_robot.devices.arms.wam7 import WAM7_JOINT_NAMES, Wam7Model
from dexterous_robot.devices.hands.linker_l20 import L20_PHYSICAL_JOINTS, LinkerL20Model
from dexterous_robot.robots import ManipulatorSystem, MountTransform
from dexterous_robot.core import Pose


def _robot() -> ManipulatorSystem:
    arm = Wam7Model()
    hand = LinkerL20Model(coupling_profile="mujoco_equal_v1")
    mount = MountTransform(
        parent_frame=arm.flange_frame,
        child_frame="l20_base",
        pose=Pose((0.0, 0.0, 0.00991000205278397), (0.0, 0.0, 0.0, 1.0), arm.flange_frame),
    )
    return ManipulatorSystem("wam7_linker_l20", arm, hand, mount, "l20_tcp")


def test_combined_28_topology_consumes_every_backend_lane_once():
    from dexterous_robot.backends.isaac.topology import ISAAC_L20_BACKEND_JOINT_ORDER, build_joint_routing

    backend_names = WAM7_JOINT_NAMES + ISAAC_L20_BACKEND_JOINT_ORDER
    routing = build_joint_routing(backend_names, _robot())

    assert routing.backend_joint_names == backend_names
    assert routing.arm_backend_indices == tuple(range(7))
    assert len(routing.hand_backend_indices) == 21
    assert set(routing.arm_backend_indices + routing.hand_backend_indices) == set(range(28))
    assert len(set(routing.arm_backend_indices + routing.hand_backend_indices)) == 28


def test_hand_routing_reorders_backend_interleaving_into_canonical_physical21():
    from dexterous_robot.backends.isaac.topology import ISAAC_L20_BACKEND_JOINT_ORDER, build_joint_routing

    routing = build_joint_routing(WAM7_JOINT_NAMES + ISAAC_L20_BACKEND_JOINT_ORDER, _robot())
    actual_index = {name: i for i, name in enumerate(WAM7_JOINT_NAMES + ISAAC_L20_BACKEND_JOINT_ORDER)}

    assert routing.hand_joint_names == L20_PHYSICAL_JOINTS
    assert routing.hand_backend_indices == tuple(actual_index[name] for name in L20_PHYSICAL_JOINTS)
    assert routing.hand_backend_indices[:5] == (7, 12, 17, 22, 27)


def test_topology_rejects_duplicate_missing_or_foreign_backend_names():
    from dexterous_robot.backends.isaac.topology import ISAAC_L20_BACKEND_JOINT_ORDER, build_joint_routing

    valid = list(WAM7_JOINT_NAMES + ISAAC_L20_BACKEND_JOINT_ORDER)
    for mutated in (
        valid[:-1],
        valid + ["foreign_joint"],
        valid[:-1] + [valid[-2]],
    ):
        with pytest.raises(ValueError, match="ISAAC_COMBINED_TOPOLOGY_INVALID"):
            build_joint_routing(tuple(mutated), _robot())


def test_scatter_and_gather_use_logical_device_orders_not_backend_order():
    from dexterous_robot.backends.isaac.topology import ISAAC_L20_BACKEND_JOINT_ORDER, build_joint_routing

    routing = build_joint_routing(WAM7_JOINT_NAMES + ISAAC_L20_BACKEND_JOINT_ORDER, _robot())
    arm = tuple(float(i + 1) for i in range(7))
    hand = tuple(float(100 + i) for i in range(21))
    full = routing.scatter(arm_values=arm, hand_values=hand)

    assert routing.gather_arm(full) == arm
    assert routing.gather_hand(full) == hand
    assert full[7] == hand[0]      # thumb_joint0
    assert full[12] == hand[1]     # thumb_joint1
    assert full[27] == hand[4]     # thumb_joint4
