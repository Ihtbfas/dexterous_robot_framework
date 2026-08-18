from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import pytest

from dexterous_robot.control.arm.cartesian_carry import CartesianCarryController, CartesianCarryGoal
from dexterous_robot.control.arm.kinematics import Wam7Kinematics
from dexterous_robot.core import Pose
from dexterous_robot.devices.arms.wam7 import WAM7_JOINT_NAMES

FIXTURE = Path(__file__).resolve().parents[1] / "fixtures" / "wam7_legacy_ik_golden_vectors.json"


def _quat_angle(a: tuple[float, ...], b: tuple[float, ...]) -> float:
    av = np.asarray(a, dtype=float); av /= np.linalg.norm(av)
    bv = np.asarray(b, dtype=float); bv /= np.linalg.norm(bv)
    return 2.0 * math.acos(float(np.clip(abs(np.dot(av, bv)), -1.0, 1.0)))


def _grasp_vector() -> dict[str, object]:
    rows = json.loads(FIXTURE.read_text(encoding="utf-8"))["vectors"]
    return next(row for row in rows if row["name"] == "grasp")


def test_cartesian_carry_locks_xy_and_orientation_and_minimum_jerks_world_z() -> None:
    kin = Wam7Kinematics()
    row = _grasp_vector()
    start = kin.forward(row["solved_q_rad"])
    goal = CartesianCarryGoal(start, (0.0, 0.0, 0.05), 10.0)
    controller = CartesianCarryController(kinematics=kin)

    command = controller.compute(elapsed_s=5.0, current_q_rad=row["solved_q_rad"], goal=goal)
    achieved = kin.forward(command.position_rad)

    assert command.device_id == "arm"
    assert command.joint_names == WAM7_JOINT_NAMES
    assert command.profile == "arm_carry_position_drive"
    assert achieved.position_xyz_m[0] == pytest.approx(start.position_xyz_m[0], abs=1.0e-4)
    assert achieved.position_xyz_m[1] == pytest.approx(start.position_xyz_m[1], abs=1.0e-4)
    assert achieved.position_xyz_m[2] == pytest.approx(start.position_xyz_m[2] + 0.025, abs=1.0e-4)
    assert _quat_angle(achieved.quaternion_xyzw, start.quaternion_xyzw) <= 1.0e-3


def test_cartesian_carry_clamps_before_start_and_after_duration() -> None:
    kin = Wam7Kinematics()
    row = _grasp_vector()
    start = kin.forward(row["solved_q_rad"])
    controller = CartesianCarryController(kinematics=kin)
    goal = CartesianCarryGoal(start, (0.0, 0.0, 0.05), 10.0)

    before = controller.compute(elapsed_s=-1.0, current_q_rad=row["solved_q_rad"], goal=goal)
    at_end = controller.compute(elapsed_s=50.0, current_q_rad=before.position_rad, goal=goal)
    before_pose = kin.forward(before.position_rad)
    end_pose = kin.forward(at_end.position_rad)
    assert before_pose.position_xyz_m == pytest.approx(start.position_xyz_m, abs=1.0e-4)
    assert end_pose.position_xyz_m[2] == pytest.approx(start.position_xyz_m[2] + 0.05, abs=1.0e-4)
    assert _quat_angle(end_pose.quaternion_xyzw, start.quaternion_xyzw) <= 1.0e-3


def test_cartesian_carry_rejects_xy_motion_and_invalid_duration() -> None:
    pose = Pose((0.4, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0), "world")
    with pytest.raises(ValueError, match="CARTESIAN_CARRY_WORLD_Z_ONLY"):
        CartesianCarryGoal(pose, (0.01, 0.0, 0.05), 10.0)
    with pytest.raises(ValueError, match="CARTESIAN_CARRY_DURATION_INVALID"):
        CartesianCarryGoal(pose, (0.0, 0.0, 0.05), 0.0)
