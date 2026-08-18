from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import pytest

from dexterous_robot.control.arm.kinematics import Wam7Kinematics
from dexterous_robot.core import Pose

FIXTURE = Path(__file__).resolve().parents[1] / "fixtures" / "wam7_legacy_ik_golden_vectors.json"


def _quat_angle(a: tuple[float, ...], b: tuple[float, ...]) -> float:
    av = np.asarray(a, dtype=float)
    bv = np.asarray(b, dtype=float)
    av /= np.linalg.norm(av)
    bv /= np.linalg.norm(bv)
    return 2.0 * math.acos(float(np.clip(abs(np.dot(av, bv)), -1.0, 1.0)))


def _fixture() -> dict[str, object]:
    return json.loads(FIXTURE.read_text(encoding="utf-8"))


def test_wam7_forward_matches_frozen_legacy_ik_vectors() -> None:
    kin = Wam7Kinematics()
    data = _fixture()
    assert data["source_sha256"]["scripts/phase2/p2b2_wam_dynamic_l20_tabletop_demo_v3_numerical_ik_grasp_lift.py"] == "1d6afd1bc20e930b536946b611fd5a6de86c31abdc2260a4717f2cd3a11e7c15"
    for row in data["vectors"]:
        target = row["target_pose"]
        pose = kin.forward(row["solved_q_rad"])
        assert np.linalg.norm(np.asarray(pose.position_xyz_m) - np.asarray(target["position_xyz_m"])) <= 2.0e-5
        assert _quat_angle(pose.quaternion_xyzw, tuple(target["quaternion_xyzw"])) <= 5.0e-5
        assert pose.frame_id == "world"


def test_wam7_solve_pose_reproduces_frozen_legacy_chain() -> None:
    kin = Wam7Kinematics()
    for row in _fixture()["vectors"]:
        target = Pose(
            tuple(row["target_pose"]["position_xyz_m"]),
            tuple(row["target_pose"]["quaternion_xyzw"]),
            row["target_pose"]["frame_id"],
        )
        q = kin.solve_pose(target, row["seed_q_rad"])
        assert np.max(np.abs(np.asarray(q) - np.asarray(row["solved_q_rad"]))) <= 5.0e-6
        achieved = kin.forward(q)
        assert np.linalg.norm(np.asarray(achieved.position_xyz_m) - np.asarray(target.position_xyz_m)) <= 1.0e-4
        assert _quat_angle(achieved.quaternion_xyzw, target.quaternion_xyzw) <= 1.0e-3


def test_wam7_kinematics_rejects_invalid_width_frame_and_unreachable_pose() -> None:
    kin = Wam7Kinematics()
    with pytest.raises(ValueError, match="WAM7_Q_INVALID"):
        kin.forward((0.0,) * 6)
    with pytest.raises(ValueError, match="WAM7_TARGET_FRAME_INVALID"):
        kin.solve_pose(Pose((0.4, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0), "camera"), (0.0,) * 7)
    with pytest.raises(RuntimeError, match="WAM7_IK_DID_NOT_CONVERGE"):
        kin.solve_pose(Pose((10.0, 10.0, 10.0), (0.0, 0.0, 0.0, 1.0), "world"), (0.0,) * 7)
