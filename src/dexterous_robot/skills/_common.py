from __future__ import annotations

import math
from collections.abc import Mapping

from dexterous_robot.core import JointState, Pose
from dexterous_robot.runtime import RuntimeSnapshot


def positive_finite(value: float, *, error: str, allow_zero: bool = False) -> float:
    if isinstance(value, bool):
        raise ValueError(error)
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(error) from exc
    if not math.isfinite(result) or result < 0.0 or (not allow_zero and result == 0.0):
        raise ValueError(error)
    return result


def snapshot_joint_state(snapshot: RuntimeSnapshot, device_id: str) -> JointState:
    state = snapshot.device_states.get(device_id)
    if not isinstance(state, JointState):
        raise KeyError(f"SKILL_DEVICE_STATE_MISSING:{device_id}")
    return state


def snapshot_pose(snapshot: RuntimeSnapshot, body_id: str) -> Pose:
    pose = snapshot.body_poses.get(body_id)
    if not isinstance(pose, Pose):
        raise KeyError(f"SKILL_BODY_POSE_MISSING:{body_id}")
    return pose


def snapshot_numeric_signal(snapshot: RuntimeSnapshot, signal_name: str) -> float:
    value = snapshot.signals.get(signal_name)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise KeyError(f"SKILL_SIGNAL_MISSING_OR_NONNUMERIC:{signal_name}")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"SKILL_SIGNAL_NONFINITE:{signal_name}")
    return result


def xyz_distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b, strict=True)))


def relative_xyz(object_pose: Pose, hand_pose: Pose) -> tuple[float, float, float]:
    if object_pose.frame_id != hand_pose.frame_id:
        raise ValueError("SKILL_RELATIVE_POSE_FRAME_MISMATCH")
    return tuple(
        object_value - hand_value
        for object_value, hand_value in zip(object_pose.position_xyz_m, hand_pose.position_xyz_m, strict=True)
    )  # type: ignore[return-value]
