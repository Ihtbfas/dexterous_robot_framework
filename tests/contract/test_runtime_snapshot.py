from __future__ import annotations

from dataclasses import FrozenInstanceError

import pytest

from dexterous_robot.core import JointState, Pose
from dexterous_robot.runtime.snapshot import RuntimeSnapshot


def _joint_state() -> JointState:
    return JointState(
        names=("joint_a",),
        position_rad=(0.0,),
        velocity_rad_s=(0.0,),
    )


def _pose() -> Pose:
    return Pose(
        position_xyz_m=(0.0, 0.0, 0.0),
        quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        frame_id="world",
    )


def test_runtime_snapshot_is_deeply_stable_at_mapping_boundary() -> None:
    device_states = {"arm": _joint_state()}
    body_poses = {"tool": _pose()}
    signals = {"ready": True}

    snapshot = RuntimeSnapshot(
        time_s=0.25,
        dt_s=0.01,
        device_states=device_states,
        body_poses=body_poses,
        signals=signals,
    )

    device_states.clear()
    body_poses.clear()
    signals["ready"] = False

    assert tuple(snapshot.device_states) == ("arm",)
    assert tuple(snapshot.body_poses) == ("tool",)
    assert snapshot.signals["ready"] is True

    with pytest.raises(TypeError):
        snapshot.signals["ready"] = False  # type: ignore[index]
    with pytest.raises(FrozenInstanceError):
        snapshot.time_s = 9.0  # type: ignore[misc]


def test_runtime_snapshot_rejects_nonfinite_or_invalid_time() -> None:
    kwargs = dict(device_states={}, body_poses={}, signals={})
    with pytest.raises(ValueError, match="RUNTIME_SNAPSHOT_TIME_INVALID"):
        RuntimeSnapshot(time_s=float("nan"), dt_s=0.01, **kwargs)
    with pytest.raises(ValueError, match="RUNTIME_SNAPSHOT_TIME_INVALID"):
        RuntimeSnapshot(time_s=-0.01, dt_s=0.01, **kwargs)
    with pytest.raises(ValueError, match="RUNTIME_SNAPSHOT_DT_INVALID"):
        RuntimeSnapshot(time_s=0.0, dt_s=0.0, **kwargs)
    with pytest.raises(ValueError, match="RUNTIME_SNAPSHOT_DT_INVALID"):
        RuntimeSnapshot(time_s=0.0, dt_s=float("inf"), **kwargs)
