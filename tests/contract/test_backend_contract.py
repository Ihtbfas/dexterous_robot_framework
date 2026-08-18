from __future__ import annotations

from dataclasses import FrozenInstanceError

import pytest

from dexterous_robot.backends.base import Backend, BackendState
from dexterous_robot.core import JointPositionCommand, JointState, Pose


def _joint_state() -> JointState:
    return JointState(
        names=("joint_a",),
        position_rad=(0.1,),
        velocity_rad_s=(0.2,),
        effort_nm=(0.3,),
    )


def _pose() -> Pose:
    return Pose(
        position_xyz_m=(1.0, 2.0, 3.0),
        quaternion_xyzw=(0.0, 0.0, 0.0, 1.0),
        frame_id="world",
    )


def test_backend_is_abstract_and_requires_complete_contract() -> None:
    class IncompleteBackend(Backend):
        pass

    with pytest.raises(TypeError):
        IncompleteBackend()


def test_backend_state_defensively_freezes_all_mappings() -> None:
    device_states = {"arm": _joint_state()}
    body_poses = {"tool": _pose()}
    signals = {"contact": True, "mode": "ready", "count": 3, "score": 0.5, "empty": None}

    state = BackendState(
        device_states=device_states,
        body_poses=body_poses,
        signals=signals,
    )

    device_states["hand"] = _joint_state()
    body_poses["object"] = _pose()
    signals["contact"] = False

    assert tuple(state.device_states) == ("arm",)
    assert tuple(state.body_poses) == ("tool",)
    assert state.signals["contact"] is True

    with pytest.raises(TypeError):
        state.device_states["hand"] = _joint_state()  # type: ignore[index]
    with pytest.raises(TypeError):
        state.body_poses["object"] = _pose()  # type: ignore[index]
    with pytest.raises(TypeError):
        state.signals["contact"] = False  # type: ignore[index]
    with pytest.raises(FrozenInstanceError):
        state.signals = {}  # type: ignore[misc]


def test_backend_state_rejects_invalid_mapping_keys_and_signal_values() -> None:
    with pytest.raises(ValueError, match="BACKEND_STATE_DEVICE_ID_INVALID"):
        BackendState(device_states={"": _joint_state()}, body_poses={}, signals={})
    with pytest.raises(ValueError, match="BACKEND_STATE_BODY_ID_INVALID"):
        BackendState(device_states={}, body_poses={3: _pose()}, signals={})  # type: ignore[dict-item]
    with pytest.raises(ValueError, match="BACKEND_STATE_SIGNAL_NAME_INVALID"):
        BackendState(device_states={}, body_poses={}, signals={"": True})
    with pytest.raises(ValueError, match="BACKEND_STATE_SIGNAL_VALUE_INVALID"):
        BackendState(device_states={}, body_poses={}, signals={"bad": object()})  # type: ignore[dict-item]


def test_backend_apply_signature_accepts_typed_joint_commands() -> None:
    events: list[tuple[str, object]] = []

    class RecordingBackend(Backend):
        def initialize(self) -> None:
            events.append(("initialize", None))

        def reset(self) -> None:
            events.append(("reset", None))

        def read_state(self) -> BackendState:
            events.append(("read_state", None))
            return BackendState(device_states={}, body_poses={}, signals={})

        def apply(self, commands) -> None:
            events.append(("apply", tuple(commands)))

        def step(self, dt_s: float) -> None:
            events.append(("step", dt_s))

        def shutdown(self) -> None:
            events.append(("shutdown", None))

    cmd = JointPositionCommand(device_id="arm", joint_names=("joint_a",), position_rad=(0.4,))
    backend = RecordingBackend()
    backend.initialize()
    backend.apply((cmd,))
    backend.step(0.01)
    backend.read_state()
    backend.shutdown()

    assert [name for name, _ in events] == ["initialize", "apply", "step", "read_state", "shutdown"]
    assert events[1][1] == (cmd,)
