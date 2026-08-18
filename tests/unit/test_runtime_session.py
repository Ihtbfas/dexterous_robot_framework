from __future__ import annotations

import math

import pytest

from dexterous_robot.backends.base import Backend, BackendState
from dexterous_robot.core import JointPositionCommand
from dexterous_robot.runtime.session import RuntimeSession


class FakeBackend(Backend):
    def __init__(self) -> None:
        self.events: list[tuple[str, object]] = []
        self.read_count = 0

    def initialize(self) -> None:
        self.events.append(("initialize", None))

    def reset(self) -> None:
        self.events.append(("reset", None))

    def read_state(self) -> BackendState:
        self.read_count += 1
        self.events.append(("read_state", self.read_count))
        return BackendState(
            device_states={},
            body_poses={},
            signals={"read_count": self.read_count},
        )

    def apply(self, commands) -> None:
        self.events.append(("apply", tuple(commands)))

    def step(self, dt_s: float) -> None:
        self.events.append(("step", dt_s))

    def shutdown(self) -> None:
        self.events.append(("shutdown", None))


def _command() -> JointPositionCommand:
    return JointPositionCommand(
        device_id="arm",
        joint_names=("joint_a",),
        position_rad=(0.5,),
    )


def test_runtime_session_rejects_invalid_dt() -> None:
    backend = FakeBackend()
    for invalid in (0.0, -0.1, float("nan"), float("inf")):
        with pytest.raises(ValueError, match="RUNTIME_SESSION_DT_INVALID"):
            RuntimeSession(backend, dt_s=invalid)


def test_initialize_calls_backend_then_samples_time_zero() -> None:
    backend = FakeBackend()
    session = RuntimeSession(backend, dt_s=0.02)

    snapshot = session.initialize()

    assert backend.events == [("initialize", None), ("read_state", 1)]
    assert snapshot.time_s == 0.0
    assert snapshot.dt_s == 0.02
    assert snapshot.signals["read_count"] == 1
    assert session.time_s == 0.0


def test_cycle_orders_apply_step_read_and_advances_runtime_owned_time() -> None:
    backend = FakeBackend()
    session = RuntimeSession(backend, dt_s=0.01)
    session.initialize()
    backend.events.clear()
    command = _command()

    snapshot = session.cycle((command,))

    assert backend.events == [
        ("apply", (command,)),
        ("step", 0.01),
        ("read_state", 2),
    ]
    assert math.isclose(snapshot.time_s, 0.01, rel_tol=0.0, abs_tol=1e-15)
    assert math.isclose(session.time_s, 0.01, rel_tol=0.0, abs_tol=1e-15)


def test_multiple_cycles_advance_by_exact_configured_dt_without_backend_clock() -> None:
    backend = FakeBackend()
    session = RuntimeSession(backend, dt_s=0.125)
    session.initialize()

    first = session.cycle(())
    second = session.cycle(())

    assert first.time_s == 0.125
    assert second.time_s == 0.25
    assert session.time_s == 0.25
    assert [event for event in backend.events if event[0] == "step"] == [
        ("step", 0.125),
        ("step", 0.125),
    ]


def test_reset_calls_backend_resets_runtime_time_and_resamples() -> None:
    backend = FakeBackend()
    session = RuntimeSession(backend, dt_s=0.05)
    session.initialize()
    session.cycle(())
    backend.events.clear()

    snapshot = session.reset()

    assert backend.events == [("reset", None), ("read_state", 3)]
    assert snapshot.time_s == 0.0
    assert session.time_s == 0.0


def test_cycle_and_reset_require_initialize() -> None:
    session = RuntimeSession(FakeBackend(), dt_s=0.01)

    with pytest.raises(RuntimeError, match="RUNTIME_SESSION_NOT_INITIALIZED"):
        session.cycle(())
    with pytest.raises(RuntimeError, match="RUNTIME_SESSION_NOT_INITIALIZED"):
        session.reset()


def test_initialize_cannot_be_called_twice_without_shutdown() -> None:
    session = RuntimeSession(FakeBackend(), dt_s=0.01)
    session.initialize()

    with pytest.raises(RuntimeError, match="RUNTIME_SESSION_ALREADY_INITIALIZED"):
        session.initialize()


def test_shutdown_delegates_once_and_disallows_further_cycles() -> None:
    backend = FakeBackend()
    session = RuntimeSession(backend, dt_s=0.01)
    session.initialize()
    backend.events.clear()

    session.shutdown()
    session.shutdown()

    assert backend.events == [("shutdown", None)]
    with pytest.raises(RuntimeError, match="RUNTIME_SESSION_NOT_INITIALIZED"):
        session.cycle(())


def test_failed_initial_state_sample_does_not_mark_session_initialized() -> None:
    class FailingReadBackend(FakeBackend):
        def read_state(self) -> BackendState:
            self.events.append(("read_state", "failure"))
            raise RuntimeError("sample failed")

    backend = FailingReadBackend()
    session = RuntimeSession(backend, dt_s=0.01)

    with pytest.raises(RuntimeError, match="sample failed"):
        session.initialize()

    assert session.initialized is False
    assert session.time_s == 0.0
    with pytest.raises(RuntimeError, match="RUNTIME_SESSION_NOT_INITIALIZED"):
        session.cycle(())
