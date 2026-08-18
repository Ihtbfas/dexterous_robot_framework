from __future__ import annotations

from math import isfinite
from typing import Sequence

from dexterous_robot.backends.base import Backend, BackendState, Command

from .snapshot import RuntimeSnapshot


class RuntimeSession:
    def __init__(self, backend: Backend, dt_s: float) -> None:
        if not isinstance(backend, Backend):
            raise TypeError("RUNTIME_SESSION_BACKEND_INVALID")
        if isinstance(dt_s, bool):
            raise ValueError("RUNTIME_SESSION_DT_INVALID")
        try:
            normalized_dt = float(dt_s)
        except (TypeError, ValueError) as exc:
            raise ValueError("RUNTIME_SESSION_DT_INVALID") from exc
        if not isfinite(normalized_dt) or normalized_dt <= 0.0:
            raise ValueError("RUNTIME_SESSION_DT_INVALID")
        self._backend = backend
        self._dt_s = normalized_dt
        self._cycle_index = 0
        self._initialized = False

    @property
    def dt_s(self) -> float:
        return self._dt_s

    @property
    def time_s(self) -> float:
        return self._cycle_index * self._dt_s

    @property
    def initialized(self) -> bool:
        return self._initialized

    def _snapshot(self, state: BackendState) -> RuntimeSnapshot:
        return RuntimeSnapshot(
            time_s=self.time_s,
            dt_s=self._dt_s,
            device_states=state.device_states,
            body_poses=state.body_poses,
            signals=state.signals,
        )

    def initialize(self) -> RuntimeSnapshot:
        if self._initialized:
            raise RuntimeError("RUNTIME_SESSION_ALREADY_INITIALIZED")
        self._backend.initialize()
        self._cycle_index = 0
        snapshot = self._snapshot(self._backend.read_state())
        self._initialized = True
        return snapshot

    def reset(self) -> RuntimeSnapshot:
        self._require_initialized()
        self._backend.reset()
        self._cycle_index = 0
        return self._snapshot(self._backend.read_state())

    def cycle(self, commands: Sequence[Command]) -> RuntimeSnapshot:
        self._require_initialized()
        self._backend.apply(tuple(commands))
        self._backend.step(self._dt_s)
        self._cycle_index += 1
        return self._snapshot(self._backend.read_state())

    def shutdown(self) -> None:
        if not self._initialized:
            return
        self._backend.shutdown()
        self._initialized = False

    def _require_initialized(self) -> None:
        if not self._initialized:
            raise RuntimeError("RUNTIME_SESSION_NOT_INITIALIZED")
