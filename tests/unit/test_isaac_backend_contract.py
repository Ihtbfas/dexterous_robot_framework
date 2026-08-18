from __future__ import annotations

import sys
from pathlib import Path

import pytest

from dexterous_robot.backends.base import Backend
from dexterous_robot.config import LocalAssetConfig
from dexterous_robot.core import Pose
from dexterous_robot.devices.arms.wam7 import Wam7Model
from dexterous_robot.devices.hands.linker_l20 import LinkerL20Model
from dexterous_robot.robots import ManipulatorSystem, MountTransform


def _robot() -> ManipulatorSystem:
    arm = Wam7Model()
    hand = LinkerL20Model(coupling_profile="mujoco_equal_v1")
    return ManipulatorSystem(
        "wam7_linker_l20",
        arm,
        hand,
        MountTransform(arm.flange_frame, "l20_base", Pose((0.0, 0.0, 0.00991000205278397), (0.0, 0.0, 0.0, 1.0), arm.flange_frame)),
        "l20_tcp",
    )


def test_isaac_backend_constructs_without_loading_simulator_modules():
    from dexterous_robot.backends.isaac.backend import IsaacBackend
    from dexterous_robot.backends.isaac.config import load_isaac_backend_config, load_tabletop_grasp_lift_config

    root = Path(__file__).resolve().parents[2]
    backend = IsaacBackend(
        robot=_robot(),
        backend_config=load_isaac_backend_config(root / "configs/backends/isaac.yaml"),
        task_config=load_tabletop_grasp_lift_config(root / "configs/tasks/tabletop_grasp_lift.yaml"),
        assets=LocalAssetConfig(Path("/tmp/wam.usda"), Path("/tmp/l20.usda")),
        headless=True,
    )
    assert isinstance(backend, Backend)
    assert backend.initialized is False
    for prefix in ("omni", "isaacsim", "pxr", "warp", "usdrt"):
        assert prefix not in sys.modules


def test_backend_rejects_calls_before_initialize():
    from dexterous_robot.backends.isaac.backend import IsaacBackend
    from dexterous_robot.backends.isaac.config import load_isaac_backend_config, load_tabletop_grasp_lift_config

    root = Path(__file__).resolve().parents[2]
    backend = IsaacBackend(
        robot=_robot(),
        backend_config=load_isaac_backend_config(root / "configs/backends/isaac.yaml"),
        task_config=load_tabletop_grasp_lift_config(root / "configs/tasks/tabletop_grasp_lift.yaml"),
        assets=LocalAssetConfig(Path("/tmp/wam.usda"), Path("/tmp/l20.usda")),
        headless=True,
    )
    with pytest.raises(RuntimeError, match="ISAAC_BACKEND_NOT_INITIALIZED"):
        backend.read_state()
    with pytest.raises(RuntimeError, match="ISAAC_BACKEND_NOT_INITIALIZED"):
        backend.apply(())
    with pytest.raises(RuntimeError, match="ISAAC_BACKEND_NOT_INITIALIZED"):
        backend.step(1.0 / 120.0)


def test_initialize_failure_preserves_runtime_for_diagnostic_cleanup(monkeypatch):
    import types
    from dexterous_robot.backends.isaac.backend import IsaacBackend
    from dexterous_robot.backends.isaac.config import load_isaac_backend_config, load_tabletop_grasp_lift_config

    class FakeApp:
        def __init__(self, _settings):
            self.closed = False

        def close(self):
            self.closed = True

    root = Path(__file__).resolve().parents[2]
    backend = IsaacBackend(
        robot=_robot(),
        backend_config=load_isaac_backend_config(root / "configs/backends/isaac.yaml"),
        task_config=load_tabletop_grasp_lift_config(root / "configs/tasks/tabletop_grasp_lift.yaml"),
        assets=LocalAssetConfig(Path("/tmp/wam.usda"), Path("/tmp/l20.usda")),
        headless=True,
    )
    monkeypatch.setitem(sys.modules, "isaacsim", types.SimpleNamespace(SimulationApp=FakeApp))
    monkeypatch.setattr(backend, "_verify_assets", lambda: {"wam_runtime": "w", "l20_runtime": "h"})

    def fail_after_kit(_hashes):
        backend._diagnostics["phase"] = "physics_initialized"
        raise RuntimeError("synthetic-runtime-failure")

    monkeypatch.setattr(backend, "_initialize_after_kit", fail_after_kit)

    with pytest.raises(RuntimeError, match="synthetic-runtime-failure"):
        backend.initialize()

    assert backend.initialized is False
    assert backend.diagnostics["phase"] == "physics_initialized"
    assert backend.diagnostics["failure"] == "RuntimeError:synthetic-runtime-failure"
    assert backend._app is not None
    app = backend._app
    backend.shutdown(force=True)
    assert app.closed is True


def test_smoke_persists_failure_receipt_before_backend_shutdown(monkeypatch, tmp_path):
    import importlib.util
    script = Path(__file__).resolve().parents[2] / "tools/isaac/run_m1_r5_smoke.py"
    spec = importlib.util.spec_from_file_location("drf_m1_r5_smoke_test_module", script)
    assert spec is not None and spec.loader is not None
    smoke = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(smoke)

    wam = tmp_path / "wam.usda"
    l20 = tmp_path / "l20.usda"
    wam.write_text("wam", encoding="utf-8")
    l20.write_text("l20", encoding="utf-8")
    output = tmp_path / "receipt.json"

    class FakeBackend:
        def __init__(self, **_kwargs):
            self.initialized = True
            self.diagnostics = {"phase": "synthetic_runtime_failure"}

        def shutdown(self, force=False):
            raise SystemExit(0)

    class FakeSession:
        def __init__(self, backend, _dt):
            self.backend = backend

        def initialize(self):
            raise RuntimeError("synthetic-runtime-failure")

        def shutdown(self):
            return None

    monkeypatch.setattr(smoke, "IsaacBackend", FakeBackend)
    monkeypatch.setattr(smoke, "RuntimeSession", FakeSession)
    monkeypatch.setattr(smoke, "load_isaac_backend_config", lambda _p: type("Cfg", (), {"physics_dt_s": 1 / 120})())
    monkeypatch.setattr(smoke, "load_tabletop_grasp_lift_config", lambda _p: object())
    monkeypatch.setattr(smoke, "load_local_asset_config", lambda _p: LocalAssetConfig(wam, l20))
    monkeypatch.setattr(smoke, "_load_robot", lambda _p: object())
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "run_m1_r5_smoke.py",
            "--backend-config", str(tmp_path / "backend.yaml"),
            "--task-config", str(tmp_path / "task.yaml"),
            "--robot-config", str(tmp_path / "robot.yaml"),
            "--local-assets", str(tmp_path / "assets.yaml"),
            "--output", str(output),
        ],
    )

    with pytest.raises(SystemExit):
        smoke.main()

    payload = __import__("json").loads(output.read_text(encoding="utf-8"))
    assert payload["status"] == "BLOCKED"
    assert payload["error"] == "RuntimeError:synthetic-runtime-failure"
    assert payload["backend_diagnostics"]["phase"] == "synthetic_runtime_failure"


def test_r5_transform_audit_is_passive_and_does_not_teleport_object():
    source = (Path(__file__).resolve().parents[2] / "src/dexterous_robot/backends/isaac/backend.py").read_text(encoding="utf-8")
    assert "_run_transform_probe" not in source
    assert "self._object_view.set_transforms" not in source
    assert "capture_transform_checkpoint" in source


def test_smoke_records_transform_checkpoint_after_ten_runtime_cycles(monkeypatch, tmp_path):
    import importlib.util
    import json

    script = Path(__file__).resolve().parents[2] / "tools/isaac/run_m1_r5_smoke.py"
    spec = importlib.util.spec_from_file_location("drf_m1_r5_smoke_checkpoint_test_module", script)
    assert spec is not None and spec.loader is not None
    smoke = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(smoke)

    wam = tmp_path / "wam.usda"
    l20 = tmp_path / "l20.usda"
    wam.write_text("wam", encoding="utf-8")
    l20.write_text("l20", encoding="utf-8")
    output = tmp_path / "receipt.json"
    events = []

    class FakeBackend:
        def __init__(self, **_kwargs):
            self.diagnostics = {
                "combined_articulation": {"count": 1, "max_dofs": 28, "backend_joint_names": [f"j{i}" for i in range(28)]},
                "transform_release": {"position_error_m": 0.0, "released_properties": ["xformOp:translate", "xformOp:orient"]},
                "transform_checkpoints": [],
            }

        def capture_transform_checkpoint(self, label):
            events.append(("checkpoint", label))
            row = {
                "label": label,
                "simulation_time_s": 10 / 120,
                "consistent": True,
                "max_pairwise_position_error_m": 0.0,
                "positions": {name: (0.0, 0.0, 1.0) for name in ("tensor", "physx", "usd", "fabric")},
            }
            self.diagnostics["transform_checkpoints"].append(row)
            return row

        def shutdown(self, force=False):
            return None

    state = type("State", (), {
        "device_states": {
            "arm": type("Joint", (), {"names": tuple(f"a{i}" for i in range(7))})(),
            "hand": type("Joint", (), {"names": tuple(f"h{i}" for i in range(21))})(),
        },
        "body_poses": {"object": Pose((0.0, 0.0, 1.0), (0.0, 0.0, 0.0, 1.0), "world")},
        "signals": {"object_table_normal_n": 0.0, "opposing_y_squeeze_n": 0.0},
        "time_s": 0.0,
    })()

    class FakeSession:
        def __init__(self, backend, _dt):
            self.backend = backend
            self.cycles = 0

        def initialize(self):
            events.append(("initialize", 0))
            return state

        def cycle(self, _commands):
            self.cycles += 1
            events.append(("cycle", self.cycles))
            return state

        def shutdown(self):
            return None

    robot = type("Robot", (), {
        "arm": type("Arm", (), {"device_id": "arm"})(),
        "hand": type("Hand", (), {"device_id": "hand"})(),
    })()
    monkeypatch.setattr(smoke, "IsaacBackend", FakeBackend)
    monkeypatch.setattr(smoke, "RuntimeSession", FakeSession)
    monkeypatch.setattr(smoke, "load_isaac_backend_config", lambda _p: type("Cfg", (), {"physics_dt_s": 1 / 120})())
    monkeypatch.setattr(smoke, "load_tabletop_grasp_lift_config", lambda _p: object())
    monkeypatch.setattr(smoke, "load_local_asset_config", lambda _p: LocalAssetConfig(wam, l20))
    monkeypatch.setattr(smoke, "_load_robot", lambda _p: robot)
    monkeypatch.setattr(sys, "argv", [
        "run_m1_r5_smoke.py",
        "--backend-config", str(tmp_path / "backend.yaml"),
        "--task-config", str(tmp_path / "task.yaml"),
        "--robot-config", str(tmp_path / "robot.yaml"),
        "--local-assets", str(tmp_path / "assets.yaml"),
        "--output", str(output),
    ])

    assert smoke.main() == 0
    payload = json.loads(output.read_text(encoding="utf-8"))
    assert payload["status"] == "PASS"
    assert events[-1] == ("checkpoint", "POST_SMOKE_10_STEPS")
    assert sum(1 for kind, _ in events if kind == "cycle") == 10


def test_r8_initialization_and_reset_apply_open_hand_profile_not_only_cache_it():
    source = (Path(__file__).resolve().parents[2] / "src/dexterous_robot/backends/isaac/backend.py").read_text(encoding="utf-8")
    assert source.count('self._apply_hand_drive_profile("hand_open_hold", wp, indices)') >= 2
    assert source.count('self._active_hand_profile = "hand_open_hold"') >= 2
