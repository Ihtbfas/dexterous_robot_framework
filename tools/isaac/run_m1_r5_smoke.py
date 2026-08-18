#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import math
import traceback
from pathlib import Path

import yaml

from dexterous_robot.backends.isaac.backend import IsaacBackend
from dexterous_robot.backends.isaac.config import load_isaac_backend_config, load_tabletop_grasp_lift_config
from dexterous_robot.config import load_local_asset_config
from dexterous_robot.core import Pose
from dexterous_robot.devices.arms.wam7 import Wam7Model
from dexterous_robot.devices.hands.linker_l20 import LinkerL20Model
from dexterous_robot.robots import ManipulatorSystem, MountTransform
from dexterous_robot.runtime import RuntimeSession


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the single-runtime M1-R5 Isaac backend smoke test.")
    parser.add_argument("--backend-config", type=Path, required=True)
    parser.add_argument("--task-config", type=Path, required=True)
    parser.add_argument("--robot-config", type=Path, required=True)
    parser.add_argument("--local-assets", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--headless", action="store_true")
    return parser.parse_args()


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _load_robot(path: Path) -> ManipulatorSystem:
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(raw, dict) or raw.get("schema_version") != 1 or raw.get("kind") != "ManipulatorSystem":
        raise RuntimeError("M1_R5_ROBOT_CONFIG_SCHEMA_INVALID")
    mount = raw.get("hand_mount")
    if not isinstance(mount, dict):
        raise RuntimeError("M1_R5_ROBOT_MOUNT_CONFIG_INVALID")
    arm = Wam7Model()
    hand = LinkerL20Model(coupling_profile="mujoco_equal_v1")
    pose = Pose(
        tuple(float(x) for x in mount["position_xyz_m"]),
        tuple(float(x) for x in mount["quaternion_xyzw"]),
        str(mount["parent_frame"]),
    )
    return ManipulatorSystem(
        system_id=str(raw["system_id"]),
        arm=arm,
        hand=hand,
        hand_mount=MountTransform(str(mount["parent_frame"]), str(mount["child_frame"]), pose),
        tcp_frame=str(raw["tcp_frame"]),
    )


def _state_summary(snapshot, robot: ManipulatorSystem) -> dict[str, object]:
    arm_state = snapshot.device_states[robot.arm.device_id]
    hand_state = snapshot.device_states[robot.hand.device_id]
    object_pose = snapshot.body_poses["object"]
    numbers = tuple(object_pose.position_xyz_m) + tuple(object_pose.quaternion_xyzw)
    return {
        "time_s": snapshot.time_s,
        "arm_width": len(arm_state.names),
        "hand_width": len(hand_state.names),
        "object_position_world_m": list(object_pose.position_xyz_m),
        "object_quaternion_xyzw": list(object_pose.quaternion_xyzw),
        "object_finite": all(math.isfinite(value) for value in numbers),
        "object_table_normal_n": snapshot.signals["object_table_normal_n"],
        "opposing_y_squeeze_n": snapshot.signals["opposing_y_squeeze_n"],
    }


def main() -> int:
    args = _parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    result: dict[str, object] = {
        "status": "BLOCKED",
        "classification": "M1_R5_ISAAC_RUNTIME_SMOKE_BLOCKED",
        "initialized": False,
        "cycles_completed": 0,
        "headless": bool(args.headless),
    }
    backend = None
    session = None
    assets = None
    try:
        backend_cfg = load_isaac_backend_config(args.backend_config)
        task_cfg = load_tabletop_grasp_lift_config(args.task_config)
        assets = load_local_asset_config(args.local_assets)
        robot = _load_robot(args.robot_config)
        before_hashes = {
            "wam_runtime": _sha256(assets.wam_runtime),
            "l20_runtime": _sha256(assets.l20_runtime),
        }
        result["asset_hashes_before"] = before_hashes
        backend = IsaacBackend(
            robot=robot,
            backend_config=backend_cfg,
            task_config=task_cfg,
            assets=assets,
            headless=args.headless,
        )
        session = RuntimeSession(backend, backend_cfg.physics_dt_s)
        initial = session.initialize()
        result["initialized"] = True
        result["initial_state"] = _state_summary(initial, robot)
        snapshot = initial
        for cycle in range(10):
            snapshot = session.cycle(())
            result["cycles_completed"] = cycle + 1
        result["final_state"] = _state_summary(snapshot, robot)
        backend.capture_transform_checkpoint("POST_SMOKE_10_STEPS")
        result["backend_diagnostics"] = backend.diagnostics
        result["asset_hashes_after"] = {
            "wam_runtime": _sha256(assets.wam_runtime),
            "l20_runtime": _sha256(assets.l20_runtime),
        }
        result["status"] = "PASS"
        result["classification"] = "M1_R5_ISAAC_RUNTIME_SMOKE_PASS"
        return_code = 0
    except Exception as exc:
        result["error"] = f"{type(exc).__name__}:{exc}"
        result["traceback"] = traceback.format_exc()
        if assets is not None:
            try:
                result["asset_hashes_after"] = {
                    "wam_runtime": _sha256(assets.wam_runtime),
                    "l20_runtime": _sha256(assets.l20_runtime),
                }
            except Exception as hash_exc:
                result["asset_hash_after_error"] = f"{type(hash_exc).__name__}:{hash_exc}"
        return_code = 1
    finally:
        # Persist the runtime outcome before closing Kit.  On the user's Isaac
        # runtime SimulationApp.close() can terminate the interpreter, so writing
        # only after shutdown can erase the only useful exception evidence.
        if backend is not None:
            try:
                result["backend_diagnostics"] = backend.diagnostics
            except Exception as exc:
                result["backend_diagnostics_error"] = f"{type(exc).__name__}:{exc}"
        args.output.write_text(json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
        print(json.dumps(result, indent=2, sort_keys=True, allow_nan=False), flush=True)

        if session is not None:
            try:
                session.shutdown()
            except Exception as exc:
                result["session_shutdown_error"] = f"{type(exc).__name__}:{exc}"
                return_code = 1
        if backend is not None:
            try:
                backend.shutdown(force=True)
            except Exception as exc:
                result["backend_shutdown_error"] = f"{type(exc).__name__}:{exc}"
                return_code = 1

        # If Kit returned normally from cleanup, refresh the file with any
        # shutdown diagnostics.  The pre-cleanup write above remains the hard
        # evidence path if close() terminates the process.
        args.output.write_text(json.dumps(result, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")
    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
