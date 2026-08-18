#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import math
import traceback
from pathlib import Path
from typing import Any

import yaml

from dexterous_robot.backends.isaac.backend import IsaacBackend
from dexterous_robot.backends.isaac.config import (
    TabletopGraspLiftConfig,
    load_isaac_backend_config,
    load_tabletop_grasp_lift_config,
)
from dexterous_robot.config import load_local_asset_config
from dexterous_robot.control.arm import CartesianCarryController, Wam7Kinematics
from dexterous_robot.control.hand import GraspLockController, GraspLockGoal
from dexterous_robot.core import JointPositionCommand, Pose, SkillStatus
from dexterous_robot.devices.arms.wam7 import WAM7_JOINT_NAMES, Wam7Model
from dexterous_robot.devices.hands.linker_l20 import (
    L20_PHYSICAL_JOINTS,
    L20PhysicalTarget21,
    LinkerL20Model,
)
from dexterous_robot.robots import ManipulatorSystem, MountTransform
from dexterous_robot.runtime import RuntimeSession
from dexterous_robot.skills import (
    ArmWaypoint,
    HoldCriteria,
    LiftCriteria,
    LivePoseLiftSkill,
    PreloadGraspCriteria,
    PreloadGraspSkill,
    PreshapeApproachPlan,
    PreshapeApproachSkill,
    SuspendedHoldSkill,
)
from dexterous_robot.tasks import TabletopGraspLiftTask, TaskPhase

_REQUIRED_TRANSFORM_CHECKPOINTS = ("PRE_LIFT", "POST_LIFT", "HOLD_END")


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the M1 WAM7 + Linker L20 Isaac tabletop grasp/lift example.")
    parser.add_argument("--backend-config", type=Path, required=True)
    parser.add_argument("--task-config", type=Path, required=True)
    parser.add_argument("--robot-config", type=Path, required=True)
    parser.add_argument("--local-assets", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout-s", type=float, default=40.0)
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
        raise RuntimeError("M1_GOLDEN_ROBOT_CONFIG_SCHEMA_INVALID")
    mount = raw.get("hand_mount")
    if not isinstance(mount, dict):
        raise RuntimeError("M1_GOLDEN_ROBOT_MOUNT_CONFIG_INVALID")
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


def _build_task(task_cfg: TabletopGraspLiftConfig) -> tuple[TabletopGraspLiftTask, JointPositionCommand]:
    control = task_cfg.control
    hand_open = JointPositionCommand(
        "hand",
        L20_PHYSICAL_JOINTS,
        task_cfg.initial_hand_q_rad,
        profile="hand_open_hold",
    )
    approach = PreshapeApproachSkill(
        plan=PreshapeApproachPlan(
            arm_waypoints=(
                ArmWaypoint("lateral_ready", control.approach.lateral_ready_q_rad, control.approach.waypoint_duration_s),
                ArmWaypoint("transit", control.approach.transit_q_rad, control.approach.waypoint_duration_s),
                ArmWaypoint("pregrasp", control.approach.pregrasp_q_rad, control.approach.waypoint_duration_s),
            ),
            preshape_hand_q_rad=control.approach.preshape_hand_q_rad,
            preshape_duration_s=control.approach.preshape_duration_s,
            grasp_waypoint=ArmWaypoint("grasp", control.approach.grasp_q_rad, control.approach.waypoint_duration_s),
            settle_duration_s=control.approach.settle_duration_s,
            joint_tolerance_rad=control.approach.joint_tolerance_rad,
        ),
        hand_open_command=hand_open,
    )

    base_target = L20PhysicalTarget21(control.grasp.base_preload_hand_q_rad, "mujoco_equal_v1")
    goal = GraspLockGoal(base_target)
    final_hand_hold = GraspLockController().compute(goal)
    grasp = PreloadGraspSkill(
        arm_hold_command=JointPositionCommand(
            "arm", WAM7_JOINT_NAMES, control.approach.grasp_q_rad, profile="arm_carry_position_drive"
        ),
        preshape_hand_command=JointPositionCommand(
            "hand", L20_PHYSICAL_JOINTS, control.approach.preshape_hand_q_rad, profile="hand_open_hold"
        ),
        controller=GraspLockController(),
        goal=goal,
        release_settle_s=control.grasp.release_settle_s,
        preload_duration_s=control.grasp.preload_duration_s,
        lock_ramp_duration_s=control.grasp.lock_ramp_duration_s,
        criteria=PreloadGraspCriteria(
            target_squeeze_n=control.grasp.target_squeeze_n,
            lock_hold_duration_s=control.grasp.lock_hold_duration_s,
        ),
    )

    lift = LivePoseLiftSkill(
        controller=CartesianCarryController(kinematics=Wam7Kinematics()),
        hand_hold_command=final_hand_hold,
        delta_world_m=(0.0, 0.0, control.lift.delta_world_z_m),
        duration_s=control.lift.duration_s,
        criteria=LiftCriteria(
            max_relative_drift_m=control.lift.max_relative_drift_m,
            minimum_object_rise_m=control.lift.minimum_object_rise_m,
            max_table_normal_n=control.lift.max_table_normal_n,
        ),
    )
    hold = SuspendedHoldSkill(
        hand_hold_command=final_hand_hold,
        criteria=HoldCriteria(
            hold_duration_s=control.hold.duration_s,
            table_top_z_m=task_cfg.table_top_world_z_m,
            object_half_height_m=task_cfg.object_dimensions_xyz_m[2] / 2.0,
            minimum_clearance_m=control.hold.minimum_clearance_m,
            max_table_normal_n=control.hold.max_table_normal_n,
            max_relative_drift_m=control.hold.max_relative_drift_m,
        ),
    )
    return TabletopGraspLiftTask(approach=approach, grasp=grasp, lift=lift, hold=hold), final_hand_hold


def _checkpoint_consistent(backend_diagnostics: dict[str, Any], tolerance_m: float) -> bool:
    checkpoints = backend_diagnostics.get("transform_checkpoints")
    if not isinstance(checkpoints, list):
        return False
    by_label = {row.get("label"): row for row in checkpoints if isinstance(row, dict)}
    for label in _REQUIRED_TRANSFORM_CHECKPOINTS:
        row = by_label.get(label)
        if not isinstance(row, dict) or row.get("consistent") is not True:
            return False
        try:
            error = float(row["max_pairwise_position_error_m"])
        except (KeyError, TypeError, ValueError):
            return False
        if not math.isfinite(error) or error > tolerance_m:
            return False
    return True


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2, sort_keys=True, allow_nan=False) + "\n", encoding="utf-8")


def main() -> int:
    args = _parse_args()
    if not math.isfinite(args.timeout_s) or args.timeout_s <= 0.0:
        raise SystemExit("M1_GOLDEN_TIMEOUT_INVALID")
    args.output.parent.mkdir(parents=True, exist_ok=True)

    summary: dict[str, Any] = {
        "status": "BLOCKED",
        "runtime_status": "NOT_STARTED",
        "wam_l20_loaded": False,
        "grasp_lock_success": False,
        "object_left_table": False,
        "cuboid_center_z_rise_m": 0.0,
        "suspended_hold_s": 0.0,
        "transform_consistency_pass": False,
        "headless": bool(args.headless),
        "timeout_s": float(args.timeout_s),
        "phase_history": [],
    }
    backend: IsaacBackend | None = None
    session: RuntimeSession | None = None
    assets = None
    return_code = 1

    try:
        backend_cfg = load_isaac_backend_config(args.backend_config)
        task_cfg = load_tabletop_grasp_lift_config(args.task_config)
        assets = load_local_asset_config(args.local_assets)
        robot = _load_robot(args.robot_config)
        task, final_hand_hold = _build_task(task_cfg)
        summary["final_hand_lock_q_rad"] = list(final_hand_hold.position_rad)
        summary["asset_hashes_before"] = {
            "wam_runtime": _sha256(assets.wam_runtime),
            "l20_runtime": _sha256(assets.l20_runtime),
        }

        backend = IsaacBackend(
            robot=robot,
            backend_config=backend_cfg,
            task_config=task_cfg,
            assets=assets,
            headless=args.headless,
        )
        session = RuntimeSession(backend, backend_cfg.physics_dt_s)
        snapshot = session.initialize()
        initial_object_z = float(snapshot.body_poses["object"].position_xyz_m[2])
        max_object_z = initial_object_z
        summary["initial_object_center_z_m"] = initial_object_z
        summary["phase_history"].append({"time_s": snapshot.time_s, "phase": task.phase.value})

        diagnostics = backend.diagnostics
        combined = diagnostics.get("combined_articulation", {})
        summary["wam_l20_loaded"] = bool(
            isinstance(combined, dict)
            and combined.get("count") == 1
            and combined.get("max_dofs") == 28
        )

        reached_lift = False
        hold_started_s: float | None = None
        terminal_result = None
        while snapshot.time_s <= args.timeout_s:
            old_phase = task.phase
            result, commands = task.step(snapshot)
            new_phase = task.phase
            snapshot = session.cycle(commands)
            max_object_z = max(max_object_z, float(snapshot.body_poses["object"].position_xyz_m[2]))

            if new_phase is not old_phase:
                summary["phase_history"].append({
                    "time_s": snapshot.time_s,
                    "from": old_phase.value,
                    "phase": new_phase.value,
                    "skill_status": result.status.value,
                    "reason": result.reason.value,
                })
                if old_phase is TaskPhase.GRASP and new_phase is TaskPhase.LIFT:
                    reached_lift = True
                    transition_squeeze = float(snapshot.signals["opposing_y_squeeze_n"])
                    summary["grasp_lock_transition_squeeze_n"] = transition_squeeze
                    summary["grasp_lock_target_squeeze_n"] = float(task_cfg.control.grasp.target_squeeze_n)
                    summary["grasp_lock_squeeze_quality_telemetry"] = bool(
                        transition_squeeze >= task_cfg.control.grasp.target_squeeze_n
                    )
                    summary["grasp_lock_squeeze_gate_hard"] = False
                    backend.capture_transform_checkpoint("PRE_LIFT")
                elif old_phase is TaskPhase.LIFT and new_phase is TaskPhase.HOLD:
                    hold_started_s = snapshot.time_s
                    backend.capture_transform_checkpoint("POST_LIFT")
                elif old_phase is TaskPhase.HOLD and new_phase is TaskPhase.SUCCESS:
                    backend.capture_transform_checkpoint("HOLD_END")

            if new_phase in (TaskPhase.SUCCESS, TaskPhase.FAILURE):
                terminal_result = result
                break

        if terminal_result is None:
            summary["runtime_status"] = "TIMEOUT"
            summary["task_phase"] = task.phase.value
            summary["task_failure_reason"] = "TIMEOUT"
        else:
            summary["runtime_status"] = "TASK_TERMINAL"
            summary["task_phase"] = task.phase.value
            summary["task_result_status"] = terminal_result.status.value
            summary["task_failure_reason"] = terminal_result.reason.value
            summary["task_message"] = terminal_result.message

        final_object_pose = snapshot.body_poses["object"]
        final_object_z = float(final_object_pose.position_xyz_m[2])
        final_table_normal = float(snapshot.signals["object_table_normal_n"])
        final_squeeze = float(snapshot.signals["opposing_y_squeeze_n"])
        rise = final_object_z - initial_object_z
        object_bottom = final_object_z - task_cfg.object_dimensions_xyz_m[2] / 2.0
        summary.update({
            "grasp_lock_success": reached_lift,
            "final_object_center_z_m": final_object_z,
            "max_object_center_z_m": max_object_z,
            "cuboid_center_z_rise_m": rise,
            "max_cuboid_center_z_rise_m": max_object_z - initial_object_z,
            "final_object_bottom_z_m": object_bottom,
            "final_object_table_normal_n": final_table_normal,
            "final_opposing_y_squeeze_n": final_squeeze,
            "object_left_table": bool(
                object_bottom >= task_cfg.table_top_world_z_m + task_cfg.control.hold.minimum_clearance_m
                and final_table_normal <= task_cfg.control.hold.max_table_normal_n
            ),
            "suspended_hold_s": 0.0 if hold_started_s is None else max(0.0, snapshot.time_s - hold_started_s),
            "backend_diagnostics": backend.diagnostics,
            "asset_hashes_after": {
                "wam_runtime": _sha256(assets.wam_runtime),
                "l20_runtime": _sha256(assets.l20_runtime),
            },
        })
        summary["transform_consistency_pass"] = _checkpoint_consistent(
            backend.diagnostics,
            backend_cfg.transform_sync.position_tolerance_m,
        )
        summary["status"] = "PASS" if task.phase is TaskPhase.SUCCESS else "BLOCKED"
        return_code = 0
    except Exception as exc:
        summary["runtime_status"] = "EXCEPTION"
        summary["error"] = f"{type(exc).__name__}:{exc}"
        summary["traceback"] = traceback.format_exc()
        if assets is not None:
            try:
                summary["asset_hashes_after"] = {
                    "wam_runtime": _sha256(assets.wam_runtime),
                    "l20_runtime": _sha256(assets.l20_runtime),
                }
            except Exception as hash_exc:
                summary["asset_hash_after_error"] = f"{type(hash_exc).__name__}:{hash_exc}"
        return_code = 1
    finally:
        if backend is not None:
            try:
                summary["backend_diagnostics"] = backend.diagnostics
            except Exception as exc:
                summary["backend_diagnostics_error"] = f"{type(exc).__name__}:{exc}"
        _write_json(args.output, summary)
        print(json.dumps(summary, indent=2, sort_keys=True, allow_nan=False), flush=True)

        try:
            if session is not None and session.initialized:
                session.shutdown()
            elif backend is not None:
                backend.shutdown(force=True)
        except Exception as exc:
            summary["shutdown_error"] = f"{type(exc).__name__}:{exc}"
            return_code = 1
        _write_json(args.output, summary)

    return return_code


if __name__ == "__main__":
    raise SystemExit(main())
