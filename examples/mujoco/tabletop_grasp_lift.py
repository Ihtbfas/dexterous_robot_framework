#!/usr/bin/env python3
from __future__ import annotations

import argparse
import contextlib
import json
import math
import time
from pathlib import Path
from typing import Any

import yaml

from dexterous_robot.assets import load_asset_registry, load_asset_selection
from dexterous_robot.backends.mujoco import (
    MuJoCoBackend,
    load_mujoco_backend_config,
)
from dexterous_robot.config.tasks import (
    TabletopGraspLiftConfig,
    load_tabletop_grasp_lift_config,
)
from dexterous_robot.control.arm import CartesianCarryController, Wam7Kinematics
from dexterous_robot.control.hand import GraspLockController, GraspLockGoal
from dexterous_robot.core import JointPositionCommand, Pose, SkillStatus
from dexterous_robot.devices.arms.wam7 import WAM7_JOINT_NAMES, Wam7Model
from dexterous_robot.devices.hands.linker_l20 import (
    L20_PHYSICAL_JOINTS,
    L20PhysicalTarget21,
    LinkerL20Model,
)
from dexterous_robot.motion import (
    load_cartesian_kinematic_limits,
    load_joint_kinematic_limits,
    load_motion_profiles,
    resolve_cartesian_limits,
    resolve_joint_limits,
)
from dexterous_robot.robots import ManipulatorSystem, MountTransform
from dexterous_robot.runtime import RuntimeSession, RuntimeSnapshot
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


ROOT = Path(__file__).resolve().parents[2]


def _load_backend(
    *,
    asset_registry: Path,
    asset_root_config: Path,
    asset_selection: Path,
    robot_config: Path,
    backend_config: Path,
) -> MuJoCoBackend:
    registry = load_asset_registry(asset_registry, asset_root_config)
    selection = load_asset_selection(asset_selection)
    resolved = registry.resolve_selection(selection, verify_hash=True)
    cfg = load_mujoco_backend_config(backend_config)

    raw = yaml.safe_load(robot_config.read_text(encoding="utf-8"))
    if not isinstance(raw, dict):
        raise ValueError("MUJOCO_DEMO_ROBOT_CONFIG_INVALID")
    mount = raw.get("hand_mount")
    if not isinstance(mount, dict):
        raise ValueError("MUJOCO_DEMO_HAND_MOUNT_INVALID")

    arm = Wam7Model()
    hand = LinkerL20Model(coupling_profile="mujoco_equal_v1")
    if mount.get("parent_frame") != arm.flange_frame:
        raise ValueError("MUJOCO_DEMO_MOUNT_PARENT_MISMATCH")
    if mount.get("child_frame") != cfg.frames.l20_base:
        raise ValueError("MUJOCO_DEMO_MOUNT_CHILD_MISMATCH")

    robot = ManipulatorSystem(
        "wam7_linker_l20",
        arm,
        hand,
        MountTransform(
            arm.flange_frame,
            cfg.frames.l20_base,
            Pose(
                tuple(float(value) for value in mount["position_xyz_m"]),
                tuple(float(value) for value in mount["quaternion_xyzw"]),
                arm.flange_frame,
            ),
        ),
        str(raw.get("tcp_frame", "l20_tcp")),
    )
    return MuJoCoBackend(
        robot=robot,
        backend_config=cfg,
        arm_runtime=resolved[cfg.model.arm_asset_role],
        hand_runtime=resolved[cfg.model.hand_asset_role],
    )


def _build_approach(
    *,
    task_config: Path,
    joint_limits: Path,
    motion_profiles: Path,
) -> tuple[TabletopGraspLiftConfig, PreshapeApproachSkill]:
    profiles = load_motion_profiles(motion_profiles)
    task_cfg = load_tabletop_grasp_lift_config(
        task_config,
        motion_profiles=profiles,
    )
    base_limits = load_joint_kinematic_limits(
        joint_limits,
        expected_joint_names=WAM7_JOINT_NAMES,
    )
    approach_limits = resolve_joint_limits(
        base_limits,
        profiles.joint(task_cfg.control.approach.motion_profile),
    )
    control = task_cfg.control.approach
    hand_open = JointPositionCommand(
        "hand",
        L20_PHYSICAL_JOINTS,
        task_cfg.initial_hand_q_rad,
        profile="hand_open_hold",
    )
    approach = PreshapeApproachSkill(
        plan=PreshapeApproachPlan(
            arm_waypoints=(
                ArmWaypoint("lateral_ready", control.lateral_ready_q_rad),
                ArmWaypoint("transit", control.transit_q_rad),
                ArmWaypoint("pregrasp", control.pregrasp_q_rad),
            ),
            preshape_hand_q_rad=control.preshape_hand_q_rad,
            preshape_duration_s=control.preshape_duration_s,
            grasp_waypoint=ArmWaypoint("grasp", control.grasp_q_rad),
            settle_duration_s=control.settle_duration_s,
            joint_tolerance_rad=control.joint_tolerance_rad,
        ),
        hand_open_command=hand_open,
        joint_limits=approach_limits,
    )
    return task_cfg, approach


def _build_task(
    *,
    backend: MuJoCoBackend,
    task_cfg: TabletopGraspLiftConfig,
    approach: PreshapeApproachSkill,
    cartesian_limits_path: Path,
    motion_profiles_path: Path,
) -> tuple[TabletopGraspLiftTask, PreloadGraspSkill, LivePoseLiftSkill]:
    control = task_cfg.control
    coupling_profile = backend._robot.hand.coupling_profile

    target = L20PhysicalTarget21(
        control.grasp.base_preload_hand_q_rad,
        coupling_profile,
    )
    goal = GraspLockGoal(target)
    final_hand_hold = GraspLockController().compute(goal)

    grasp = PreloadGraspSkill(
        arm_hold_command=JointPositionCommand(
            "arm",
            WAM7_JOINT_NAMES,
            control.approach.grasp_q_rad,
            profile="arm_carry_position_drive",
        ),
        preshape_hand_command=JointPositionCommand(
            "hand",
            L20_PHYSICAL_JOINTS,
            control.approach.preshape_hand_q_rad,
            profile="hand_open_hold",
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

    profiles = load_motion_profiles(motion_profiles_path)
    base_cartesian_limits = load_cartesian_kinematic_limits(
        cartesian_limits_path
    )
    carry_cartesian_limits = resolve_cartesian_limits(
        base_cartesian_limits,
        profiles.cartesian(control.lift.motion_profile),
    )
    lift = LivePoseLiftSkill(
        controller=CartesianCarryController(kinematics=Wam7Kinematics()),
        hand_hold_command=final_hand_hold,
        delta_world_m=(0.0, 0.0, control.lift.delta_world_z_m),
        cartesian_limits=carry_cartesian_limits,
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
    return (
        TabletopGraspLiftTask(
            approach=approach,
            grasp=grasp,
            lift=lift,
            hold=hold,
        ),
        grasp,
        lift,
    )


def _seed_initial_state(
    backend: MuJoCoBackend,
    task_cfg: TabletopGraspLiftConfig,
) -> None:
    if (
        backend._routing is None
        or backend._data is None
        or backend._mujoco is None
    ):
        raise RuntimeError("MUJOCO_DEMO_BACKEND_NOT_READY")

    for joint_name, value in zip(
        WAM7_JOINT_NAMES,
        task_cfg.initial_wam_q_rad,
        strict=True,
    ):
        address = backend._routing.joint_by_name[joint_name]
        backend._data.qpos[address.qpos_adr] = float(value)
        backend._data.qvel[address.qvel_adr] = 0.0

    for joint_name, value in zip(
        L20_PHYSICAL_JOINTS,
        task_cfg.initial_hand_q_rad,
        strict=True,
    ):
        address = backend._routing.joint_by_name[joint_name]
        backend._data.qpos[address.qpos_adr] = float(value)
        backend._data.qvel[address.qvel_adr] = 0.0

    backend._data.time = 0.0
    backend._mujoco.mj_forward(backend._model, backend._data)


def _snapshot_after_seed(
    session: RuntimeSession,
    backend: MuJoCoBackend,
) -> RuntimeSnapshot:
    state = backend.read_state()
    return RuntimeSnapshot(
        time_s=session.time_s,
        dt_s=session.dt_s,
        device_states=state.device_states,
        body_poses=state.body_poses,
        signals=state.signals,
    )


def _configure_camera(viewer: Any) -> None:
    try:
        viewer.cam.lookat[:] = (0.62, -0.14, 1.03)
        viewer.cam.distance = 1.25
        viewer.cam.azimuth = 135.0
        viewer.cam.elevation = -18.0
    except Exception:
        pass


def _viewer_context(backend: MuJoCoBackend, enabled: bool):
    if not enabled:
        return contextlib.nullcontext(None)
    import mujoco.viewer

    return mujoco.viewer.launch_passive(backend._model, backend._data)


def _sleep_to_realtime(
    *,
    started_at: float,
    dt_s: float,
    realtime_scale: float,
) -> None:
    target = dt_s * realtime_scale
    remaining = target - (time.monotonic() - started_at)
    if remaining > 0.0:
        time.sleep(remaining)


def run(args: argparse.Namespace) -> dict[str, object]:
    if not math.isfinite(args.dt_s) or args.dt_s <= 0.0:
        raise ValueError("MUJOCO_DEMO_DT_INVALID")
    if not math.isfinite(args.timeout_s) or args.timeout_s <= 0.0:
        raise ValueError("MUJOCO_DEMO_TIMEOUT_INVALID")
    if not math.isfinite(args.realtime_scale) or args.realtime_scale <= 0.0:
        raise ValueError("MUJOCO_DEMO_REALTIME_SCALE_INVALID")

    task_cfg, approach = _build_approach(
        task_config=args.task_config,
        joint_limits=args.joint_limits,
        motion_profiles=args.motion_profiles,
    )
    backend = _load_backend(
        asset_registry=args.asset_registry,
        asset_root_config=args.asset_root_config,
        asset_selection=args.asset_selection,
        robot_config=args.robot_config,
        backend_config=args.backend_config,
    )
    backend.configure_tabletop_scene(task_cfg)
    task, grasp, lift = _build_task(
        backend=backend,
        task_cfg=task_cfg,
        approach=approach,
        cartesian_limits_path=args.cartesian_limits,
        motion_profiles_path=args.motion_profiles,
    )

    session = RuntimeSession(backend, args.dt_s)
    snapshot = session.initialize()

    try:
        _seed_initial_state(backend, task_cfg)
        snapshot = _snapshot_after_seed(session, backend)

        initial_object_z = float(
            snapshot.body_poses["object"].position_xyz_m[2]
        )
        max_object_z = initial_object_z
        phase_history = [task.phase.value]
        terminal_result = None
        hold_started_s: float | None = None
        viewer_closed_early = False

        with _viewer_context(backend, args.viewer) as viewer:
            if viewer is not None:
                _configure_camera(viewer)
                viewer.sync()
                if args.pre_roll_s > 0.0:
                    time.sleep(args.pre_roll_s)

            max_cycles = int(math.ceil(args.timeout_s / args.dt_s)) + 1
            for _ in range(max_cycles):
                if viewer is not None and not viewer.is_running():
                    viewer_closed_early = True
                    break

                old_phase = task.phase
                result, commands = task.step(snapshot)
                new_phase = task.phase

                if new_phase.value != phase_history[-1]:
                    phase_history.append(new_phase.value)

                step_started = time.monotonic()
                snapshot = session.cycle(tuple(commands))

                max_object_z = max(
                    max_object_z,
                    float(
                        snapshot.body_poses["object"].position_xyz_m[2]
                    ),
                )

                if viewer is not None:
                    viewer.sync()
                    _sleep_to_realtime(
                        started_at=step_started,
                        dt_s=args.dt_s,
                        realtime_scale=args.realtime_scale,
                    )

                if (
                    old_phase is TaskPhase.LIFT
                    and new_phase is TaskPhase.HOLD
                ):
                    hold_started_s = float(snapshot.time_s)

                if new_phase in (TaskPhase.SUCCESS, TaskPhase.FAILURE):
                    terminal_result = result
                    break

            if viewer is not None:
                viewer.sync()
                if args.post_roll_s > 0.0 and viewer.is_running():
                    time.sleep(args.post_roll_s)

        final_object_z = float(
            snapshot.body_poses["object"].position_xyz_m[2]
        )
        final_table_normal = float(
            snapshot.signals["object_table_normal_n"]
        )
        final_squeeze = float(
            snapshot.signals["opposing_y_squeeze_n"]
        )
        hold_duration_s = (
            0.0
            if hold_started_s is None
            else max(0.0, float(snapshot.time_s) - hold_started_s)
        )

        success = bool(
            not viewer_closed_early
            and task.phase is TaskPhase.SUCCESS
            and terminal_result is not None
            and terminal_result.status is SkillStatus.SUCCESS
        )

        return {
            "status": "PASS" if success else "BLOCKED",
            "task_phase": task.phase.value,
            "task_result_status": (
                None
                if terminal_result is None
                else terminal_result.status.value
            ),
            "task_failure_reason": (
                "TIMEOUT_OR_VIEWER_CLOSED"
                if terminal_result is None
                else terminal_result.reason.value
            ),
            "phase_history": phase_history,
            "viewer": bool(args.viewer),
            "viewer_closed_early": viewer_closed_early,
            "target_lift_m": task_cfg.control.lift.delta_world_z_m,
            "minimum_object_rise_m": (
                task_cfg.control.lift.minimum_object_rise_m
            ),
            "final_object_rise_m": final_object_z - initial_object_z,
            "max_object_rise_m": max_object_z - initial_object_z,
            "hold_duration_s": hold_duration_s,
            "required_hold_duration_s": task_cfg.control.hold.duration_s,
            "final_object_table_normal_n": final_table_normal,
            "final_opposing_y_squeeze_n": final_squeeze,
            "grasp_squeeze_quality_met": grasp.squeeze_quality_met,
            "lift_duration_s": (
                None
                if lift.timing_result is None
                else lift.timing_result.duration_s
            ),
            "final_time_s": float(snapshot.time_s),
        }
    finally:
        session.shutdown()


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run the WAM7 + Linker Hand L20 tabletop grasp/lift demo "
            "with the MuJoCo backend."
        )
    )
    parser.add_argument(
        "--backend-config",
        type=Path,
        default=ROOT / "configs/backends/mujoco/wam7_linker_l20.yaml",
    )
    parser.add_argument(
        "--task-config",
        type=Path,
        default=ROOT / "configs/tasks/tabletop_grasp_lift.yaml",
    )
    parser.add_argument(
        "--robot-config",
        type=Path,
        default=ROOT / "configs/robots/wam7_linker_l20.yaml",
    )
    parser.add_argument(
        "--joint-limits",
        type=Path,
        default=ROOT / "configs/devices/arms/wam7_kinematic_limits.yaml",
    )
    parser.add_argument(
        "--cartesian-limits",
        type=Path,
        default=ROOT / "configs/motion/cartesian_limits.yaml",
    )
    parser.add_argument(
        "--motion-profiles",
        type=Path,
        default=ROOT / "configs/motion/profiles.yaml",
    )
    parser.add_argument(
        "--asset-registry",
        type=Path,
        default=ROOT / "configs/assets/registry.yaml",
    )
    parser.add_argument(
        "--asset-selection",
        type=Path,
        default=ROOT / "configs/assets/wam7_linker_l20_mujoco.yaml",
    )
    parser.add_argument(
        "--asset-root-config",
        type=Path,
        default=ROOT / "configs/assets/robot_assets.example.yaml",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=ROOT / "runs/mujoco_tabletop_grasp_lift_summary.json",
    )
    parser.add_argument("--dt-s", type=float, default=0.01)
    parser.add_argument("--timeout-s", type=float, default=30.0)
    parser.add_argument(
        "--viewer",
        action="store_true",
        help="Show the interactive MuJoCo viewer and pace at real time.",
    )
    parser.add_argument(
        "--realtime-scale",
        type=float,
        default=1.0,
        help="Wall-clock seconds per simulated dt multiplier in viewer mode.",
    )
    parser.add_argument("--pre-roll-s", type=float, default=1.0)
    parser.add_argument("--post-roll-s", type=float, default=2.0)
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)

    try:
        summary = run(args)
    except Exception as exc:
        summary = {
            "status": "ERROR",
            "error": f"{type(exc).__name__}:{exc}",
        }

    args.output.write_text(
        json.dumps(summary, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )

    print(f"STATUS={summary['status']}")
    print(f"OUTPUT={args.output}")
    if summary["status"] == "ERROR":
        print(f"ERROR={summary['error']}")
        return 1

    print("PHASE_HISTORY=" + "->".join(summary["phase_history"]))
    print(
        "RISE="
        f"target={summary['target_lift_m']:.9f}|"
        f"minimum={summary['minimum_object_rise_m']:.9f}|"
        f"final={summary['final_object_rise_m']:.9f}|"
        f"max={summary['max_object_rise_m']:.9f}"
    )
    print(
        "HOLD="
        f"actual={summary['hold_duration_s']:.9f}|"
        f"required={summary['required_hold_duration_s']:.9f}|"
        f"final_table={summary['final_object_table_normal_n']:.9f}"
    )
    print(
        "GRASP="
        f"final_squeeze={summary['final_opposing_y_squeeze_n']:.9f}|"
        f"quality={summary['grasp_squeeze_quality_met']}"
    )
    print(
        "TERMINAL="
        f"phase={summary['task_phase']}|"
        f"status={summary['task_result_status']}|"
        f"reason={summary['task_failure_reason']}"
    )
    print(f"FINAL_TIME_S={summary['final_time_s']:.9f}")
    return 0 if summary["status"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
