from __future__ import annotations

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

from dexterous_robot.config.tasks import load_tabletop_grasp_lift_config
from dexterous_robot.devices.arms.wam7 import WAM7_JOINT_NAMES
from dexterous_robot.motion import (
    load_cartesian_kinematic_limits,
    load_joint_kinematic_limits,
    load_motion_profiles,
    resolve_cartesian_limits,
    resolve_joint_limits,
)
from dexterous_robot.tasks import TaskPhase


def _load_example_module():
    path = Path(__file__).resolve().parents[2] / "examples" / "isaac" / "tabletop_grasp_lift.py"
    spec = spec_from_file_location("m1_r8_tabletop_example", path)
    if spec is None or spec.loader is None:
        raise AssertionError("M1_R8_EXAMPLE_IMPORT_SPEC_INVALID")
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return path, module


def _load_current_motion(root: Path):
    profiles = load_motion_profiles(root / "configs/motion/profiles.yaml")
    task_cfg = load_tabletop_grasp_lift_config(
        root / "configs/tasks/tabletop_grasp_lift.yaml",
        motion_profiles=profiles,
    )
    joint_limits = resolve_joint_limits(
        load_joint_kinematic_limits(
            root / "configs/devices/arms/wam7_kinematic_limits.yaml",
            expected_joint_names=WAM7_JOINT_NAMES,
        ),
        profiles.joint(task_cfg.control.approach.motion_profile),
    )
    cartesian_limits = resolve_cartesian_limits(
        load_cartesian_kinematic_limits(root / "configs/motion/cartesian_limits.yaml"),
        profiles.cartesian(task_cfg.control.lift.motion_profile),
    )
    return task_cfg, joint_limits, cartesian_limits


def test_golden_example_is_public_framework_code_without_legacy_imports() -> None:
    path, _ = _load_example_module()
    source = path.read_text(encoding="utf-8")
    assert "scripts.phase2" not in source
    assert "p2b2_" not in source.lower()
    assert "R15U" not in source
    assert "ruckig" not in source.lower()
    assert "moveit" not in source.lower()


def test_golden_example_builds_task_from_typed_config_and_resolved_motion() -> None:
    root = Path(__file__).resolve().parents[2]
    _, module = _load_example_module()
    task_cfg, joint_limits, cartesian_limits = _load_current_motion(root)
    task, final_hand_hold, approach, lift = module._build_task(
        task_cfg,
        approach_joint_limits=joint_limits,
        carry_cartesian_limits=cartesian_limits,
    )
    assert task.phase is TaskPhase.APPROACH
    assert approach.segment_timings == ()
    assert lift.timing_result is None
    assert final_hand_hold.device_id == "hand"
    assert final_hand_hold.profile == "hand_grasp_lock"
    assert len(final_hand_hold.position_rad) == 21
    assert final_hand_hold.position_rad[2] == task_cfg.control.grasp.base_preload_hand_q_rad[2] + 0.04
    assert final_hand_hold.position_rad[3] == task_cfg.control.grasp.base_preload_hand_q_rad[3] + 0.04
    assert final_hand_hold.position_rad[14] == task_cfg.control.grasp.base_preload_hand_q_rad[14] + 0.08
    assert final_hand_hold.position_rad[18] == task_cfg.control.grasp.base_preload_hand_q_rad[18] + 0.08


def test_golden_example_resolves_assets_and_motion_authorities_explicitly() -> None:
    path, _ = _load_example_module()
    source = path.read_text(encoding="utf-8")
    assert "load_asset_registry" in source
    assert "load_asset_selection" in source
    assert "load_local_asset_config" not in source
    assert "--asset-registry" in source
    assert "--asset-root-config" in source
    assert "--joint-limits" in source
    assert "--cartesian-limits" in source
    assert "--motion-profiles" in source
    assert "JointRateAudit" in source
    assert "load_tabletop_grasp_lift_config(args.task_config, motion_profiles=profiles)" in source
