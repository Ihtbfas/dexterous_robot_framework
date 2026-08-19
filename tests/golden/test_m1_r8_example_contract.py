from __future__ import annotations

from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path

from dexterous_robot.backends.isaac.config import load_tabletop_grasp_lift_config
from dexterous_robot.tasks import TaskPhase


def _load_example_module():
    path = Path(__file__).resolve().parents[2] / "examples" / "isaac" / "tabletop_grasp_lift.py"
    spec = spec_from_file_location("m1_r8_tabletop_example", path)
    if spec is None or spec.loader is None:
        raise AssertionError("M1_R8_EXAMPLE_IMPORT_SPEC_INVALID")
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    return path, module


def test_golden_example_is_public_framework_code_without_legacy_imports() -> None:
    path, _ = _load_example_module()
    source = path.read_text(encoding="utf-8")
    assert "scripts.phase2" not in source
    assert "p2b2_" not in source.lower()
    assert "R15U" not in source


def test_golden_example_builds_task_from_typed_config() -> None:
    root = Path(__file__).resolve().parents[2]
    _, module = _load_example_module()
    task_cfg = load_tabletop_grasp_lift_config(root / "configs" / "tasks" / "tabletop_grasp_lift.yaml")
    task, final_hand_hold = module._build_task(task_cfg)
    assert task.phase is TaskPhase.APPROACH
    assert final_hand_hold.device_id == "hand"
    assert final_hand_hold.profile == "hand_grasp_lock"
    assert len(final_hand_hold.position_rad) == 21
    # Frozen M1 grasp-lock trims: thumb 2/3 +0.04; ring/little side leaders +0.08.
    assert final_hand_hold.position_rad[2] == task_cfg.control.grasp.base_preload_hand_q_rad[2] + 0.04
    assert final_hand_hold.position_rad[3] == task_cfg.control.grasp.base_preload_hand_q_rad[3] + 0.04
    assert final_hand_hold.position_rad[14] == task_cfg.control.grasp.base_preload_hand_q_rad[14] + 0.08
    assert final_hand_hold.position_rad[18] == task_cfg.control.grasp.base_preload_hand_q_rad[18] + 0.08


def test_golden_example_resolves_assets_through_registry_not_legacy_direct_paths() -> None:
    path, _ = _load_example_module()
    source = path.read_text(encoding="utf-8")
    assert "load_asset_registry" in source
    assert "load_asset_selection" in source
    assert "load_local_asset_config" not in source
    assert "--asset-registry" in source
    assert "--asset-root-config" in source
