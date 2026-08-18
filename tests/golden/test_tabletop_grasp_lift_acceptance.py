from __future__ import annotations

import json
from pathlib import Path

import pytest

from dexterous_robot.golden import (
    BLOCK_CLASSIFICATION,
    PASS_CLASSIFICATION,
    evaluate_m1_golden,
)


def _passing_summary() -> dict[str, object]:
    return {
        "wam_l20_loaded": True,
        "grasp_lock_success": True,
        "object_left_table": True,
        "cuboid_center_z_rise_m": 0.034,
        "suspended_hold_s": 1.0,
        "transform_consistency_pass": True,
    }


def test_finalizer_accepts_only_when_all_six_hard_gates_pass() -> None:
    result = evaluate_m1_golden(_passing_summary())
    assert result["status"] == "PASS"
    assert result["classification"] == PASS_CLASSIFICATION
    assert result["errors"] == []


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("wam_l20_loaded", False),
        ("grasp_lock_success", False),
        ("object_left_table", False),
        ("cuboid_center_z_rise_m", 0.024999),
        ("suspended_hold_s", 0.499999),
        ("transform_consistency_pass", False),
    ],
)
def test_finalizer_blocks_each_failed_hard_gate(field: str, value: object) -> None:
    summary = _passing_summary()
    summary[field] = value
    result = evaluate_m1_golden(summary)
    assert result["status"] == "BLOCKED"
    assert result["classification"] == BLOCK_CLASSIFICATION
    assert result["errors"]


def test_finalizer_blocks_missing_field_instead_of_defaulting() -> None:
    summary = _passing_summary()
    del summary["suspended_hold_s"]
    result = evaluate_m1_golden(summary)
    assert result["status"] == "BLOCKED"
    assert any("MISSING:suspended_hold_s" in error for error in result["errors"])


def test_finalizer_cli_writes_machine_readable_result(tmp_path: Path) -> None:
    from importlib.util import module_from_spec, spec_from_file_location

    path = Path(__file__).resolve().parents[2] / "tools" / "review" / "finalize_m1_golden.py"
    spec = spec_from_file_location("finalize_m1_golden_cli", path)
    assert spec is not None and spec.loader is not None
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    main = module.main

    summary_path = tmp_path / "summary.json"
    output_path = tmp_path / "final.json"
    summary_path.write_text(json.dumps(_passing_summary()), encoding="utf-8")
    assert main(["--summary", str(summary_path), "--output", str(output_path)]) == 0
    result = json.loads(output_path.read_text(encoding="utf-8"))
    assert result["status"] == "PASS"
