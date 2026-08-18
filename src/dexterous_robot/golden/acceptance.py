from __future__ import annotations

import math
from typing import Any

PASS_CLASSIFICATION = "M1_GOLDEN_WAM7_L20_ISAAC_TABLETOP_GRASP_LIFT_ACCEPTED"
BLOCK_CLASSIFICATION = "M1_GOLDEN_WAM7_L20_ISAAC_TABLETOP_GRASP_LIFT_BLOCKED"
_REQUIRED = (
    "wam_l20_loaded",
    "grasp_lock_success",
    "object_left_table",
    "cuboid_center_z_rise_m",
    "suspended_hold_s",
    "transform_consistency_pass",
)


def _finite_number(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def evaluate_m1_golden(summary: dict[str, Any]) -> dict[str, Any]:
    if not isinstance(summary, dict):
        return {"status": "BLOCKED", "classification": BLOCK_CLASSIFICATION, "errors": ["SUMMARY_INVALID"], "gates": {}}
    errors = [f"MISSING:{field}" for field in _REQUIRED if field not in summary]
    gates: dict[str, bool] = {}
    if not errors:
        gates["wam_l20_loaded"] = summary["wam_l20_loaded"] is True
        gates["grasp_lock_success"] = summary["grasp_lock_success"] is True
        gates["object_left_table"] = summary["object_left_table"] is True
        rise = _finite_number(summary["cuboid_center_z_rise_m"])
        hold = _finite_number(summary["suspended_hold_s"])
        gates["cuboid_center_z_rise_m"] = rise is not None and rise >= 0.025
        gates["suspended_hold_s"] = hold is not None and hold >= 0.5
        gates["transform_consistency_pass"] = summary["transform_consistency_pass"] is True
        errors.extend(f"GATE_FAILED:{name}" for name, passed in gates.items() if not passed)
    passed = not errors
    return {
        "status": "PASS" if passed else "BLOCKED",
        "classification": PASS_CLASSIFICATION if passed else BLOCK_CLASSIFICATION,
        "errors": errors,
        "gates": gates,
    }
