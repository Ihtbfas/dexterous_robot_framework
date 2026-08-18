from __future__ import annotations

from math import isfinite
from typing import Mapping


def _mapping(value) -> Mapping[str, object]:
    return value if isinstance(value, Mapping) else {}


def validate_m1_r5_smoke_receipt(receipt: Mapping[str, object], *, tolerance_m: float) -> dict[str, object]:
    """Validate the observable M1-R5 runtime contract without importing Isaac."""

    tolerance = float(tolerance_m)
    errors: list[str] = []
    if not isfinite(tolerance) or tolerance <= 0.0:
        raise ValueError("ISAAC_SMOKE_RECEIPT_TOLERANCE_INVALID")
    if receipt.get("status") != "PASS":
        errors.append("STATUS_NOT_PASS")
    if receipt.get("classification") != "M1_R5_ISAAC_RUNTIME_SMOKE_PASS":
        errors.append("CLASSIFICATION_INVALID")
    if receipt.get("initialized") is not True:
        errors.append("INITIALIZED_FALSE")
    if receipt.get("cycles_completed") != 10:
        errors.append("CYCLE_COUNT_INVALID")

    for label in ("initial_state", "final_state"):
        state = _mapping(receipt.get(label))
        if state.get("arm_width") != 7:
            errors.append(f"ARM_WIDTH_INVALID:{label}")
        if state.get("hand_width") != 21:
            errors.append(f"HAND_WIDTH_INVALID:{label}")
        if state.get("object_finite") is not True:
            errors.append(f"OBJECT_NONFINITE:{label}")

    diagnostics = _mapping(receipt.get("backend_diagnostics"))
    combined = _mapping(diagnostics.get("combined_articulation"))
    names = combined.get("backend_joint_names")
    if combined.get("count") != 1 or combined.get("max_dofs") != 28 or not isinstance(names, list) or len(names) != 28:
        errors.append("COMBINED_ARTICULATION_INVALID")

    release = _mapping(diagnostics.get("transform_release"))
    try:
        release_error = float(release.get("position_error_m"))
    except (TypeError, ValueError):
        release_error = float("inf")
    if not isfinite(release_error) or release_error > tolerance:
        errors.append("TRANSFORM_RELEASE_INCONSISTENT")
    released = release.get("released_properties")
    if released != ["xformOp:translate", "xformOp:orient"]:
        errors.append("TRANSFORM_RELEASE_PROPERTIES_INVALID")

    checkpoints = diagnostics.get("transform_checkpoints")
    if not isinstance(checkpoints, list) or not checkpoints:
        errors.append("TRANSFORM_CHECKPOINT_MISSING")
    else:
        row = _mapping(checkpoints[-1])
        label = str(row.get("label") or "MISSING")
        if label != "POST_SMOKE_10_STEPS":
            errors.append("TRANSFORM_FINAL_CHECKPOINT_LABEL_INVALID")
        if row.get("consistent") is not True:
            errors.append(f"TRANSFORM_INCONSISTENT:{label}")
        try:
            max_error = float(row.get("max_pairwise_position_error_m"))
        except (TypeError, ValueError):
            max_error = float("inf")
        if not isfinite(max_error) or max_error > tolerance:
            errors.append(f"TRANSFORM_ERROR_EXCEEDED:{label}")
        positions = _mapping(row.get("positions"))
        for source in ("tensor", "physx", "usd", "fabric"):
            if positions.get(source) is None:
                errors.append(f"TRANSFORM_{source.upper()}_MISSING:{label}")

    before = _mapping(receipt.get("asset_hashes_before"))
    after = _mapping(receipt.get("asset_hashes_after"))
    for label in ("wam_runtime", "l20_runtime"):
        if before.get(label) != after.get(label):
            errors.append(f"ASSET_HASH_CHANGED:{label}")

    return {"valid": not errors, "errors": tuple(errors)}
