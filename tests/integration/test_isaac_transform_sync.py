from __future__ import annotations

import json
import os
from pathlib import Path

import pytest


def _receipt():
    path = os.environ.get("DEXTEROUS_ROBOT_ISAAC_R5_RECEIPT")
    if not path:
        pytest.skip("DEXTEROUS_ROBOT_ISAAC_R5_RECEIPT not set")
    return json.loads(Path(path).read_text(encoding="utf-8"))


def test_m1_dynamic_object_transform_checkpoint_agrees_across_tensor_physx_usd_and_fabric():
    receipt = _receipt()
    release = receipt["backend_diagnostics"]["transform_release"]
    assert release["released_properties"] == ["xformOp:translate", "xformOp:orient"]
    assert release["position_error_m"] <= 0.001
    checkpoints = receipt["backend_diagnostics"]["transform_checkpoints"]
    assert checkpoints
    row = checkpoints[-1]
    assert row["label"] == "POST_SMOKE_10_STEPS"
    assert row["consistent"] is True
    assert row["max_pairwise_position_error_m"] <= 0.001
    assert set(row["positions"]) == {"tensor", "physx", "usd", "fabric"}
    assert all(row["positions"][source] is not None for source in row["positions"])
