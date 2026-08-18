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


def test_m1_isaac_runtime_loads_combined_manipulator_and_cycles_ten_times():
    receipt = _receipt()
    assert receipt["status"] == "PASS"
    assert receipt["initialized"] is True
    assert receipt["cycles_completed"] == 10
    assert receipt["initial_state"]["arm_width"] == 7
    assert receipt["initial_state"]["hand_width"] == 21
    assert receipt["final_state"]["object_finite"] is True
    topology = receipt["backend_diagnostics"]["combined_articulation"]
    assert topology["count"] == 1
    assert topology["max_dofs"] == 28
    assert len(topology["backend_joint_names"]) == 28
