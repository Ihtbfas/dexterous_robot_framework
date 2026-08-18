from __future__ import annotations


def _good_receipt():
    return {
        "status": "PASS",
        "classification": "M1_R5_ISAAC_RUNTIME_SMOKE_PASS",
        "initialized": True,
        "cycles_completed": 10,
        "initial_state": {"arm_width": 7, "hand_width": 21, "object_finite": True},
        "final_state": {"arm_width": 7, "hand_width": 21, "object_finite": True},
        "backend_diagnostics": {
            "combined_articulation": {"count": 1, "max_dofs": 28, "backend_joint_names": [f"j{i}" for i in range(28)]},
            "transform_release": {"position_error_m": 0.0, "released_properties": ["xformOp:translate", "xformOp:orient"]},
            "transform_checkpoints": [
                {
                    "label": "POST_SMOKE_10_STEPS",
                    "simulation_time_s": 10.0 / 120.0,
                    "consistent": True,
                    "max_pairwise_position_error_m": 0.0002,
                    "positions": {
                        "tensor": [0.0, 0.0, 1.0],
                        "physx": [0.0, 0.0, 1.0],
                        "usd": [0.0, 0.0, 1.0],
                        "fabric": [0.0, 0.0, 1.0],
                    },
                }
            ],
        },
        "asset_hashes_before": {"wam_runtime": "a" * 64, "l20_runtime": "b" * 64},
        "asset_hashes_after": {"wam_runtime": "a" * 64, "l20_runtime": "b" * 64},
    }


def test_smoke_receipt_validator_accepts_complete_pass_receipt():
    from dexterous_robot.backends.isaac.verification import validate_m1_r5_smoke_receipt

    result = validate_m1_r5_smoke_receipt(_good_receipt(), tolerance_m=0.001)
    assert result["valid"] is True
    assert result["errors"] == ()


def test_smoke_receipt_validator_rejects_missing_fabric_or_changed_asset():
    from dexterous_robot.backends.isaac.verification import validate_m1_r5_smoke_receipt

    receipt = _good_receipt()
    receipt["backend_diagnostics"]["transform_checkpoints"][-1]["positions"]["fabric"] = None
    receipt["asset_hashes_after"]["wam_runtime"] = "c" * 64
    result = validate_m1_r5_smoke_receipt(receipt, tolerance_m=0.001)
    assert result["valid"] is False
    assert "TRANSFORM_FABRIC_MISSING:POST_SMOKE_10_STEPS" in result["errors"]
    assert "ASSET_HASH_CHANGED:wam_runtime" in result["errors"]


def test_smoke_receipt_validator_rejects_wrong_checkpoint_label_or_desync():
    from dexterous_robot.backends.isaac.verification import validate_m1_r5_smoke_receipt

    receipt = _good_receipt()
    checkpoint = receipt["backend_diagnostics"]["transform_checkpoints"][-1]
    checkpoint["label"] = "UNEXPECTED"
    checkpoint["consistent"] = False
    checkpoint["max_pairwise_position_error_m"] = 0.002
    result = validate_m1_r5_smoke_receipt(receipt, tolerance_m=0.001)
    assert result["valid"] is False
    assert "TRANSFORM_FINAL_CHECKPOINT_LABEL_INVALID" in result["errors"]
    assert "TRANSFORM_INCONSISTENT:UNEXPECTED" in result["errors"]
    assert "TRANSFORM_ERROR_EXCEEDED:UNEXPECTED" in result["errors"]
