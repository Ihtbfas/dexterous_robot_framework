from __future__ import annotations

import pytest


def test_transform_audit_classifies_position_consistency_at_one_mm_boundary():
    from dexterous_robot.backends.isaac.transform_sync import compare_position_sources

    good = compare_position_sources(
        tensor_xyz=(0.68, -0.14, 1.02),
        physx_xyz=(0.6802, -0.14, 1.0201),
        usd_xyz=(0.6798, -0.14, 1.0201),
        fabric_xyz=(0.6801, -0.1401, 1.0199),
        tolerance_m=0.001,
    )
    assert good.consistent is True
    assert good.max_pairwise_position_error_m < 0.001

    bad = compare_position_sources(
        tensor_xyz=(0.68, -0.14, 1.02),
        physx_xyz=(0.68, -0.14, 1.02),
        usd_xyz=(0.68, -0.14, 1.0185),
        fabric_xyz=None,
        tolerance_m=0.001,
    )
    assert bad.consistent is False
    assert bad.max_pairwise_position_error_m == pytest.approx(0.0015)


def test_session_release_contract_only_removes_dynamic_pose_opinions():
    from dexterous_robot.backends.isaac.transform_sync import SESSION_DYNAMIC_PROPERTIES_TO_RELEASE, SESSION_PROPERTIES_TO_PRESERVE

    assert SESSION_DYNAMIC_PROPERTIES_TO_RELEASE == ("xformOp:translate", "xformOp:orient")
    assert SESSION_PROPERTIES_TO_PRESERVE == ("xformOp:scale", "xformOpOrder")
