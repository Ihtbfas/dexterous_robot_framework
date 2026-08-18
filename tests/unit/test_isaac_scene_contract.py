from __future__ import annotations


def test_articulation_candidate_patterns_prioritize_exact_body_before_fallbacks():
    from dexterous_robot.backends.isaac.scene import build_articulation_candidate_patterns

    patterns = build_articulation_candidate_patterns(
        "/wam_7dof",
        ("/wam_7dof/Geometry/world",),
        ("/wam_7dof/Geometry/world/wam_footprint/wam_base",),
    )
    assert patterns[0] == "/wam_7dof/Geometry/world/wam_footprint/wam_base"
    assert patterns[1] == "/wam_7dof/Geometry/world"
    assert patterns[-1] == "/wam_7dof*"
    assert len(patterns) == len(set(patterns))
