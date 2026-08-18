from __future__ import annotations

import pytest

from dexterous_robot.control.math.minimum_jerk import minimum_jerk_fraction, minimum_jerk_position


def test_minimum_jerk_fraction_matches_legacy_polynomial_and_clamps_endpoints() -> None:
    assert minimum_jerk_fraction(-1.0) == 0.0
    assert minimum_jerk_fraction(0.0) == 0.0
    assert minimum_jerk_fraction(0.25) == pytest.approx(0.103515625)
    assert minimum_jerk_fraction(0.5) == pytest.approx(0.5)
    assert minimum_jerk_fraction(0.75) == pytest.approx(0.896484375)
    assert minimum_jerk_fraction(1.0) == 1.0
    assert minimum_jerk_fraction(2.0) == 1.0


def test_minimum_jerk_position_clamps_elapsed_time() -> None:
    assert minimum_jerk_position(2.0, 6.0, -0.1, 1.0) == pytest.approx(2.0)
    assert minimum_jerk_position(2.0, 6.0, 0.5, 1.0) == pytest.approx(4.0)
    assert minimum_jerk_position(2.0, 6.0, 1.5, 1.0) == pytest.approx(6.0)


def test_minimum_jerk_rejects_nonfinite_or_nonpositive_duration() -> None:
    with pytest.raises(ValueError, match="MINIMUM_JERK_DURATION_INVALID"):
        minimum_jerk_position(0.0, 1.0, 0.0, 0.0)
    with pytest.raises(ValueError, match="MINIMUM_JERK_INPUT_INVALID"):
        minimum_jerk_fraction(float("nan"))
