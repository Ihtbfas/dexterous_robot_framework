from __future__ import annotations

import pytest


def test_normalized_contact_summary_uses_impulse_over_dt_and_opposing_y_minimum():
    from dexterous_robot.backends.isaac.contacts import ContactSample, summarize_contacts

    rows = (
        ContactSample("/Object", "/Table", (0.0, 0.0, 1.0), 0.025),
        ContactSample("/Object", "/Hand/thumb", (0.0, 1.0, 0.0), 0.004),
        ContactSample("/Object", "/Hand/ring", (0.0, -1.0, 0.0), 0.006),
    )
    summary = summarize_contacts(rows, dt_s=0.01, object_path="/Object", table_path="/Table")
    assert summary.object_table_normal_n == pytest.approx(2.5)
    assert summary.opposing_y_squeeze_n == pytest.approx(0.4)


def test_contact_summary_is_zero_without_matching_contacts():
    from dexterous_robot.backends.isaac.contacts import summarize_contacts

    summary = summarize_contacts((), dt_s=0.01, object_path="/Object", table_path="/Table")
    assert summary.object_table_normal_n == 0.0
    assert summary.opposing_y_squeeze_n == 0.0
