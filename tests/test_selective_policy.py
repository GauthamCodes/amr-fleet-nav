"""The selective-update policy: what earns a merge, what gets deferred, and why.

The five cases that matter are the ones PLAN.md section 6 names - frontier accepted,
revisited deferred, changed re-accepted - plus the two that keep the policy honest:
the bootstrap update, and recency preventing a permanently stale fleet map.
"""

import numpy as np
import pytest

from amr_fleet_control.fleet_grid import UNKNOWN
from amr_fleet_control.selective_policy import (
    bump_visits,
    DEFAULTS,
    resolve_config,
    score_update,
)

SHAPE = (60, 60)


def blank():
    """Return an all-unknown region the size the tests work in."""
    return np.full(SHAPE, UNKNOWN, dtype=np.int8)


def mapped(value=0):
    """Return a fully known region, free by default."""
    return np.full(SHAPE, value, dtype=np.int8)


def visits(count=0):
    """Return a merge-count array with every cell at the same count."""
    return np.full(SHAPE, count, dtype=np.int32)


def test_defaults_are_complete_and_numeric():
    assert set(resolve_config()) == set(DEFAULTS)
    assert all(isinstance(v, float) for v in resolve_config().values())


def test_overrides_are_applied():
    assert resolve_config({"accept_threshold": 0.9})["accept_threshold"] == 0.9


def test_a_misspelt_key_is_rejected_rather_than_ignored():
    """A silently ignored weight is a config that does not mean what it says."""
    with pytest.raises(KeyError, match="unknown selective-policy key"):
        resolve_config({"w_frontiers": 1.0})


@pytest.mark.parametrize("term", ["w_frontier", "w_change", "w_recency"])
def test_revisit_penalty_cannot_veto_a_reason_to_merge(term):
    """The veto invariant, stated directly on the weights.

    The first weights tried in this module failed this for ``w_change``: a new
    obstacle on a corridor driven six times scored 0.20 against a 0.35 threshold and
    was deferred. That is the failure a visit-count-only policy has, reintroduced
    through a weight choice, and it is worth a test rather than a comment.
    """
    config = resolve_config()
    assert config[term] > config["w_revisit"] + config["accept_threshold"]


def test_each_positive_term_alone_clears_the_bar_under_full_penalty():
    """The invariant above, exercised through the scorer rather than the weights."""
    saturated = {
        # A frontier push into wholly unknown territory.
        "frontier": (blank(), mapped(), 0.0),
        # An occupancy flip across an already-known region.
        "change": (mapped(), mapped(100), 0.0),
        # Nothing new, but the deferral has aged out.
        "recency": (mapped(), mapped(), 1e6),
    }
    for name, (previous, candidate, elapsed) in saturated.items():
        decision = score_update(
            previous, candidate, visits(count=999), elapsed_s=elapsed
        )
        assert decision.accepted, f"{name} alone was vetoed by the revisit penalty"


def test_shape_mismatch_is_rejected():
    with pytest.raises(ValueError, match="shape mismatch"):
        score_update(blank(), np.zeros((3, 3), dtype=np.int8), visits(), 1.0)


def test_first_update_is_always_accepted_however_it_scores():
    """Bootstrap: a fleet map missing a robot is worse than an over-eager merge."""
    decision = score_update(
        mapped(), mapped(), visits(count=99), elapsed_s=0.0, first=True
    )
    assert decision.accepted
    assert "bootstrap" in decision.reason
    # And it is genuinely a bootstrap override, not a high score.
    assert decision.score < DEFAULTS["accept_threshold"]


def test_frontier_is_accepted():
    """A robot exploring unknown territory always earns a merge."""
    previous = blank()
    candidate = blank()
    candidate[:40, :40] = 0  # 1600 newly known cells, well past the 400-cell scale

    decision = score_update(previous, candidate, visits(), elapsed_s=0.0)
    assert decision.accepted
    assert decision.frontier == pytest.approx(1.0)
    assert decision.new_cells == 1600
    assert decision.changed_cells == 0


def test_a_revisited_region_with_nothing_new_is_deferred():
    """The saving the policy exists for: re-traversing a corridor nothing changed in."""
    previous = mapped()
    candidate = mapped()

    decision = score_update(previous, candidate, visits(count=6), elapsed_s=0.0)
    assert not decision.accepted
    assert decision.new_cells == 0
    assert decision.changed_cells == 0
    assert decision.revisit == pytest.approx(1.0)
    assert decision.score < 0.0


def test_a_change_in_a_revisited_region_is_re_accepted():
    """Re-accept a genuine change inside a heavily revisited region.

    This is the failure mode a visit-count-only policy has: an obstacle appears on
    a corridor the robot has already driven many times, and the merge is skipped.
    """
    previous = mapped()
    candidate = mapped()
    candidate[:12, :12] = 100  # 144 cells flip free -> occupied

    decision = score_update(previous, candidate, visits(count=6), elapsed_s=0.0)
    assert decision.accepted
    assert decision.changed_cells == 144
    assert decision.change == pytest.approx(1.0)
    assert decision.new_cells == 0


def test_recency_eventually_forces_a_merge():
    """Without this term a robot parked in a mapped aisle defers forever."""
    previous, candidate = mapped(), mapped()

    fresh = score_update(previous, candidate, visits(count=1), elapsed_s=0.0)
    stale = score_update(previous, candidate, visits(count=1), elapsed_s=60.0)

    assert not fresh.accepted
    assert stale.recency == pytest.approx(1.0)
    assert stale.score > fresh.score


def test_recency_saturates_at_the_horizon():
    previous, candidate = mapped(), mapped()
    at_horizon = score_update(previous, candidate, visits(), elapsed_s=20.0)
    far_past = score_update(previous, candidate, visits(), elapsed_s=2000.0)
    assert at_horizon.recency == pytest.approx(1.0)
    assert far_past.recency == pytest.approx(1.0)


def test_every_term_is_normalised_to_the_unit_interval():
    """The weights are only comparable to each other if the terms share a scale."""
    previous = blank()
    candidate = mapped(100)
    decision = score_update(previous, candidate, visits(count=999), elapsed_s=1e6)
    for term in (
        decision.frontier,
        decision.change,
        decision.recency,
        decision.revisit,
    ):
        assert 0.0 <= term <= 1.0


def test_frontier_cells_are_not_double_counted_as_changes():
    """A cell crossing unknown -> occupied is exploration, not an occupancy flip."""
    previous = blank()
    candidate = blank()
    candidate[:10, :10] = 100

    decision = score_update(previous, candidate, visits(), elapsed_s=0.0)
    assert decision.new_cells == 100
    assert decision.changed_cells == 0


def test_a_stationary_robot_republishing_the_same_map_is_deferred():
    """Defer a stationary robot republishing an identical map.

    Nothing is touched at all, so the penalty is charged over the re-observed
    region rather than defaulting to zero, and the idle stream defers on its own
    merits rather than by accident.
    """
    previous, candidate = mapped(), mapped()
    decision = score_update(previous, candidate, visits(count=4), elapsed_s=0.0)
    assert not decision.accepted
    assert decision.revisit > 0.0


def test_revisit_penalty_is_charged_only_where_something_changed():
    """Charge the revisit penalty only where something actually changed.

    A frontier push into virgin cells must not be penalised by a heavily revisited
    region elsewhere in the same map.
    """
    previous = mapped()
    previous[:30, :] = UNKNOWN
    candidate = mapped()

    heavy = visits(count=0)
    heavy[30:, :] = 50  # the already-merged half, untouched by this update

    decision = score_update(previous, candidate, heavy, elapsed_s=0.0)
    assert decision.accepted
    assert decision.revisit == pytest.approx(0.0)


def test_bump_visits_counts_only_changed_cells():
    previous = mapped()
    candidate = mapped()
    candidate[:10, :10] = 100

    counter = visits()
    touched = bump_visits(counter, previous, candidate)

    assert touched == 100
    assert np.all(counter[:10, :10] == 1)
    assert np.all(counter[20:, 20:] == 0)


def test_bump_visits_counts_newly_known_cells():
    previous = blank()
    candidate = blank()
    candidate[:5, :5] = 0

    counter = visits()
    assert bump_visits(counter, previous, candidate) == 25
    assert np.all(counter[:5, :5] == 1)


def test_repeated_merges_drive_a_region_toward_deferral():
    """Drive a repeatedly merged region toward deferral.

    The end-to-end property: the same corridor, merged over and over, gets
    progressively cheaper to skip.
    """
    previous = mapped()
    counter = visits()
    scores = []

    for _ in range(8):
        candidate = mapped()
        candidate[:8, :8] = 100 if len(scores) % 2 == 0 else 0
        decision = score_update(previous, candidate, counter, elapsed_s=0.0)
        scores.append(decision.score)
        if decision.accepted:
            bump_visits(counter, previous, candidate)
            previous = candidate.copy()

    assert scores[-1] < scores[0]


def test_decision_repr_names_the_verdict_and_the_terms():
    decision = score_update(blank(), mapped(), visits(), elapsed_s=0.0)
    text = repr(decision)
    assert "ACCEPT" in text
    assert "f=" in text and "c=" in text and "r=" in text and "v=" in text
