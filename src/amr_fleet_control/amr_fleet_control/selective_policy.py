"""Scored selective map-update policy, as pure functions.

No ROS imports, for the same reason ``amr_bsp.validators`` has none: the decision rule
is the part worth testing, and it should be testable without a graph.

WHAT THE ASSIGNMENT ASKS FOR, AND WHAT THIS DOES

    The requirement is selective MAP UPDATES - "publishing to the merged map" - not a
    rewrite of slam_toolbox's internals. Each robot keeps mapping continuously; what
    this policy decides is whether a given candidate is worth merging into the fleet
    map and republishing to both global costmaps.

    A visit-count threshold alone is too blunt: it defers exactly the updates a robot
    makes while re-traversing a corridor, which is also when an obstacle is most
    likely to have appeared there. So the score has four terms, and the two that
    argue FOR merging are about new information:

        score = w_f*frontier + w_c*occupancy_change + w_r*recency - w_v*revisit

    frontier          cells crossing unknown -> known. Exploration is the one thing
                      that always earns a merge.
    occupancy_change  cells flipping free <-> occupied among cells BOTH grids already
                      knew. This is the term that lets a re-traversal through a
                      heavily revisited corridor still publish when something moved.
    recency           time since this robot last had an update accepted, saturating.
                      Without it a robot parked in a well-mapped aisle would defer
                      forever and the fleet map would silently go stale.
    revisit           mean merge count over the cells that changed. This is the
                      penalty that makes repeatedly-traversed cells cheap to skip.

    Each term is normalised to [0, 1] by a scale in the config, so the weights are
    comparable to each other and the threshold means something.

THE VETO INVARIANT, AND WHY THE WEIGHTS ARE WHAT THEY ARE

    The revisit penalty must never be able to veto a term that argues for merging.
    The first weights tried here failed that: ``w_change 0.8`` against
    ``w_revisit 0.6`` meant a NEW OBSTACLE appearing on a corridor the robot had
    already driven six times scored 0.20 against a 0.35 threshold and was deferred -
    the exact failure a visit-count-only policy has, reintroduced through the back
    door of a weight choice. It was found by a test, not by reading the formula.

    So each of the three positive weights must exceed ``w_revisit + threshold``:

        w_frontier, w_change, w_recency  >  w_revisit + accept_threshold

    which guarantees that a fully saturated frontier push, a fully saturated
    occupancy change, or a fully aged deferral each clears the bar on its own even
    under the maximum revisit penalty. The penalty then does what it is for - it
    discriminates between updates that are otherwise close - and cannot suppress one
    outright.

BOOTSTRAP

    A robot's FIRST update is always accepted, whatever it scores. Until one has been
    accepted there is no previous grid to diff against, so every term except recency
    is measuring against all-unknown; and a fleet map missing a robot entirely is a
    worse failure than an over-eager merge.
"""

import numpy as np

from amr_fleet_control.fleet_grid import OCCUPIED_THRESHOLD

#: Config keys with their defaults. Every one is overridable from fleet_map.yaml;
#: they live here so the policy has a complete, documented default and the node does
#: not have to restate them.
DEFAULTS = {
    # Weights. Chosen to satisfy the veto invariant above, not by taste:
    #   1.2 - 0.6 = 0.6 >= 0.35   frontier alone, fully penalised
    #   1.2 - 0.6 = 0.6 >= 0.35   change alone, fully penalised
    #   1.0 - 0.6 = 0.4 >= 0.35   recency alone, fully penalised
    # tests/test_selective_policy.py asserts all three, so a later retune that breaks
    # one fails a test instead of quietly dropping obstacle updates.
    "w_frontier": 1.2,
    "w_change": 1.2,
    "w_recency": 1.0,
    "w_revisit": 0.6,
    # Normalisation scales, in cells. At 0.05 m, 400 cells is 1 m^2.
    "frontier_scale_cells": 400.0,
    "change_scale_cells": 120.0,
    # Seconds after which the recency term saturates at 1.0.
    "recency_horizon_s": 20.0,
    # Merge count at which the revisit penalty saturates.
    "revisit_scale": 6.0,
    # Accept when the score reaches this.
    "accept_threshold": 0.35,
}


class UpdateDecision:
    """One selective-update decision, with the terms that produced it.

    Carries the component scores as well as the verdict so that the evidence log can
    show WHY an update was deferred. A log that only records accept/defer cannot
    distinguish a working policy from a stuck threshold.
    """

    def __init__(
        self,
        accepted,
        score,
        frontier,
        change,
        recency,
        revisit,
        new_cells,
        changed_cells,
        reason,
    ):
        """Store the verdict, the total score, and every term behind it."""
        self.accepted = bool(accepted)
        self.score = float(score)
        self.frontier = float(frontier)
        self.change = float(change)
        self.recency = float(recency)
        self.revisit = float(revisit)
        self.new_cells = int(new_cells)
        self.changed_cells = int(changed_cells)
        self.reason = reason

    def __repr__(self):
        """Return a debuggable one-line summary."""
        verdict = "ACCEPT" if self.accepted else "DEFER"
        return (
            f"{verdict} score={self.score:+.3f} "
            f"(f={self.frontier:.2f} c={self.change:.2f} "
            f"r={self.recency:.2f} v={self.revisit:.2f}) {self.reason}"
        )


def resolve_config(overrides=None):
    """Return DEFAULTS merged with overrides, rejecting unknown keys.

    An unknown key is a typo in fleet_map.yaml, and a policy that silently ignores a
    misspelt weight is a policy whose configuration does not mean what it says.
    """
    config = dict(DEFAULTS)
    for key, value in (overrides or {}).items():
        if key not in DEFAULTS:
            raise KeyError(
                f"unknown selective-policy key '{key}'; known keys: {sorted(DEFAULTS)}"
            )
        config[key] = float(value)
    return config


def _saturate(value, scale):
    """Return ``value / scale`` clipped to [0, 1], treating a zero scale as 0."""
    if scale <= 0.0:
        return 0.0
    return float(min(1.0, max(0.0, value / scale)))


def score_update(previous, candidate, visits, elapsed_s, config=None, first=False):
    """Score one candidate map update against what has already been merged.

    All three arrays describe the SAME region and must have the same shape. The node
    achieves that by slicing its fleet-sized accumulators to the candidate's footprint
    rather than by resampling, so no interpolation happens anywhere in this path.

    Args:
        previous: ``int8`` occupancy of the last accepted state for this robot.
        candidate: ``int8`` occupancy of the incoming map.
        visits: ``int`` array of how many times each cell has already been merged.
        elapsed_s: Seconds since this robot's last accepted update.
        config: Optional overrides for :data:`DEFAULTS`.
        first: True for a robot's first update, which is always accepted.

    Returns:
        An :class:`UpdateDecision`.
    """
    if previous.shape != candidate.shape or visits.shape != candidate.shape:
        raise ValueError(
            f"shape mismatch: previous {previous.shape}, candidate "
            f"{candidate.shape}, visits {visits.shape}"
        )
    config = resolve_config(config)

    candidate_known = candidate >= 0
    previous_known = previous >= 0

    # Exploration: cells this update turns from unknown into known.
    newly_known = candidate_known & ~previous_known
    new_cells = int(np.count_nonzero(newly_known))

    # Change: cells BOTH grids know, whose classification flipped. Restricting to
    # mutually known cells is what stops the frontier from being counted twice.
    both_known = candidate_known & previous_known
    flipped = both_known & (
        (candidate >= OCCUPIED_THRESHOLD) != (previous >= OCCUPIED_THRESHOLD)
    )
    changed_cells = int(np.count_nonzero(flipped))

    frontier = _saturate(new_cells, config["frontier_scale_cells"])
    change = _saturate(changed_cells, config["change_scale_cells"])
    recency = _saturate(elapsed_s, config["recency_horizon_s"])

    # The revisit penalty is charged only where this update actually claims to add
    # something. Averaging over the whole grid would make it a constant, and a
    # constant term cannot discriminate between updates.
    touched = newly_known | flipped
    if np.any(touched):
        mean_visits = float(np.mean(visits[touched]))
    else:
        # Nothing new at all. Charge the full penalty over the region the robot is
        # re-observing, so a stationary robot's stream of identical maps is deferred
        # on its own merits rather than by accident.
        mean_visits = float(np.mean(visits[both_known])) if np.any(both_known) else 0.0
    revisit = _saturate(mean_visits, config["revisit_scale"])

    score = (
        config["w_frontier"] * frontier
        + config["w_change"] * change
        + config["w_recency"] * recency
        - config["w_revisit"] * revisit
    )

    if first:
        accepted, reason = True, "bootstrap: first update from this robot"
    elif score >= config["accept_threshold"]:
        accepted, reason = True, f"score >= {config['accept_threshold']:.2f}"
    else:
        accepted, reason = False, f"score < {config['accept_threshold']:.2f}"

    return UpdateDecision(
        accepted=accepted,
        score=score,
        frontier=frontier,
        change=change,
        recency=recency,
        revisit=revisit,
        new_cells=new_cells,
        changed_cells=changed_cells,
        reason=reason,
    )


def bump_visits(visits, previous, candidate):
    """Increment the merge counter on the cells an accepted update actually changed.

    Called only after an update is accepted. Counting every known cell instead would
    make the revisit penalty grow uniformly and stop discriminating.

    Returns:
        The number of cells incremented.
    """
    candidate_known = candidate >= 0
    previous_known = previous >= 0
    touched = (candidate_known & ~previous_known) | (
        candidate_known
        & previous_known
        & ((candidate >= OCCUPIED_THRESHOLD) != (previous >= OCCUPIED_THRESHOLD))
    )
    visits[touched] += 1
    return int(np.count_nonzero(touched))
