"""The records TrafficControlNode keeps, and the evidence it writes from them.

Split out of the node for the same reason the policy is: the node is what runs
during a mission, and a hundred lines of report formatting in the middle of an
arbiter makes the arbitration harder to read than it should be.

WHAT THE REPORT HAS TO SHOW, AND WHY EACH PART IS THERE

    Section A is the escalation ordering (ENGINEERING_NOTES rule 7). A yield count alone
    would say nothing about whether the local layer was given first refusal, so
    every conflict the arbiter DECLINED to act on is counted beside every one it
    took, with the age and the separation gain that decided it.

    Section B is the yield itself: when, how long, and on which of the two
    release conditions it ended.

    Section C is rule 2 - the recovery behaviours that did not fire, with the
    progress-checker allowance read back from controller_server at both ends of
    the hold rather than assumed from the write.

    Section D disambiguates the yield from a safety halt. Both stop a robot, and
    a report that could not separate them would let SafetyGate's work be
    presented as the arbiter's.
"""

from dataclasses import dataclass
import math


@dataclass
class ConflictRecord:
    """One predicted conflict, and how it stopped being one."""

    pair: tuple
    first_seen: float
    decided_at: float
    age_s: float
    outcome: str
    minimum_m: float
    gain_m: float
    yielder: str


@dataclass
class YieldRecord:
    """One yield: everything measured between escalation and release."""

    robot: str
    peer: str
    entered: float
    separation_entry: float
    age_at_escalation: float
    released: float = None
    held_s: float = float("nan")
    reason: str = ""
    separation_min: float = float("inf")
    separation_exit: float = float("nan")
    commands: int = 0
    gate_blocked_ticks: int = 0
    recoveries_at_entry: int = 0
    recoveries_during: int = 0
    allowance_entry: float = None
    allowance_exit: float = None
    last_asserted: float = 0.0


def _allowance(value):
    """Format a read-back allowance, distinguishing 'not read' from a value."""
    return "n/a" if value is None else f"{value:.0f} s"


def report(node):
    """Return the full text report for a finished traffic-control run."""
    lines = []
    add = lines.append
    rule = "=" * 78
    thin = "-" * 78

    add(rule)
    add(node.title)
    add(rule)
    add("PRIORITY, DERIVED FROM fleet.yaml GROSS MASS - no robot is named in code")
    add("")
    add("      rank  robot         gross mass   footprint radius")
    for rank, (name, mass, radius) in enumerate(node.priority_table(), start=1):
        add(f"      {rank:4d}  {name:<12} {mass:8.1f} kg   {radius:12.3f} m")
    add("")
    add(f"    conflict radius:      {node.radius:8.2f} m   (r_a + r_b + d_safe_max)")
    add(f"    time window:          {node.time_window:8.2f} s")
    add(
        f"    escalate after:       {node.escalate_after:8.2f} s of unresolved conflict"
    )
    add(f"    release radius:       {node.radius * node.release_factor:8.2f} m")
    add(f"    max hold (fail-safe): {node.max_hold:8.2f} s")
    add(thin)

    _section_pipeline(node, add, thin)
    _section_yields(node, add, thin)
    _section_recovery(node, add, thin)
    _section_gate(node, add, thin)
    _section_verdict(node, add, rule)
    return "\n".join(lines)


def _section_pipeline(node, add, thin):
    """Section A: how many conflicts the local layer handled on its own."""
    escalated = [c for c in node.conflicts_seen if c.outcome == "escalated"]
    local = [c for c in node.conflicts_seen if c.outcome == "resolved locally"]
    add("[A] CONFLICT PIPELINE  (rule 7: the local layer first, this node only after)")
    add(f"    conflicts predicted:                      {len(node.conflicts_seen):8d}")
    add(f"    resolved WITHOUT escalation:              {len(local):8d}")
    add(f"    escalated to a yield:                     {len(escalated):8d}")
    add("")
    if node.conflicts_seen:
        add("      #  pair               outcome           age    min sep    gain")
        for index, conflict in enumerate(node.conflicts_seen, start=1):
            pair = f"{conflict.pair[0]}/{conflict.pair[1]}"
            minimum = conflict.minimum_m
            add(
                f"    {index:3d}  {pair:<18} {conflict.outcome:<16} "
                f"{conflict.age_s:5.1f}s  {minimum:7.2f}m  {conflict.gain_m:+6.2f}m"
            )
    else:
        add("    no conflict was ever predicted - the two routes never came within")
        add("    the conflict radius of each other inside the prediction horizon")
    add("")
    add("    'gain' is how far the PREDICTED closest approach opened up over the")
    add("    life of the conflict. A conflict that opens by more than")
    add(f"    {node.improvement:.2f} m is the local layer resolving it, and this node")
    add("    leaves it alone. Escalation needs BOTH a conflict older than")
    add(f"    {node.escalate_after:.1f} s and a separation that has stopped improving.")
    add(thin)


def _section_yields(node, add, thin):
    """Section B: the yields themselves."""
    add("[B] YIELDS COMMANDED")
    if not node.yields:
        add("    none - no conflict met the escalation test")
        add(thin)
        return
    add("")
    add("      #  robot  yields to   held     release condition        entry  min sep")
    for index, entry in enumerate(node.yields, start=1):
        add(
            f"    {index:3d}  {entry.robot:<6} {entry.peer:<11} "
            f"{entry.held_s:5.1f}s   {entry.reason:<22} "
            f"{entry.separation_entry:5.2f}m  {entry.separation_min:5.2f}m"
        )
    add("")
    for index, entry in enumerate(node.yields, start=1):
        add(
            f"    yield {index}: escalated {entry.age_at_escalation:.1f} s after the "
            f"conflict was first predicted, at t = {entry.entered:.1f} s"
        )
        add(
            f"             {entry.commands} zero-twist commands published on "
            f"{entry.robot}/cmd_vel_yield (mux priority 150)"
        )
        add(
            f"             released at t = "
            f"{'n/a' if entry.released is None else f'{entry.released:.1f} s'}"
            f" on: {entry.reason}"
        )
    add("")
    add("    A yield is commanded by PUBLISHING zero on the mux's priority-150")
    add("    channel and released by ceasing to publish - the mux's own 0.5 s")
    add("    timeout drops the channel, so there is no release message to lose and")
    add("    an arbiter that dies releases the robot rather than pinning it.")
    add(thin)


def _section_recovery(node, add, thin):
    """Section C: what Nav2 did while a robot was held."""
    add("[C] NAV2 RECOVERY DURING THE HOLD  (ENGINEERING_NOTES rule 2)")
    if not node.bt_log_available:
        add("    behaviour-tree log unavailable on this Nav2 build")
        add(thin)
        return
    during = sum(entry.recoveries_during for entry in node.yields)
    add(
        f"    recovery suppression:                     "
        f"{'ON' if node.suppress_recovery else 'OFF  <- CONTROL':>8}"
    )
    add(f"    recovery behaviours fired DURING a hold:  {during:8d}")
    for name in node.names:
        total = sum(node.recoveries[name].values())
        detail = ", ".join(
            f"{k} x{v}" for k, v in sorted(node.recoveries[name].items())
        )
        add(f"    {name}, whole run:{total:8d}   {detail}")
    add("")
    add(f"    {ALLOWANCE_LABEL}, read back from controller_server:")
    for index, entry in enumerate(node.yields, start=1):
        add(
            f"      yield {index} ({entry.robot}): at entry "
            f"{_allowance(entry.allowance_entry):>12}   after release "
            f"{_allowance(entry.allowance_exit):>12}"
        )
    add("")
    add("    Read back rather than assumed. A write that silently failed would")
    add("    otherwise be indistinguishable from a mechanism that worked and was")
    add("    never needed - which is exactly what Phase 2 could not rule out until")
    add("    Phase 3 produced a halt longer than the allowance.")
    add(thin)


def _section_gate(node, add, thin):
    """Section D: separate the yield from a safety halt."""
    add("[D] SAFETY GATE STATE DURING THE HOLD  (is this a yield or a halt?)")
    if not node.yields:
        add("    no hold to attribute")
        add(thin)
        return
    for index, entry in enumerate(node.yields, start=1):
        share = 0.0 if not entry.commands else entry.gate_blocked_ticks / entry.commands
        add(
            f"    yield {index} ({entry.robot}): SafetyGate was blocking on "
            f"{entry.gate_blocked_ticks} of {entry.commands} held cycles "
            f"({100.0 * share:.0f} %)"
        )
    add("")
    add("    A yield and a safety halt both end with a robot at a standstill. The")
    add("    gate's own diagnostics are recorded alongside the hold so the two are")
    add("    never conflated: 0 % means the robot was stopped by this arbiter and")
    add("    by nothing else.")
    add(thin)


def _section_verdict(node, add, rule):
    """Section E: the pass/fail checks, or a statement that there was nothing to check.

    A run in which no conflict needed arbitration is NOT a failed run, and
    printing FAIL on one would be a lie in the safest-looking direction: the
    artifact would read as a broken yield protocol when what actually happened is
    that the local layer opened the gap and this node correctly stood down. The
    encounter is staged, and a simulator is not obliged to stage it identically
    twice - so the report has to be able to say "not exercised".
    """
    escalated = [c for c in node.conflicts_seen if c.outcome == "escalated"]
    add("[E] VERDICT")
    if not escalated:
        local = [c for c in node.conflicts_seen if c.outcome == "resolved locally"]
        predicted = len(node.conflicts_seen)
        add(f"    conflicts predicted:                      {predicted:8d}")
        add(f"    resolved by the local layer:              {len(local):8d}")
        add(f"    escalated:                                {0:8d}")
        add("    RESULT: NOT EXERCISED")
        add("")
        add("        No conflict in this run met the escalation test, so there is no")
        add("        yield to pass or fail on. This says the encounter did not")
        add("        require central arbitration - which for a conflict the local")
        add("        layer opened up is the CORRECT outcome (rule 7) - and it says")
        add("        nothing about whether the protocol works. The run that")
        add("        exercises it is the one with a yield in section B.")
        add(rule)
        return
    checks = [
        ("a conflict was predicted", bool(node.conflicts_seen)),
        ("a conflict was escalated to a yield", bool(escalated)),
        (
            "every yield was given by the lower-priority robot",
            all(
                node.order.index(entry.robot) > node.order.index(entry.peer)
                for entry in node.yields
            ),
        ),
        (
            "every hold ended",
            all(entry.released is not None for entry in node.yields),
        ),
        (
            "no hold ended on the fail-safe ceiling",
            all("fail-safe" not in entry.reason for entry in node.yields),
        ),
        (
            "no recovery behaviour fired during any hold",
            sum(entry.recoveries_during for entry in node.yields) == 0,
        ),
    ]
    if node.suppress_recovery and node.yields:
        checks.append(
            (
                "the allowance was read back raised at every hold",
                all(
                    entry.allowance_entry is not None and entry.allowance_entry > 100.0
                    for entry in node.yields
                ),
            )
        )
    for label, passed in checks:
        add(f"    {label + ':':<52} {'YES' if passed else 'NO'}")
    add(f"    RESULT: {'PASS' if all(p for _, p in checks) else 'FAIL'}")
    add(rule)


#: Spelled out once so the report and the node cannot drift apart in wording.
ALLOWANCE_LABEL = "progress_checker.movement_time_allowance"


def write_csv(node, path):
    """Write one row per predicted conflict, escalated or not."""
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(
            "pair,first_seen_s,decided_s,age_s,outcome,min_separation_m,"
            "gain_m,yielder,held_s,release_reason,recoveries_during,"
            "allowance_entry_s,allowance_exit_s\n"
        )
        holds = {entry.entered: entry for entry in node.yields}
        for conflict in node.conflicts_seen:
            entry = holds.get(conflict.decided_at)
            held = "" if entry is None else f"{entry.held_s:.3f}"
            reason = "" if entry is None else entry.reason
            during = "" if entry is None else str(entry.recoveries_during)
            allow_in = (
                ""
                if entry is None or entry.allowance_entry is None
                else f"{entry.allowance_entry:.1f}"
            )
            allow_out = (
                ""
                if entry is None or entry.allowance_exit is None
                else f"{entry.allowance_exit:.1f}"
            )
            minimum = (
                "" if math.isinf(conflict.minimum_m) else f"{conflict.minimum_m:.4f}"
            )
            handle.write(
                f"{conflict.pair[0]}|{conflict.pair[1]},{conflict.first_seen:.3f},"
                f"{conflict.decided_at:.3f},{conflict.age_s:.3f},{conflict.outcome},"
                f"{minimum},{conflict.gain_m:.4f},{conflict.yielder},{held},"
                f"{reason},{during},{allow_in},{allow_out}\n"
            )
