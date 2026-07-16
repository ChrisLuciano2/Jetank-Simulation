"""
test_avoidance_logic.py — OFFLINE regression tests for jetbot_nav.navigation.

Runs with NO Unity and NO robot: scripted sensor sequences are fed straight
into AvoidanceController and its decisions are asserted. Every previously
observed field bug is encoded here as a scenario, so if a future change
re-introduces one, this catches it in seconds instead of a live test session.

Run:
    cd Python
    py test_avoidance_logic.py

Expected output: one PASS line per scenario, exit code 0.
(If a scenario fails, it prints the tick-by-tick trace for that scenario so
you can see exactly where the controller went wrong.)
"""

import sys

from jetbot_nav import navigation
from jetbot_nav.navigation import (
    AvoidanceController, CRUISE, AVOID, PIVOT, BACKUP,
    MAX_SPEED, MAX_BACKUP_ATTEMPTS, STOP_DISTANCE,
)

MAX_RANGE = 12.0   # matches the recommended Unity Inspector value

_results = []


def run_trace(controller, readings):
    """Feed (left, center, right) tuples; return list of per-tick records."""
    trace = []
    for (l, c, r) in readings:
        motors = controller.step({"left": l, "center": c, "right": r})
        trace.append({"prox": (l, c, r), "motors": motors, **controller.debug()})
    return trace


def check(name, condition, trace=None, detail=""):
    if condition:
        print(f"PASS  {name}")
        _results.append(True)
    else:
        print(f"FAIL  {name}  {detail}")
        if trace:
            for i, t in enumerate(trace):
                l, c, r = t["prox"]
                ml, mr = t["motors"]
                print(f"   tick {i:3d}: L={l:5.2f} C={c:5.2f} R={r:5.2f} -> "
                      f"({ml:5.2f},{mr:5.2f}) {t['state']:6s} "
                      f"escape={t['escape_side']} attempts={t['backup_attempts']}")
        _results.append(False)


def clear():
    """A fully-clear reading (nothing detected -> sensor reports max range)."""
    return (MAX_RANGE, MAX_RANGE, MAX_RANGE)


# ─── Scenario 1: open field — must drive straight, never turn ────────────────
# Regression for: "keeps turning even when clear" (the maxRange/SLOW_DOWN
# mismatch made an empty field look like a wall at 5 m).

def scenario_open_field():
    ctrl = AvoidanceController(max_range=MAX_RANGE)
    trace = run_trace(ctrl, [clear()] * 30)
    all_straight = all(t["motors"] == (MAX_SPEED, MAX_SPEED) for t in trace)
    check("open field: drives straight every tick", all_straight, trace)


# ─── Scenario 2: pass a single block, then straighten out ────────────────────
# Regression for: "clears one block but keeps turning that same direction".
# Phase A: block ahead-right (steer left). Phase B: block is beside us —
# right sensor still sees it at ~3.6 m but center is open; the trailing
# sensor must NOT keep forcing a turn. Phase C: fully past — all clear.

def scenario_pass_block_then_straighten():
    ctrl = AvoidanceController(max_range=MAX_RANGE)
    # Approach: block ahead-right; steering left arrests the closure before
    # anything reaches backup range.
    approach = [(MAX_RANGE, 7.0 - 0.3 * i, 6.0 - 0.3 * i) for i in range(7)]
    # Passing: block is now BESIDE us (right sensor 4.2 m — outside the
    # side-threat zone), forward path fully open.
    passing  = [(MAX_RANGE, MAX_RANGE, 4.2)] * 8
    gone     = [clear()] * 15

    trace = run_trace(ctrl, approach + passing + gone)

    a = trace[1:7]
    steered_left = all(t["motors"][0] < t["motors"][1] for t in a)
    check("pass block: initially steers away (left)", steered_left, trace)

    b = trace[7:15]
    # A block merely beside us (outside SIDE_THREAT_DISTANCE) must not keep
    # forcing a turn — the robot should drive dead straight while passing it.
    no_lingering_turn = all(abs(t["motors"][0] - t["motors"][1]) < 1e-9 for t in b)
    check("pass block: trailing side sensor does not keep forcing a turn",
          no_lingering_turn, trace,
          detail="(this was the lingering-turn half of bug 1)")

    tail = trace[-8:]
    ended_straight = all(
        t["state"] == CRUISE and t["motors"] == (MAX_SPEED, MAX_SPEED)
        for t in tail
    )
    check("pass block: returns to CRUISE straight once clear", ended_straight, trace)


# ─── Scenario 3: wide symmetric wall — direction must never flip mid-fight ───
# Regression for: "gets stuck in a loop… starts trying to go the other
# direction after it starts the backup". Left/right alternate which reads
# larger (symmetric wall noise), backups occur, and while pivoting the
# wall-side sensor sweeps very close — the old code flipped here.

def scenario_wall_no_flip_flop():
    # Phase 1: symmetric approach with alternating left/right jitter, then
    # contact (all sensors inside backup range). Run it, then read which
    # side the controller committed to.
    ctrl = AvoidanceController(max_range=MAX_RANGE)
    phase1 = []
    for i in range(6):
        base = 6.0 - 0.5 * i
        jitter = 0.05 if i % 2 == 0 else -0.05
        phase1.append((base + jitter, base, base - jitter))
    phase1 += [(3.0, 3.0, 3.0)] * 8
    trace = run_trace(ctrl, phase1)
    side = ctrl.escape_side
    check("wall: a direction is committed on contact",
          side in ("left", "right"), trace)

    # Phase 2: pivoting along the wall — the wall-side sensor (opposite the
    # escape side) reads VERY close the whole time, while the center slowly
    # opens up. The old code abandoned its commitment exactly here (because
    # a committed-side/wall-side sensor dipped below STOP_DISTANCE) and
    # re-derived the direction from raw readings, flipping every cycle.
    phase2 = []
    for i in range(12):
        center = min(3.2 + 0.35 * i, MAX_RANGE)
        if side == "left":
            phase2.append((MAX_RANGE, center, 3.2))   # wall swept by RIGHT sensor
        else:
            phase2.append((3.2, center, MAX_RANGE))   # wall swept by LEFT sensor
    trace += run_trace(ctrl, phase2 + [clear()] * 10)

    sides_seen = {t["escape_side"] for t in trace if t["escape_side"]}
    check("wall: escape direction NEVER flips during the encounter",
          sides_seen == {side}, trace,
          detail=f"sides seen: {sides_seen} (this was bug 2)")
    check("wall: ends the encounter in CRUISE",
          trace[-1]["state"] == CRUISE, trace)




# ─── Scenario 4: dead-end corner — flip exactly when attempts run out ────────
# Both directions blocked no matter what: the controller should persist with
# one direction for MAX_BACKUP_ATTEMPTS backups, then flip ONCE — never
# per-tick flapping.

def scenario_corner_flip_discipline():
    ctrl = AvoidanceController(max_range=MAX_RANGE)
    blocked = (3.0, 3.0, 3.0)          # inside backup range every re-approach
    trace = run_trace(ctrl, [blocked] * 200)

    # Count how many backups happened between consecutive flips.
    flips = trace[-1]["flips"]
    check("corner: escape side flips at least once (not stuck forever)",
          flips >= 1, trace)

    # Direction changes must be exactly at attempt-counter boundaries: the
    # side sequence should look like AAAA...BBBB...AAAA, never ABAB per tick.
    sides = [t["escape_side"] for t in trace if t["escape_side"]]
    changes = sum(1 for a, b in zip(sides, sides[1:]) if a != b)
    check("corner: direction changes are rare, deliberate flips (no flapping)",
          changes == flips, trace,
          detail=f"{changes} raw changes vs {flips} counted flips")


# ─── Scenario 5: constants sanity — legacy 5.0 m sensor auto-clamps ──────────
# Regression for the root cause itself: with a 5.0 sensor the slow-down band
# must clamp below the max range, so "clear" remains detectable.

def scenario_legacy_range_clamp():
    ctrl = AvoidanceController(max_range=5.0)
    check("legacy sensor: slow-down band clamped below max range",
          ctrl.slow_down < 5.0 and ctrl.slow_down > STOP_DISTANCE,
          detail=f"slow_down={ctrl.slow_down}")
    # And a clear field (readings == 5.0) must actually go straight:
    trace = run_trace(ctrl, [(5.0, 5.0, 5.0)] * 20)
    tail_straight = all(t["motors"] == (MAX_SPEED, MAX_SPEED) for t in trace[-5:])
    check("legacy sensor: clear field still resolves to straight driving",
          tail_straight, trace,
          detail="(with the old constants this was impossible — bug 1 root cause)")


# ─── Run all ─────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    scenario_open_field()
    scenario_pass_block_then_straighten()
    scenario_wall_no_flip_flop()
    scenario_corner_flip_discipline()
    scenario_legacy_range_clamp()

    passed = sum(_results)
    print(f"\n{passed}/{len(_results)} checks passed")
    sys.exit(0 if all(_results) else 1)
