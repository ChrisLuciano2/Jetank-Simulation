"""
test_heading.py — offline tests for jetbot_nav.heading.

Synthetic frames are RENDERED THROUGH THE PINHOLE MODEL, not produced by
rolling pixels sideways. That distinction is the whole point: rolling
would bake in the very "a yaw is a uniform pixel shift" assumption the
module deliberately rejects, and the test would then confirm the
assumption rather than the geometry. Here a scene is a set of vertical
stripes at fixed world BEARINGS, and a view at yaw t places each stripe
at column cx + fx*tan(bearing - t) — so a correct estimator recovers t
and a naive atan(shift/fx) one comes back biased high.

    py -3.8 test_heading.py
"""

import math
import sys

import numpy as np

from jetbot_nav.heading import (
    VisualGyro, CourseLock, column_signature, yaw_between,
    degrees_per_bin, far_field_band, lens_params, measurable_range_deg,
    MIN_CONFIDENCE,
)
from jetbot_nav.visual_scan import CameraGeometry, sim_jetank

_results = []


def check(label, ok, detail=""):
    _results.append(bool(ok))
    print(f"{'PASS' if ok else 'FAIL'}  {label}" + (f"  {detail}" if detail and not ok else ""))


GEOM = sim_jetank()
W, H = GEOM.width, GEOM.height

# A fixed, reproducible "world": vertical stripes at assorted bearings,
# widths and brightnesses, spread wider than the lens so that rotating
# brings new scenery into view from the side (as it must in reality).
_rng = np.random.RandomState(7)
STRIPES = [(float(b), float(wdeg), int(v)) for b, wdeg, v in zip(
    _rng.uniform(-70, 70, 26), _rng.uniform(1.2, 4.5, 26),
    _rng.randint(40, 250, 26))]


def render(yaw_deg, geom=GEOM, texture_rows=True):
    """
    Render the stripe world as seen from a camera yawed by `yaw_deg`
    (positive = turned right). Stripes land at cx + fx*tan(b - yaw),
    which is genuine pinhole projection, not a translation.
    """
    fx, cx, _ = lens_params(geom, geom.width)
    frame = np.full((geom.height, geom.width, 3), 18, dtype=np.uint8)

    for bearing, width_deg, value in STRIPES:
        rel = bearing - yaw_deg
        if abs(rel) > 85:
            continue
        u0 = cx + fx * math.tan(math.radians(rel - width_deg / 2))
        u1 = cx + fx * math.tan(math.radians(rel + width_deg / 2))
        a, b = int(round(min(u0, u1))), int(round(max(u0, u1)))
        a, b = max(0, a), min(geom.width, b)
        if b <= a:
            continue
        frame[:, a:b] = value

    if texture_rows:
        # Mild vertical variation so the band mean is not a perfect
        # copy of a single row — closer to a real scene.
        grad = np.linspace(0.85, 1.15, geom.height).reshape(-1, 1, 1)
        frame = np.clip(frame * grad, 0, 255).astype(np.uint8)
    return frame


# ─── 1. Sign convention, in both directions ─────────────────────────────────
# The single easiest thing to get backwards. A wrong sign here turns the
# robot AWAY from its course, faster the further off it gets.

dpb = degrees_per_bin(GEOM, W)
ref = column_signature(render(0.0), GEOM)

check("signature is produced for a textured scene", ref is not None)

right = yaw_between(ref, column_signature(render(+8.0), GEOM), dpb)
left = yaw_between(ref, column_signature(render(-8.0), GEOM), dpb)

check("turning RIGHT reports a positive yaw",
      right is not None and right > 0, f"got {right}")
check("turning LEFT reports a negative yaw",
      left is not None and left < 0, f"got {left}")
check("equal turns either way have equal magnitude",
      right is not None and left is not None and abs(abs(right) - abs(left)) < 1.0,
      f"right={right:.2f} left={left:.2f}")


# ─── 2. Magnitude accuracy against true pinhole geometry ────────────────────
# The angular resampling exists so this is accurate at the EDGES too, not
# just near the optical axis. A naive atan(shift/fx) reads ~20% high.

print("\n      true    measured   error")
worst = 0.0
for truth in [-20, -12, -6, -3, -1, 1, 3, 6, 12, 20]:
    est = yaw_between(ref, column_signature(render(float(truth)), GEOM), dpb)
    if est is None:
        print(f"      {truth:+5.1f}    (no match)")
        worst = 99.0
        continue
    err = est - truth
    worst = max(worst, abs(err))
    print(f"      {truth:+5.1f}    {est:+7.2f}   {err:+6.2f}")

check("yaw magnitude is accurate to within 1.5 deg across +-20 deg",
      worst <= 1.5, f"worst error {worst:.2f} deg")

# Pin the bias the angular resampling removes: correlating raw pixel
# columns and converting with atan(shift/fx) reads systematically high.
fx, cx, hfov = lens_params(GEOM, W)


def naive_yaw(truth):
    """What the module would report WITHOUT angular resampling."""
    band = far_field_band(GEOM, H)
    def raw(frame):
        from jetbot_nav.heading import _luminance, _best_shift
        s = _luminance(frame[band[0]:band[1], :]).mean(axis=0)
        s = s - s.mean()
        n = float(np.sqrt((s ** 2).sum()))
        return s / n if n > 1e-6 else None
    from jetbot_nav.heading import _best_shift
    shift, score, _ = _best_shift(raw(render(0.0)), raw(render(truth)), int(W * 0.45))
    return math.degrees(math.atan(shift / fx))


naive_err = abs(naive_yaw(15.0) - 15.0)
exact_err = abs(yaw_between(ref, column_signature(render(15.0), GEOM), dpb) - 15.0)
check("angular resampling beats the naive pixel-shift conversion",
      exact_err < naive_err, f"resampled err {exact_err:.2f} vs naive {naive_err:.2f}")
print(f"      (at 15 deg: resampled off by {exact_err:.2f}, "
      f"naive pixel-shift off by {naive_err:.2f})")


# ─── 3. Featureless views produce None, never a number ──────────────────────
# A fabricated heading is far worse than a missing one: the caller can
# wait out a missing one but will confidently drive on a wrong one.

blank = np.full((H, W, 3), 128, dtype=np.uint8)
check("a perfectly flat frame yields no signature",
      column_signature(blank, GEOM) is None)
check("yaw_between returns None when a signature is missing",
      yaw_between(ref, None, dpb) is None)


# ─── 4. VisualGyro accumulates a turn ───────────────────────────────────────

gyro = VisualGyro(GEOM)
gyro.update(render(0.0))                      # first frame primes the state
for step in range(1, 13):
    gyro.update(render(step * 2.0))           # 2 deg per frame, 24 deg total

check("gyro accumulates a multi-frame turn to roughly the truth",
      abs(gyro.heading_deg - 24.0) <= 3.0, f"got {gyro.heading_deg:.2f}, want 24")
check("gyro reports no lost frames on a clean sequence",
      gyro.lost_frames == 0, f"lost={gyro.lost_frames}")

# Turning back must unwind the accumulator, not add to it.
for step in range(11, -1, -1):
    gyro.update(render(step * 2.0))
check("reversing the turn unwinds the accumulator toward zero",
      abs(gyro.heading_deg) <= 3.0, f"got {gyro.heading_deg:.2f}")

gyro.reset()
check("reset clears the accumulated heading", gyro.heading_deg == 0.0)

# A lost frame must not silently invent a delta.
g2 = VisualGyro(GEOM)
g2.update(render(0.0))
g2.update(render(4.0))
before = g2.heading_deg
check("a featureless frame returns None", g2.update(blank) is None)
check("a featureless frame leaves the accumulator untouched",
      g2.heading_deg == before, f"{before:.2f} -> {g2.heading_deg:.2f}")
check("a featureless frame is counted as lost", g2.lost_frames == 1)


# ─── 5. CourseLock measures against the ORIGINAL, so it cannot drift ────────

lock = CourseLock(GEOM)
check("capture succeeds on a textured view", lock.capture(render(0.0)))
check("lock reports itself armed", lock.armed)

check("zero error when back at the captured view",
      abs(lock.error_deg(render(0.0))) < 0.5,
      f"got {lock.error_deg(render(0.0))}")
check("turned right of course -> positive error (turn LEFT to recover)",
      lock.error_deg(render(10.0)) > 0, f"got {lock.error_deg(render(10.0))}")
check("turned left of course -> negative error",
      lock.error_deg(render(-10.0)) < 0, f"got {lock.error_deg(render(-10.0))}")

# The property that makes it the drift fix: error depends only on where
# the robot is now, never on the path taken to get there.
lock2 = CourseLock(GEOM)
lock2.capture(render(0.0))
direct = lock2.error_deg(render(14.0))
for wander in [3.0, -9.0, 21.0, -4.0, 17.0]:      # thrash around first
    lock2.error_deg(render(wander))
after_wandering = lock2.error_deg(render(14.0))
check("error is path-independent (the anti-drift property)",
      abs(direct - after_wandering) < 1e-9,
      f"{direct:.4f} vs {after_wandering:.4f}")

check("unarmed lock reports None rather than zero error",
      CourseLock(GEOM).error_deg(render(0.0)) is None)
check("capture reports failure on a featureless view",
      not CourseLock(GEOM).capture(blank))

check("on_course accepts a small residual", CourseLock.on_course(2.0))
check("on_course rejects a large one", not CourseLock.on_course(25.0))
check("on_course treats None as not-on-course", not CourseLock.on_course(None))


# ─── 6. A closed-loop recovery actually converges ───────────────────────────
# The realistic use: knocked off course by a detour, turn back using the
# lock's error until on course. Uses a crude proportional turn with the
# same 10 Hz tick the drive loop runs at.

def recover_from(start_yaw, max_ticks=60):
    lock = CourseLock(GEOM)
    lock.capture(render(0.0))
    yaw = start_yaw
    for tick in range(max_ticks):
        err = lock.error_deg(render(yaw))
        if err is None:
            return None, tick
        if CourseLock.on_course(err):
            return yaw, tick
        yaw -= max(-6.0, min(6.0, 0.6 * err))    # rate-limited correction
    return yaw, max_ticks


print()
converged = True
for start in [-24.0, -18.0, -7.0, 7.0, 18.0, 24.0]:
    final, ticks = recover_from(start)
    ok = final is not None and abs(final) <= 4.0
    converged &= ok
    print(f"      from {start:+6.1f} deg -> "
          f"{'settled at %+5.1f in %d ticks' % (final, ticks) if final is not None else 'LOST'}")
check("closed-loop recovery converges from within the measurable range",
      converged)


# ─── 6b. Beyond the measurable range: None, never a confident wrong answer ──
# Correlation always returns SOME argmax. When the true alignment is
# outside the searched range that argmax is spurious, and it can clear an
# absolute score threshold easily — this reported "you are 1.8 deg off
# course" for a robot 30 deg off, which would have ended the recovery
# instantly and left it pointing the wrong way. The prominence gate is
# what catches it.

rng_deg = measurable_range_deg(GEOM)
print(f"\n      measurable range: +-{rng_deg:.1f} deg")
check("measurable range is a sane fraction of the lens FOV",
      20.0 < rng_deg < GEOM.hfov_deg, f"got {rng_deg:.1f}")

far_lock = CourseLock(GEOM)
far_lock.capture(render(0.0))
beyond = [far_lock.error_deg(render(y)) for y in (35.0, 45.0, -35.0, -45.0)]
check("yaw beyond the measurable range reports None, not a wrong number",
      all(e is None for e in beyond), f"got {beyond}")

within = [far_lock.error_deg(render(y)) for y in (24.0, -24.0)]
check("yaw just inside the range still measures correctly",
      all(e is not None and abs(abs(e) - 24.0) < 1.5 for e in within),
      f"got {within}")


# ─── 6c. The gate thresholds, against MEASURED values ───────────────────────
# Both thresholds were originally set from synthetic scenes and were too
# strict for real renders: MIN_PEAK_PROMINENCE = 1.8 rejected four of six
# valid measurements from Unity, including a 5 deg turn that missed by
# 0.01. Rather than leave the choice as a bare constant that the next
# person re-guesses, pin it against the actual observations from both
# sides. Rows marked accept=True are correct measurements that MUST get
# through; accept=False are wrong answers that MUST be rejected.

from jetbot_nav.heading import MIN_PEAK_PROMINENCE

GATE_TABLE = [
    # (score, prominence, accept, source)
    (0.755, 1.52, True,  "Unity render, -20 deg (correct)"),
    (0.935, 1.68, True,  "Unity render, -10 deg (correct)"),
    (0.972, 1.79, True,  "Unity render,  -5 deg (correct)"),
    (0.966, 2.16, True,  "Unity render,  +5 deg (correct, +0.57 err)"),
    (0.905, 2.01, True,  "Unity render, +10 deg (correct, +1.14 err)"),
    (0.763, 1.66, True,  "Unity render, +20 deg (correct)"),
    # Repetitive scenery: score is HIGH and the answer is wrong. Only
    # prominence can see these.
    (0.989, 1.00, False, "periodic stripes: score high, answer off by 8 deg"),
    (0.970, 1.00, False, "periodic stripes: score high, answer off by 14 deg"),
    (0.980, 1.02, False, "periodic stripes: score high, answer off by 14 deg"),
    # Beyond the measurable range: correlation settles on an interior
    # impostor. Prominence does NOT reliably catch these (several exceed
    # the real-match floor), so the score gate has to.
    (0.4463, 1.17, False, "beyond range, +30 deg -> reported -1.85"),
    (0.4684, 1.37, False, "beyond range, +35 deg -> reported +2.53"),
    (0.4808, 1.66, False, "beyond range, +45 deg -> reported +12.54"),
    (0.4889, 1.71, False, "beyond range, -30 deg -> reported +1.46"),
    (0.4385, 1.38, False, "beyond range, -35 deg -> reported -3.63"),
    (0.5001, 1.68, False, "beyond range, +40 deg -> reported +7.56"),
    (0.5132, 1.49, False, "beyond range, -40 deg -> reported -9.13"),
    (0.5207, 1.56, False, "beyond range, -45 deg -> reported -14.12"),
]


def accepted(score, prom):
    return score >= MIN_CONFIDENCE and prom >= MIN_PEAK_PROMINENCE


print()
wrong = []
for score, prom, want, source in GATE_TABLE:
    if accepted(score, prom) != want:
        wrong.append(f"{source}: score {score}, prom {prom} -> "
                     f"{'wrongly accepted' if not want else 'wrongly rejected'}")
check("gate thresholds accept every measured-correct match and reject "
      "every measured-wrong one", not wrong,
      "; ".join(wrong))

# Each threshold must sit in a GAP between the two populations, not on top
# of one of them. Sitting on top is how 1.8 came to reject a valid 5 deg
# turn by 0.01, and how 0.5 came to admit a 45 deg one as -14 deg.
good_prom = min(p for _, p, a, _ in GATE_TABLE if a)
bad_prom = max(p for s, p, a, _ in GATE_TABLE
               if not a and s >= MIN_CONFIDENCE)
check("prominence threshold sits in a gap between the populations",
      bad_prom < MIN_PEAK_PROMINENCE < good_prom,
      f"worst accepted {good_prom}, best score-passing reject {bad_prom}, "
      f"threshold {MIN_PEAK_PROMINENCE}")

good_score = min(s for s, _, a, _ in GATE_TABLE if a)
bad_score = max(s for s, p, a, _ in GATE_TABLE
                if not a and p >= MIN_PEAK_PROMINENCE)
check("score threshold sits in a gap between the populations",
      bad_score < MIN_CONFIDENCE < good_score,
      f"worst accepted {good_score}, best prominence-passing reject "
      f"{bad_score}, threshold {MIN_CONFIDENCE}")


# ─── 6d. A railed peak is rejected, not reported ────────────────────────────
# Regression for the first live course-keeping run, where VisualGyro
# accumulated in steps of exactly max_shift * deg_per_bin (+28.0 deg) and
# course keeping steered on the total. When the true alignment is outside
# the searched span the argmax pins to the boundary, and a boundary peak
# passes both score gates: its overlap is the narrowest available (easy to
# match by accident) and it is one-sided, so the excluded-rival set holds
# only the far tail and prominence comes out high.
#
# Constructed directly rather than by rendering a large yaw, because the
# point is what happens AT the boundary regardless of how it got there.

from jetbot_nav.heading import (
    MAX_SHIFT_FRACTION, RAIL_MARGIN_BINS, yaw_between,
)

_n = 640
_max_shift = int(_n * MAX_SHIFT_FRACTION)
_dpb = degrees_per_bin(GEOM, _n)

# A signature that correlates best with itself shifted right to the rail:
# a single sharp feature, so the peak is unambiguous and clears both gates.
_rng = np.random.RandomState(7)
_base = _rng.randn(_n)


def _norm(v):
    v = v - v.mean()
    return v / float(np.sqrt((v ** 2).sum()))


for _railed_shift in (_max_shift, -_max_shift):
    _ref = _norm(_base)
    _live = _norm(np.roll(_base, -_railed_shift))
    _got = yaw_between(_ref, _live, _dpb, max_shift=_max_shift)
    check(f"peak railed at {_railed_shift:+d} bins reports None, not "
          f"{_railed_shift * _dpb:+.1f} deg",
          _got is None, f"got {_got}")

# The rejection must be narrow: a genuine large-but-measurable yaw still
# has to get through, or the fix has simply broken the estimator.
_inside = _max_shift - RAIL_MARGIN_BINS - 4
_ref = _norm(_base)
_live = _norm(np.roll(_base, -_inside))
_got = yaw_between(_ref, _live, _dpb, max_shift=_max_shift)
check("a shift just inside the rail still measures",
      _got is not None and abs(_got - _inside * _dpb) < 0.5,
      f"want {_inside * _dpb:+.2f}, got {_got}")

check("measurable_range_deg excludes the rejected boundary bins",
      measurable_range_deg(GEOM, _n) < _max_shift * _dpb,
      f"range {measurable_range_deg(GEOM, _n):.3f} vs searched "
      f"{_max_shift * _dpb:.3f}")


# ─── 6e. One unmeasurable step must not blind the gyro forever ──────────────
# The rail rejection above turns a fast turn into a None. That is correct,
# but update() also has to re-baseline on the way out, or every subsequent
# frame is compared against the pre-jump view and fails for the same
# reason — one bad step becomes a permanent blackout. Measured live: 133
# of 164 ticks with no heading at all after a single fast pivot.

_dpb7 = degrees_per_bin(GEOM, 640)
_prof = np.random.RandomState(3).randn(4000)
_prof = np.convolve(_prof, np.ones(9) / 9, mode="same")
_prof = ((_prof - _prof.min()) / (_prof.max() - _prof.min()) * 255)
_world = np.repeat(_prof.astype(np.uint8)[None, :], 480, axis=0)
_world = np.repeat(_world[:, :, None], 3, axis=2)


def _pan(yaw_deg):
    c = 1500 + int(round(yaw_deg / _dpb7))
    return _world[:, c:c + 640, :]


_g = VisualGyro(GEOM)
_g.update(_pan(0.0))
_before = [_g.update(_pan(y)) for y in (2.0, 4.0, 6.0)]
check("gyro tracks small steps before the jump",
      all(d is not None for d in _before), f"got {_before}")

_jump = _g.update(_pan(60.0))          # far outside the measurable range
check("an out-of-range step reports None rather than a railed number",
      _jump is None, f"got {_jump}")

_after = [_g.update(_pan(y)) for y in (62.0, 64.0, 66.0)]
check("gyro RESUMES tracking on the next frame after an unmeasurable step",
      all(d is not None and abs(d - 2.0) < 0.6 for d in _after),
      f"got {_after}")

check("lost_frames clears once tracking resumes",
      _g.lost_frames == 0, f"got {_g.lost_frames}")

# The missed rotation stays missing — that is the honest outcome, and it
# is precisely why CourseKeeper declares UNKNOWN and CourseLock re-anchors
# rather than trusting the accumulator indefinitely.
check("the unmeasured rotation is a permanent offset, not silently invented",
      _g.heading_deg < 20.0,
      f"heading {_g.heading_deg:.1f} after 66 deg of true rotation")


# ─── 6f. Lever-arm correction ───────────────────────────────────────────────
# A camera mounted ahead of the turning centre swings sideways when the
# robot turns, and that sideways motion is indistinguishable from extra
# rotation. Measured in sim: readings ran 8.0-9.4% high across rotation in
# place and turns while driving. Per tick that is invisible; VisualGyro
# accumulates, so over a drive it ran the total past 360 deg.

from jetbot_nav.heading import (
    lever_arm_gain, MIN_CORRECTABLE_DEPTH, MAX_LEVER_GAIN,
)
from jetbot_nav.visual_scan import CameraGeometry

_off = CameraGeometry(height_m=0.8, tilt_deg=20.0)                    # no offset
_on = CameraGeometry(height_m=0.8, tilt_deg=20.0, pivot_offset=0.8)

check("no correction when the geometry does not say where the camera sits",
      lever_arm_gain(_off, 12.0) == 1.0, f"got {lever_arm_gain(_off, 12.0)}")
check("no correction when depth is unknown",
      lever_arm_gain(_on, None) == 1.0, f"got {lever_arm_gain(_on, None)}")
check("gain matches 1 + offset/depth",
      abs(lever_arm_gain(_on, 12.0) - (1.0 + 0.8 / 12.0)) < 1e-9,
      f"got {lever_arm_gain(_on, 12.0)}")
check("further away means less correction",
      lever_arm_gain(_on, 24.0) < lever_arm_gain(_on, 6.0),
      f"{lever_arm_gain(_on, 24.0)} vs {lever_arm_gain(_on, 6.0)}")

# Guards. A correction that can scale a reading without limit is just a
# new way to be confidently wrong.
check("correction disabled below MIN_CORRECTABLE_DEPTH",
      lever_arm_gain(_on, MIN_CORRECTABLE_DEPTH - 0.01) == 1.0,
      f"got {lever_arm_gain(_on, MIN_CORRECTABLE_DEPTH - 0.01)}")
check("gain is capped at MAX_LEVER_GAIN",
      lever_arm_gain(_on, MIN_CORRECTABLE_DEPTH) <= MAX_LEVER_GAIN,
      f"got {lever_arm_gain(_on, MIN_CORRECTABLE_DEPTH)}")

# End to end: the same rotation, corrected and uncorrected, using the
# panning world from 6e so the "true" answer is known exactly.
_g_raw = VisualGyro(_off)
_g_cor = VisualGyro(_on)
_g_raw.update(_pan(0.0))
_g_cor.update(_pan(0.0))
for _y in (2.0, 4.0, 6.0, 8.0, 10.0):
    _g_raw.update(_pan(_y))
    _g_cor.update(_pan(_y), depth=12.0)
check("the correction reduces an accumulated reading, and only slightly",
      _g_cor.heading_deg < _g_raw.heading_deg
      and _g_cor.heading_deg > 0.85 * _g_raw.heading_deg,
      f"raw {_g_raw.heading_deg:.2f} corrected {_g_cor.heading_deg:.2f}")

# A caller that never supplies depth must get exactly the old behaviour,
# so adding the offset to a geometry cannot change existing results on its
# own. This is what makes the correction safe to add to sim_jetank.
_g_nodepth = VisualGyro(_on)
_g_nodepth.update(_pan(0.0))
for _y in (2.0, 4.0, 6.0, 8.0, 10.0):
    _g_nodepth.update(_pan(_y))
check("an offset geometry with no depth behaves exactly as before",
      abs(_g_nodepth.heading_deg - _g_raw.heading_deg) < 1e-9,
      f"nodepth {_g_nodepth.heading_deg:.4f} raw {_g_raw.heading_deg:.4f}")


# ─── 7. Works without a CameraGeometry (hardware default lens) ──────────────

plain = CourseLock(None)
plain.capture(render(0.0))
err = plain.error_deg(render(9.0))
check("usable with no CameraGeometry, assuming the default lens",
      err is not None and err > 0, f"got {err}")


# ─── Summary ────────────────────────────────────────────────────────────────

passed = sum(_results)
print(f"\n{passed}/{len(_results)} checks passed")
sys.exit(0 if all(_results) else 1)
