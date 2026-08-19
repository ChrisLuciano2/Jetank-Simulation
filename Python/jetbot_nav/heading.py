"""
jetbot_nav.heading — relative heading from the camera alone.

WHY NOT AN ENCODER, AN IMU, OR THE SIMULATOR'S TRANSFORM
────────────────────────────────────────────────────────────────
To resume a course after detouring around an obstacle, the robot has to
know how far it turned. It has no way to know:

  - no wheel encoders (the SCSCtrl shim documents "no encoder read-back",
    and tank tracks slip worst during exactly the turns being measured),
  - no IMU on the stock build,
  - and Unity's transform is ground truth that does not exist on the
    Jetson, so reading it would be the same class of mistake as feeding
    navigation from SimCamera's detect_objects.

What both machines genuinely have is the camera. Rotating the camera
translates the image sideways, so yaw is recoverable from the picture.

READ HEADING FROM THE FAR FIELD, NOT THE WHOLE FRAME
────────────────────────────────────────────────────────────────
The robot translates while it rotates, and translation also moves the
image — but by an amount that depends on depth. Near content sweeps
across the frame while distant content barely moves, so a whole-frame
estimate reports some unknowable blend of "I turned" and "I drove past
something". Restricting the estimate to content at and above the horizon
removes most of that: those pixels are far away, so their parallax under
translation is small and what is left is dominated by rotation.

Conveniently this is exactly the part of the frame jetbot_nav.visual_scan
discards. visual_scan walks UP from the bottom to find where the floor
stops (near field, distance information); heading reads the band around
and above the horizon (far field, rotation information). One camera, two
non-overlapping jobs.

HORIZONTAL SIGNATURE + 1D CORRELATION, NOT FEATURE TRACKING
────────────────────────────────────────────────────────────────
Yaw moves the whole far-field band sideways together, so the band can be
collapsed to one number per direction (a mean down its rows) and the
rotation recovered by correlating two of those 1D signatures. That is a
few thousand operations per frame instead of a feature detector and
matcher, which matters on a Nano already running the drive loop — and it
has no detector thresholds to go wrong on a bare wall.

The cost is that it cannot separate rotation from sideways translation.
A tracked robot cannot translate sideways without rotating, so this is
close to free here.

RESAMPLE TO ANGLE FIRST — A YAW IS NOT A UNIFORM PIXEL SHIFT
────────────────────────────────────────────────────────────────
It is tempting to correlate raw pixel columns and convert the winning
shift with atan(shift / fx). That is wrong away from the image centre. A
world direction at bearing b sits at column cx + fx*tan(b), so rotating
by t moves it to cx + fx*tan(b - t): the pixel shift GROWS toward the
edges. On this lens a 15 deg yaw moves the image centre by ~142 px but
one edge by ~231 px, so no single shift describes the frame. Correlation
lands on a blend and reports a yaw biased high — measured at ~7% for that
15 deg case, consistently and in the same direction, which looks like a
calibration error and integrates into steady drift.

So the signature is resampled into equal-ANGLE bins before correlating.
In that space a rotation genuinely is a uniform shift, the correlation
peak means exactly one thing, and the conversion is a multiply by the
bin width with no trigonometry left to get wrong.

DRIFT, AND WHY SNAPSHOTS EXIST
────────────────────────────────────────────────────────────────
VisualGyro accumulates per-frame deltas, so it accumulates per-frame
error too — a slow walk away from truth that no amount of care in the
per-frame estimate removes. It is reliable over the seconds a detour
lasts, not over a whole run.

CourseLock fixes that where it counts: photograph the view when leaving
the course, then compare the LIVE frame against that photograph rather
than against the previous frame. The error is measured against the
original directly, so nothing accumulates. Use the gyro to steer during
the detour and the lock to finish the turn.

CONFIDENCE IS NOT OPTIONAL
────────────────────────────────────────────────────────────────
A wrong heading is far worse than a missing one: the caller can hold
still and wait out a missing one, but it will confidently drive the wrong
way on a fabricated one. So there are two independent rejection gates,
and every entry point returns None rather than guess.

  - ABSOLUTE SCORE. Pointed at a blank wall there is nothing to
    correlate and the peak means nothing.
  - PEAK PROMINENCE. Subtler and the one that actually bit during
    development: correlation always returns its argmax, so when the true
    alignment lies OUTSIDE the searched range, it happily reports a
    spurious peak that clears any fixed score threshold. Requiring the
    peak to tower over well-separated rivals catches that, and being a
    ratio it survives the overall score drop real optics cause.

MEASURABLE RANGE
────────────────────────────────────────────────────────────────
Roughly +/-28 deg on a 62 deg lens (see measurable_range_deg()). The
limit is close to physical rather than a tuning choice: rotate past the
field of view and the two frames share no scenery to align. This is why
the division of labour matters — VisualGyro tracks arbitrarily large
rotations because each frame-to-frame step is small, and CourseLock only
ever has to measure the last stretch back onto course.

TESTING
    py -3.8 test_heading.py    — offline, synthetic frames, no Unity
"""

import math

import numpy as np

# ─── Tuning ──────────────────────────────────────────────────────────────────

MIN_CONFIDENCE = 0.65   # normalised correlation peak below this -> no estimate.
                        # A featureless band correlates weakly with everything,
                        # and reporting its argmax would be pure noise dressed
                        # up as a measurement.
                        #
                        # This gate is what rejects OUT-OF-RANGE rotations,
                        # where the true alignment lies outside the searched
                        # span and correlation settles on an interior
                        # impostor. Those impostors were measured at
                        # 0.44-0.52 across +-30..45 deg, while correct matches
                        # score 0.98+ synthetically and 0.755+ on real Unity
                        # renders. 0.65 sits in that gap. It was 0.5, which
                        # let three out-of-range cases through as confident
                        # wrong answers once MIN_PEAK_PROMINENCE was relaxed
                        # to fit real scenes — the two gates have to be
                        # retuned together, since each covers what the other
                        # misses.

MIN_PEAK_PROMINENCE = 1.35  # best score must beat the best WELL-SEPARATED
                        # rival by this ratio. Catches what an absolute
                        # threshold misses: correlation always returns its
                        # argmax, so on REPETITIVE scenery it reports a
                        # confident, high-scoring, completely wrong peak
                        # (measured: score 0.97 with the answer off by 8 deg).
                        #
                        # Was 1.8, chosen from synthetic scenes where genuine
                        # matches scored 1.96-2.63. Real Unity renders are
                        # much less distinctive than synthetic stripes and
                        # measured 1.52-2.16 for CORRECT matches, so 1.8
                        # rejected four of six valid measurements including a
                        # 5 deg turn that missed the bar by 0.01. The three
                        # cases this gate must still reject cluster at
                        # 1.00-1.17, well below the real-match floor, so 1.35
                        # sits in the gap rather than on top of either group.
                        # See test_heading.py's gate table for the values.
                        #
                        # This gate does NOT have to catch everything on its
                        # own: out-of-range matches also collapse in absolute
                        # score (~0.45 against ~0.9 for real matches), so
                        # MIN_CONFIDENCE covers them. Each gate handles the
                        # failure the other cannot see.

PROMINENCE_EXCLUSION = 0.05  # rivals within this fraction of the width of the
                        # peak are part of the same lobe, not competitors.
                        #
                        # Deriving this from the curve instead of fixing it was
                        # tried and is a trap: with no rival left outside a
                        # broad lobe, prominence goes to infinity, and a
                        # featureless view — which correlates with itself at
                        # 0.998 however far the robot turned — sails through as
                        # a confident 0.0 deg. The regime difference it was
                        # meant to solve is handled by the two thresholds
                        # below instead.

GYRO_MIN_PEAK_PROMINENCE = 1.05  # prominence floor for FRAME-TO-FRAME matches.
                        # Much lower than MIN_PEAK_PROMINENCE because the two
                        # comparisons are not alike. CourseLock matches against
                        # a photo taken degrees away, giving a sharp peak.
                        # VisualGyro matches consecutive frames that are nearly
                        # identical, and a near-identical pair has a broad,
                        # flat-topped correlation curve whose best
                        # well-separated rival still sits on the peak's own
                        # lobe. The same reading is therefore worth far less
                        # prominence here, and judging it by CourseLock's
                        # standard threw away most of the good data.
                        #
                        # The cost of over-rejecting is also different, which
                        # is what makes this worth splitting. A rejected
                        # CourseLock reading is free: try again next tick,
                        # nothing accumulates. A rejected gyro reading is a
                        # PERMANENT hole in the accumulated total, exactly as
                        # unrecoverable as a wrong one. At 1.35 that cost was
                        # 49 of 130 ticks rejected on a live drive, discarding
                        # 132 deg of real rotation in 25 seconds.
                        #
                        # Both populations measured against Unity ground truth
                        # on a textured scene, 5 deg turns unless noted:
                        #
                        #   WRONG   (nose to a wall, one texture band filling
                        #            the frame; errors 3.3-8.6 deg)
                        #            n=63   prominence 1.015 - 1.04
                        #   CORRECT (while driving; mean error 0.59 deg)
                        #            n=41   prominence 1.06  - 1.34
                        #   CORRECT (while driving, already accepted at 1.35)
                        #            n=99   prominence 1.38  - 11.35
                        #   CORRECT (static, textured; errors 0.10-0.30 deg)
                        #            n=8    prominence 1.72  - 6.53
                        #
                        # 1.05 sits in the gap between 1.04 and 1.06 rather
                        # than on top of either group, which is the same
                        # standard MIN_PEAK_PROMINENCE is held to. Be honest
                        # about the margin though: it is 0.01 either side,
                        # far tighter than the gap CourseLock's threshold
                        # enjoys. It rests on the wrong population being
                        # tightly clustered — 63 samples spanning 0.013 — not
                        # on comfortable separation. Re-measure both
                        # populations before moving it, and do not tune it
                        # against a scene with untextured walls.

MAX_SHIFT_FRACTION = 0.45   # widest shift searched, as a fraction of image
                        # width. Beyond this the two views barely overlap and
                        # the "best" alignment is comparing different scenery.
                        # This caps the MEASURABLE yaw at roughly 0.45 * hfov
                        # (~28 deg on a 62 deg lens) -- see
                        # measurable_range_deg(). That ceiling is close to
                        # physical: rotate past the FOV and there is no shared
                        # scenery left to match at all. Larger rotations are
                        # VisualGyro's job (each frame-to-frame step is tiny);
                        # CourseLock only has to finish the finalapproach.
RAIL_MARGIN_BINS = 1    # reject peaks landing this close to the edge of the
                        # searched span. NOT a tuning knob — it closes a hole
                        # neither gate above can see.
                        #
                        # _best_shift only searches [-max_shift, +max_shift].
                        # If the true alignment lies beyond that, the argmax
                        # gets pinned AT the boundary, and a boundary peak is
                        # indistinguishable from a real one: it is one-sided,
                        # so the excluded-rival set that MIN_PEAK_PROMINENCE
                        # divides by contains only the far tail, and the ratio
                        # comes out high. The score survives too, because the
                        # smallest overlap is also the easiest to match by
                        # accident.
                        #
                        # This is not theoretical. The first live
                        # course-keeping run had VisualGyro accumulating in
                        # steps of exactly +28.0 deg — max_shift * deg_per_bin
                        # to three figures — through both gates, and course
                        # keeping steered confidently on the total. A peak on
                        # the boundary does not mean "the answer is exactly
                        # 28 deg", it means "the answer is 28 deg OR MORE, and
                        # I cannot tell which", so the only honest report is
                        # None. Rejecting the adjacent bin too, because
                        # parabolic refinement needs a neighbour on each side
                        # and silently skips at the edge.

FAR_FIELD_MARGIN = 0.15  # how far BELOW the horizon to keep reading, as a
                        # fraction of image height. Content just under the
                        # horizon is still distant, and the sim's sky is often
                        # a smooth gradient with nothing to correlate, so the
                        # band needs to reach far enough down to catch actual
                        # objects.

ARRIVED_DEG = 3.0       # |error| under this counts as back on course. Tighter
                        # than the robot can reliably hold given yaw friction,
                        # so demanding more just spins in place.


# ─── Signature extraction ────────────────────────────────────────────────────

def _luminance(frame_rgb: np.ndarray) -> np.ndarray:
    """Rec. 601 luma. Written out rather than calling cv2.cvtColor so this
    module works on a numpy-only install, and so the identical arithmetic
    runs on the Jetson."""
    f = frame_rgb.astype(np.float32)
    return 0.299 * f[:, :, 0] + 0.587 * f[:, :, 1] + 0.114 * f[:, :, 2]


def far_field_band(geom, image_height: int = None) -> tuple:
    """
    (top_row, bottom_row) of the band to read heading from: the top of the
    frame down to a little below the horizon. See the module docstring for
    why the near field is excluded.

    `geom` is a jetbot_nav.visual_scan.CameraGeometry (used only for
    horizon_row), or None to fall back to the top half of the frame.
    """
    h = image_height if image_height is not None else (geom.height if geom else 0)
    if geom is None:
        return 0, max(1, h // 2)

    horizon = geom.horizon_row()
    bottom = int(round(horizon + FAR_FIELD_MARGIN * h))
    # A camera tilted well down puts the horizon above the frame entirely,
    # leaving a negative row. Clamp to a usable band rather than an empty
    # one: what is left is still the most distant content available.
    bottom = max(int(0.15 * h), min(h, bottom))
    return 0, bottom


DEFAULT_HFOV_DEG = 62.2     # IMX219 standard lens, when no geom is supplied


def lens_params(geom, width: int):
    """(fx, cx, hfov_deg) from a CameraGeometry, or sane defaults."""
    if geom is not None:
        return geom.fx, geom.cx, geom.hfov_deg
    hfov = DEFAULT_HFOV_DEG
    fx = (width / 2.0) / math.tan(math.radians(hfov) / 2.0)
    return fx, (width - 1) / 2.0, hfov


def degrees_per_bin(geom, width: int) -> float:
    """Angular width of one signature bin — the conversion factor from a
    correlation shift to a yaw."""
    _, _, hfov = lens_params(geom, width)
    return hfov / (width - 1)


def column_signature(frame_rgb: np.ndarray, geom=None, band: tuple = None) -> np.ndarray:
    """
    Collapse the far-field band to one value per equal-angle bin.

    Two steps, both load-bearing:
      1. Mean down the band's rows -> one value per image COLUMN.
      2. Resample onto uniformly spaced BEARINGS, so that a camera
         rotation becomes a uniform shift (see the module docstring).

    Normalised to zero mean and unit norm so correlation scores compare
    across frames of different brightness — auto-exposure moves absolute
    levels a lot on real hardware and would otherwise swamp the peak.

    Returns None when the band has no usable contrast.
    """
    h, w = frame_rgb.shape[:2]
    top, bottom = band if band is not None else far_field_band(geom, h)
    top = max(0, min(h - 1, top))
    bottom = max(top + 1, min(h, bottom))

    by_column = _luminance(frame_rgb[top:bottom, :]).mean(axis=0)

    fx, cx, hfov = lens_params(geom, w)
    bearings = np.linspace(-hfov / 2.0, hfov / 2.0, w)
    source_u = cx + fx * np.tan(np.radians(bearings))
    # Bearings at the very edge can project just outside the sensor for a
    # slightly-off fx; clamping is better than dropping bins, which would
    # change the signature length and break correlation against a
    # reference taken with a different clamp.
    source_u = np.clip(source_u, 0, w - 1)
    sig = np.interp(source_u, np.arange(w), by_column)

    sig = sig - sig.mean()
    norm = float(np.sqrt((sig ** 2).sum()))
    if norm < 1e-6:
        return None              # perfectly flat band: nothing to align
    return sig / norm


def _best_shift(ref: np.ndarray, live: np.ndarray, max_shift: int):
    """
    Shift (in bins) that best aligns `live` onto `ref`, by normalised
    cross-correlation over the overlapping region, plus the peak score.

    A shift of s scores ref[i + s] against live[i], so it is high when
    live[i] == ref[i + s]: the feature that sat at i + s in the reference
    now sits at i. Its index DECREASED, so a POSITIVE shift means the
    content moved LEFT across the frame.

    Returns (shift_px, score). Scores are normalised per-overlap so that a
    wide, well-matched overlap is not beaten by a narrow accidental one.
    """
    n = len(ref)
    best_shift, best_score = 0, -2.0
    scores = {}

    for s in range(-max_shift, max_shift + 1):
        if s >= 0:
            a, b = ref[s:], live[:n - s] if s else live
        else:
            a, b = ref[:n + s], live[-s:]
        if len(a) < n * 0.35:
            continue                     # too little overlap to trust
        na = float(np.sqrt((a ** 2).sum()))
        nb = float(np.sqrt((b ** 2).sum()))
        if na < 1e-9 or nb < 1e-9:
            continue
        score = float((a * b).sum() / (na * nb))
        scores[s] = score
        if score > best_score:
            best_shift, best_score = s, score

    if best_score < -1.0:
        return None, 0.0, 0.0

    # How far the peak towers over the rest of the curve. Rivals close to
    # the peak belong to the same lobe and are excluded.
    exclusion = max(1, int(PROMINENCE_EXCLUSION * n))
    rivals = [v for s, v in scores.items() if abs(s - best_shift) > exclusion]
    runner_up = max(rivals) if rivals else 0.0
    prominence = (best_score / runner_up) if runner_up > 1e-6 else float("inf")

    # Parabolic refinement: the true peak rarely lands exactly on an
    # integer bin, and rounding to one puts a floor on angular resolution
    # that shows up as a persistent bias once integrated.
    y0 = scores.get(best_shift - 1)
    y1 = scores.get(best_shift)
    y2 = scores.get(best_shift + 1)
    sub = float(best_shift)
    if y0 is not None and y2 is not None:
        denom = y0 - 2 * y1 + y2
        if abs(denom) > 1e-9:
            offset = 0.5 * (y0 - y2) / denom
            if -1.0 < offset < 1.0:
                sub = best_shift + offset

    return sub, best_score, prominence


def yaw_between(ref_sig: np.ndarray, live_sig: np.ndarray,
                deg_per_bin: float, max_shift: int = None,
                min_prominence: float = None):
    """
    Yaw change in DEGREES that carries the reference view to the live view,
    or None if the match is not trustworthy.

    Both signatures must come from column_signature(), i.e. already
    resampled to equal-angle bins, so the conversion is one multiply.

    Sign follows the rest of jetbot_nav: positive = the robot turned RIGHT.
    Turning right sweeps the scene LEFT across the frame, and _best_shift
    returns a POSITIVE shift for leftward movement (see its docstring), so
    the two conventions already agree and no negation is needed. This is
    the single easiest thing to get backwards here — it was in fact
    written backwards first — so test_heading.py pins both directions.
    """
    if ref_sig is None or live_sig is None:
        return None
    n = len(ref_sig)
    if len(live_sig) != n:
        raise ValueError("signatures must come from the same image width")

    if max_shift is None:
        max_shift = int(n * MAX_SHIFT_FRACTION)

    shift, score, prominence = _best_shift(ref_sig, live_sig, max_shift)
    if shift is None:
        return None
    # Railed against the edge of the search window: the correlator ran out
    # of room, so this is a lower bound rather than a measurement. Checked
    # BEFORE the score gates because it is a structural fact about where
    # the peak landed, not a question of how good it looks — and a railed
    # peak looks good (see RAIL_MARGIN_BINS).
    if abs(shift) >= max_shift - RAIL_MARGIN_BINS:
        return None
    # Both gates, deliberately. The absolute score catches a featureless
    # view; the prominence ratio catches an out-of-range or aliased match,
    # which can score respectably while meaning nothing.
    #
    # The prominence bar depends on WHAT is being compared, not on how
    # much risk the caller feels like taking: a near-identical pair of
    # consecutive frames cannot produce the peak sharpness that a
    # reference photo taken degrees away does. See
    # GYRO_MIN_PEAK_PROMINENCE.
    floor = MIN_PEAK_PROMINENCE if min_prominence is None else min_prominence
    if score < MIN_CONFIDENCE or prominence < floor:
        return None
    return shift * deg_per_bin


MIN_CORRECTABLE_DEPTH = 1.5   # below this the correction is not applied.
                        # The factor blows up as depth falls (at 0.8 units it
                        # would halve every reading), and a depth that small
                        # means the view is filled by one close object, which
                        # is both the least reliable input to the estimate and
                        # the case where a big multiplicative correction does
                        # the most damage if the depth is wrong.

MAX_LEVER_GAIN = 1.6    # never divide by more than this, whatever the depth.
                        # A guard, not a tuning knob: a correction that can
                        # scale a reading arbitrarily is a new way to be
                        # confidently wrong, which is the failure this module
                        # exists to avoid.


def lever_arm_gain(geom, depth: float) -> float:
    """
    How much larger than the true yaw a correlation reading will be, for a
    camera mounted `geom.pivot_offset` ahead of the robot's turning centre
    looking at content `depth` away.

    Turning by t swings the camera sideways by about pivot_offset * t, and
    that sideways motion shifts content at distance d by pivot_offset * t / d
    in bearing. The camera sees the sum, so:

        apparent = true * (1 + pivot_offset / depth)

    Returns 1.0 (no correction) when the geometry does not say where the
    camera sits, when the depth is unknown, or when the depth is too small
    to correct against — see MIN_CORRECTABLE_DEPTH.
    """
    if geom is None or depth is None:
        return 1.0
    offset = getattr(geom, "pivot_offset", 0.0)
    if offset <= 0.0 or depth < MIN_CORRECTABLE_DEPTH:
        return 1.0
    return min(1.0 + offset / float(depth), MAX_LEVER_GAIN)


def measurable_range_deg(geom=None, width: int = None) -> float:
    """
    Largest yaw magnitude that can be measured at all, in degrees. Beyond
    this the two views no longer overlap enough to align and both
    yaw_between() and CourseLock.error_deg() return None rather than a
    number — deliberately, since the alternative is a confident lie.

    Excludes the boundary bins RAIL_MARGIN_BINS rejects, so this stays the
    range that can actually be MEASURED rather than merely searched. The
    difference is a fraction of a degree, but reporting the searched span
    here would document a capability the function refuses to deliver.
    """
    w = width if width is not None else (geom.width if geom else 640)
    usable = max(0, int(w * MAX_SHIFT_FRACTION) - RAIL_MARGIN_BINS)
    return usable * degrees_per_bin(geom, w)


# ─── Continuous estimate ─────────────────────────────────────────────────────

class VisualGyro:
    """
    Accumulates yaw frame to frame. Relative only — there is no absolute
    reference, so heading_deg is "degrees turned since reset()", which is
    all a return-to-course manoeuvre needs.

    Drifts, by construction. Use CourseLock to close out a turn.
    """

    def __init__(self, geom=None, band: tuple = None):
        self.geom = geom
        self.band = band
        self.heading_deg = 0.0
        self.lost_frames = 0        # consecutive frames with no usable match
        self._prev_sig = None

    def reset(self):
        self.heading_deg = 0.0
        self.lost_frames = 0
        self._prev_sig = None

    def update(self, frame_rgb: np.ndarray, depth: float = None):
        """
        Feed one frame. Returns the yaw delta in degrees since the previous
        frame, or None if this frame could not be matched.

        `depth` is roughly how far away the scene in view is, in the same
        units the caller's scan reports. Supplying it corrects the
        over-read caused by the camera sitting ahead of the turning centre
        (see lever_arm_gain); omitting it leaves the reading uncorrected
        rather than guessing at a distance. It matters here more than
        anywhere else in this module, because this is the one estimate
        that ACCUMULATES: a steady few percent is invisible per tick and
        unbounded over a drive.

        On None the accumulated heading is left UNCHANGED rather than
        guessed at — a gap in the record is recoverable, an invented delta
        is not. Callers should watch lost_frames: a run of them means the
        accumulated heading is quietly going stale.

        Tracking resumes on the very next frame: a failed match still
        re-baselines the reference, so the damage is a permanent OFFSET in
        the accumulated total, never a permanent loss of the ability to
        measure. Closing that offset is CourseLock's job.
        """
        sig = column_signature(frame_rgb, self.geom, self.band)
        if sig is None:
            self.lost_frames += 1
            return None

        if self._prev_sig is None:
            self._prev_sig = sig
            self.lost_frames = 0
            return 0.0

        delta = yaw_between(self._prev_sig, sig,
                            degrees_per_bin(self.geom, frame_rgb.shape[1]),
                            min_prominence=GYRO_MIN_PEAK_PROMINENCE)
        if delta is None:
            self.lost_frames += 1
            # Re-baseline onto the frame we could not match FROM. The
            # rotation across this step is gone for good — that is what
            # lost_frames exists to announce — but keeping the old
            # reference would compare the next frame against an
            # ever-more-distant view, so a single unmeasurable step
            # becomes a permanent blackout instead of a one-tick gap.
            #
            # Measured: one 54 deg jump used to silence the gyro for the
            # whole remaining run even though every later step was 2 deg.
            # In the first graded live run that was 133 of 164 ticks with
            # no heading at all, which reads as "course keeping is off"
            # rather than as a failure.
            self._prev_sig = sig
            return None

        delta /= lever_arm_gain(self.geom, depth)

        self.heading_deg += delta
        self.lost_frames = 0
        self._prev_sig = sig
        return delta


# ─── Snapshot re-acquisition ─────────────────────────────────────────────────

class CourseLock:
    """
    Remembers the view along the original course and reports how far the
    robot has since turned away from it.

    Because every measurement compares the live frame against the ORIGINAL
    photograph rather than the previous frame, the error does not
    accumulate — this is what bounds VisualGyro's drift at the one moment
    accuracy actually matters, namely finishing the turn back onto course.

    Typical use:

        lock = CourseLock(geom)
        lock.capture(frame)              # about to detour
        ...                              # gap_follow does its thing
        err = lock.error_deg(frame)      # + means turned right of course
        while err is not None and not lock.on_course(err):
            ...turn by -sign(err)...
    """

    def __init__(self, geom=None, band: tuple = None):
        self.geom = geom
        self.band = band
        self._ref_sig = None

    @property
    def armed(self) -> bool:
        return self._ref_sig is not None

    def capture(self, frame_rgb: np.ndarray) -> bool:
        """
        Photograph the current view as the course reference. Returns False
        if the view has too little texture to be usable — the caller must
        check, because an unarmed lock silently reports no error at all,
        which is indistinguishable from being perfectly on course.
        """
        sig = column_signature(frame_rgb, self.geom, self.band)
        self._ref_sig = sig
        return sig is not None

    def clear(self):
        self._ref_sig = None

    def error_deg(self, frame_rgb: np.ndarray, depth: float = None):
        """
        Degrees the robot has turned away from the captured course:
        POSITIVE means it is now pointing right of it, so it must turn
        LEFT to recover. None if not armed, or if this frame cannot be
        matched to the reference (turned so far the views no longer
        overlap, or looking at something featureless).

        `depth` applies the same lever-arm correction VisualGyro uses. It
        is a weaker approximation here: this compares against a frame the
        robot may have driven a long way from, so the difference between
        the two views is rotation AND translation, while the correction
        only models the sideways swing of a turn. It is still the right
        sign and roughly the right size, and unlike the gyro a mistake
        here does not accumulate — each reading is independent of the
        last.
        """
        if self._ref_sig is None:
            return None
        live = column_signature(frame_rgb, self.geom, self.band)
        raw = yaw_between(self._ref_sig, live,
                          degrees_per_bin(self.geom, frame_rgb.shape[1]))
        if raw is None:
            return None
        return raw / lever_arm_gain(self.geom, depth)

    @staticmethod
    def on_course(error_deg: float, tolerance: float = ARRIVED_DEG) -> bool:
        return error_deg is not None and abs(error_deg) <= tolerance
