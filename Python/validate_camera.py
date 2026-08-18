"""
validate_camera.py — check the camera pipeline against Unity ground truth.

Everything in jetbot_nav.visual_scan and jetbot_nav.heading has so far
been verified only against SYNTHETIC frames. Synthetic frames confirm the
geometry, which is the part most likely to be silently wrong, but they
cannot answer the questions that decide whether any of it works in
practice:

  1. Does the floor actually segment against the scene's real lighting,
     shading and shadows?
  2. Do visual_scan's distances agree with the true geometry?
  3. Is there enough texture around the horizon for heading to lock onto,
     or is the sky a smooth gradient with nothing to correlate?

This harness answers all three against the running simulation, using
ProximitySensor and set_rotation as ground truth. Both are Unity-only and
have no hardware equivalent — which is exactly why they belong in a
validation tool and nowhere near the navigation code.

REQUIRES Unity playing, with RobotCamera and ProximitySensor in the scene
(Tools > Setup Robot Simulator Scene adds both).

    py -3.8 validate_camera.py            run every check
    py -3.8 validate_camera.py --save     also write annotated PNGs
"""

import math
import sys

import numpy as np

import sim_client
from jetbot_nav import visual_scan, heading
from jetbot_nav.visual_scan import CameraGeometry

SAVE = "--save" in sys.argv

_results = []


def check(label, ok, detail=""):
    _results.append(bool(ok))
    mark = "PASS" if ok else "FAIL"
    print(f"{mark}  {label}" + (f"\n        {detail}" if detail else ""))


def warn(label, detail=""):
    print(f"WARN  {label}" + (f"\n        {detail}" if detail else ""))


def grab(camera):
    import jetson_utils
    img = camera.Capture()
    if img is None:
        return None
    return jetson_utils.cudaToNumpy(img)


# ─── 0. Connect ──────────────────────────────────────────────────────────────

print(__doc__.strip().splitlines()[0])
print()

if not sim_client.connect():
    print("FATAL  Could not connect to Unity. Press Play first, and check the "
          "Console shows both TcpServer (5555) and SimQueryServer (5556).")
    sys.exit(2)

import jetson_utils
camera = jetson_utils.videoSource("csi://0")

frame = grab(camera)
if frame is None:
    print("FATAL  Connected, but no frame came back. Is SimCamera attached to "
          "RobotCamera? Run Tools > Setup Robot Simulator Scene.")
    sys.exit(2)

H, W = frame.shape[:2]
print(f"frame: {W}x{H}")

truth = sim_client.send_query({"command": "get_proximity_scan"})
have_truth = bool(truth) and truth.get("status") == "ok"
if not have_truth:
    warn("ProximitySensor did not answer get_proximity_scan",
         "Distance accuracy cannot be checked. Attach ProximitySensor to the "
         "robot (it is dev-only ground truth, not a deployment sensor).")

geom = visual_scan.sim_jetank(width=W, height=H)
print(f"geometry: height {geom.height_m}, tilt {geom.tilt_deg} deg, "
      f"lens {geom.hfov_deg}x{geom.vfov_deg} deg")
print(f"          horizon row {geom.horizon_row():.0f}, "
      f"ground fov {geom.ground_fov_deg():.1f} deg, "
      f"blind inside {geom.min_visible_range():.2f} units")
print()


# ─── 1. The frame is a real render ───────────────────────────────────────────

check("frame size matches CameraGeometry",
      (W, H) == (geom.width, geom.height),
      f"frame {W}x{H} vs geometry {geom.width}x{geom.height}")

spread = float(frame.std())
check("frame has real image content (not black or a flat fill)",
      spread > 8.0, f"pixel std {spread:.1f}")


# ─── 1b. Is this actually the robot's camera? ────────────────────────────────
# Everything downstream converts pixels to metres using CameraGeometry, so
# a frame from a DIFFERENT camera produces a full page of confident,
# meaningless numbers rather than an error. That is not hypothetical: the
# first run of this harness measured the world-fixed overview camera,
# because SimCamera had not yet been moved onto RobotCamera, and reported
# the robot's own chassis as an obstacle 1.0 units ahead.
#
# The sky/ground boundary is a strong horizontal edge, so its row can be
# recovered and compared against the row the geometry predicts. They only
# agree when the frame really does come from a camera mounted and aimed
# the way CameraGeometry says.

def detect_horizon_row(img):
    """Row of the strongest horizontal edge, i.e. the sky/ground boundary."""
    lum = (0.299 * img[:, :, 0] + 0.587 * img[:, :, 1]
           + 0.114 * img[:, :, 2]).astype(np.float32)
    profile = lum.mean(axis=1)
    grad = np.abs(np.diff(profile))
    if grad.size == 0 or float(grad.max()) < 1.0:
        return None                       # no clear boundary in view
    return int(np.argmax(grad))


predicted = geom.horizon_row()
observed = detect_horizon_row(frame)

if observed is None:
    warn("no clear horizon in view, cannot confirm which camera this is",
         "Point the robot somewhere the sky/ground boundary is visible if the "
         "numbers below look wrong.")
else:
    agree = abs(observed - predicted) <= 40
    check("the frame comes from a camera matching CameraGeometry",
          agree,
          f"horizon observed at row {observed}, geometry predicts "
          f"{predicted:.0f}"
          + ("" if agree else
             "\n        This frame is NOT from the robot-mounted camera. Run "
             "Tools > Setup Robot Simulator Scene in Unity to create "
             "RobotCamera\n        and move SimCamera onto it, then re-run. "
             "Every distance and heading below is measured against the wrong "
             "optics until you do."))
    if not agree:
        print("\nSkipping the remaining checks — they would only produce "
              "confident nonsense against the wrong camera.")
        sim_client.disconnect()
        passed = sum(_results)
        print(f"\n{passed}/{len(_results)} checks passed (stopped early)")
        sys.exit(1)


# ─── 2. Floor segmentation ───────────────────────────────────────────────────

mask = visual_scan.floor_mask(frame)
floor_fraction = float(mask.mean())

below_horizon = mask[max(0, int(geom.horizon_row())):, :]
below_fraction = float(below_horizon.mean()) if below_horizon.size else 0.0

check("floor segmentation finds a plausible amount of floor",
      0.10 < below_fraction < 0.99,
      f"{below_fraction * 100:.0f}% of the sub-horizon image reads as floor "
      f"({floor_fraction * 100:.0f}% of the whole frame)")

if below_fraction > 0.985:
    warn("almost everything below the horizon reads as floor",
         "Either the view is genuinely empty, or obstacles are being absorbed "
         "into the floor colour. Drive somewhere with an obstacle in view and "
         "re-run before trusting the distances below.")


# ─── 3. visual_scan against ProximitySensor ──────────────────────────────────

scan = visual_scan.free_space_scan(frame, geom, n_rays=13)
print()
print("visual_scan:", " ".join(f"{d:5.1f}" for d in scan["distances"]))

if have_truth:
    t_fov = float(truth["fov"])
    t_n = int(truth["count"])
    t_step = t_fov / (t_n - 1)
    t_ang = [-t_fov / 2 + i * t_step for i in range(t_n)]
    t_dist = [float(d) for d in truth["distances"]]
    print("ground truth:", " ".join(f"{d:5.1f}" for d in t_dist))

    def truth_at(bearing):
        """Nearest ground-truth ray to a bearing, or None if out of its fan."""
        best_i, best_d = None, None
        for i, a in enumerate(t_ang):
            diff = abs(a - bearing)
            if best_d is None or diff < best_d:
                best_i, best_d = i, diff
        if best_d is None or best_d > t_step:
            return None
        return t_dist[best_i]

    print()
    print("  bearing |  visual | truth  | delta   note")
    errors = []
    for a, d in zip(scan["angles_deg"], scan["distances"]):
        gt = truth_at(a)
        if gt is None:
            print(f"   {a:+6.1f} | {d:6.2f} |   --   |         outside sensor fan")
            continue
        # Both saturate at max_range; comparing "nothing there" to
        # "nothing there" says nothing about accuracy.
        if d >= scan["max_range"] - 0.01 and gt >= float(truth["max_range"]) - 0.01:
            print(f"   {a:+6.1f} | {d:6.2f} | {gt:6.2f} |         both clear")
            continue
        blind = gt < geom.min_visible_range()
        delta = d - gt
        note = "INSIDE BLIND ZONE (expected miss)" if blind else ""
        if not blind:
            errors.append(abs(delta))
        print(f"   {a:+6.1f} | {d:6.2f} | {gt:6.2f} | {delta:+6.2f}  {note}")

    if errors:
        worst, mean = max(errors), sum(errors) / len(errors)
        check("visual distances track ground truth outside the blind zone",
              mean < 1.5,
              f"mean |error| {mean:.2f}, worst {worst:.2f} over "
              f"{len(errors)} comparable rays")
    else:
        warn("no rays were comparable",
             "Nothing in view is both visible and within range. Drive toward "
             "an obstacle and re-run — this check proves nothing as it stands.")


# ─── 4. Heading: is there anything to lock onto? ─────────────────────────────

print()
top, bottom = heading.far_field_band(geom, H)
band = frame[top:bottom, :]
print(f"far-field band: rows {top}-{bottom} ({bottom - top} rows), "
      f"pixel std {band.std():.1f}")

sig = heading.column_signature(frame, geom)
check("far-field band has enough texture for a heading lock",
      sig is not None,
      "The band around the horizon is featureless — a smooth skybox with no "
      "objects breaking it. heading will return None every frame until "
      "something textured is in view." if sig is None else
      f"signature spans {float(sig.max() - sig.min()):.3f} normalised units")


# ─── 5. Heading against a commanded rotation ─────────────────────────────────
# set_rotation teleports the heading, so the truth is exact. Unity-only,
# which is fine here and would not be fine in the navigation code.

if sig is not None:
    print()
    print("rotation test (set_rotation is ground truth):")
    import time

    def face(deg):
        sim_client.send_command({"command": "set_rotation", "rotation_y": deg})
        time.sleep(0.45)          # let Unity render and SimCamera re-cache
        return grab(camera)

    base = face(0.0)
    lock = heading.CourseLock(geom)
    armed = lock.capture(base)
    check("CourseLock arms on a real frame", armed)

    if armed:
        # Raw score/prominence alongside the verdict. A bare "None" cannot
        # distinguish "nothing to correlate" from "rejected by a gate that
        # is too strict for this scene", and those need opposite fixes —
        # one is scene-side, the other is a threshold.
        from jetbot_nav.heading import (
            _best_shift, MIN_CONFIDENCE, MIN_PEAK_PROMINENCE,
            MAX_SHIFT_FRACTION,
        )
        ref_sig = heading.column_signature(base, geom)
        dpb = heading.degrees_per_bin(geom, W)
        max_shift = int(W * MAX_SHIFT_FRACTION)

        print("     commanded |  measured | error  |  score  promin  gate")
        rot_err = []
        for truth_deg in [-20.0, -10.0, -5.0, 5.0, 10.0, 20.0]:
            f = face(truth_deg)
            est = lock.error_deg(f) if f is not None else None

            live_sig = heading.column_signature(f, geom) if f is not None else None
            if live_sig is None:
                score = prom = float("nan")
                gate = "no signature"
            else:
                _, score, prom = _best_shift(ref_sig, live_sig, max_shift)
                if score < MIN_CONFIDENCE:
                    gate = "score"
                elif prom < MIN_PEAK_PROMINENCE:
                    gate = "PROMINENCE"
                else:
                    gate = "ok"

            if est is None:
                print(f"      {truth_deg:+8.1f} |    (none) |        | "
                      f"{score:6.3f}  {prom:6.2f}  {gate}")
                rot_err.append(None)
            else:
                print(f"      {truth_deg:+8.1f} | {est:+9.2f} | "
                      f"{est - truth_deg:+6.2f} | {score:6.3f}  {prom:6.2f}  {gate}")
                rot_err.append(est - truth_deg)

        good = [e for e in rot_err if e is not None]
        check("heading measures commanded rotations on real frames",
              len(good) >= 4 and max(abs(e) for e in good) < 5.0,
              f"{len(good)}/6 measured, worst error "
              f"{max((abs(e) for e in good), default=float('nan')):.2f} deg")

        measured = [(t, r) for t, r in
                    zip([-20.0, -10.0, -5.0, 5.0, 10.0, 20.0], rot_err)
                    if r is not None]
        check("heading sign matches the commanded direction",
              all((t > 0) == ((t + r) > 0) for t, r in measured) if measured else False,
              "A sign error here turns the robot AWAY from its course.")

    face(0.0)     # leave the robot as we found it


# ─── 6. Optional annotated dumps ─────────────────────────────────────────────

if SAVE:
    import cv2
    vis = visual_scan.annotate(frame, geom, scan, mask=mask)
    cv2.imwrite("validate_scan.png", cv2.cvtColor(vis, cv2.COLOR_RGB2BGR))
    band_img = frame.copy()
    band_img[top:bottom, :, 1] = 255      # tint the heading band green
    cv2.imwrite("validate_band.png", cv2.cvtColor(band_img, cv2.COLOR_RGB2BGR))
    print("\nwrote validate_scan.png and validate_band.png")


# ─── Summary ─────────────────────────────────────────────────────────────────

sim_client.disconnect()
passed = sum(_results)
print(f"\n{passed}/{len(_results)} checks passed")
if passed != len(_results):
    print("\nA failure here means the pipeline does not work against real "
          "renders yet — the offline suites cannot catch that. Fix before "
          "building the course-keeping loop on top.")
sys.exit(0 if passed == len(_results) else 1)
