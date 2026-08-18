"""
jetbot_nav.visual_scan — free-space distance scan derived from a camera frame.

WHY THIS EXISTS: THE PROXIMITY SENSOR WAS NOT REAL HARDWARE
────────────────────────────────────────────────────────────────
gap_follow was originally fed by Unity's ProximitySensor, which casts 13
rays simultaneously every frame — that is a 2D lidar, and the JETANK does
not have one. It has a camera. The old ProximitySensor docstring waved at
"an ultrasonic swept by a servo" as the hardware equivalent, but that
substitution does not survive contact:

  - SIMULTANEITY. A swept sensor returns its readings sequentially across
    the sweep (~0.5-2 s), during which the robot has moved AND rotated.
    gap_follow polls at 10 Hz assuming each tick is one coherent
    instantaneous snapshot, and its debounce counters are measured in
    ticks — at real sweep rates those silently become multi-second delays.
  - BEAM SHAPE. An ultrasonic returns the nearest echo anywhere in a
    ~15-30 deg cone. Physics.Raycast is infinitely thin. That difference
    is exactly why a thin pole is separable from a wall in simulation and
    would not be on hardware.

So the scan is computed HERE instead, in Python, from the camera image.
Unity's only job is to hand over a rendered frame — which it already does
honestly. The identical code then runs on the Jetson against the identical
frame shape from the real camera. There is no simulation-only sensor left
in the loop, which is the whole point.

Critically, gap_follow does NOT change: it consumes
{"angles_deg", "distances", "max_range"} and does not care where the
numbers came from. free_space_scan() returns exactly that shape, so the
follow-the-gap controller, target_seek, and their regression suites stay
valid on top of a completely different sensing front-end.

HOW IT WORKS: GROUND-PLANE PROJECTION
────────────────────────────────────────────────────────────────
On a flat floor, a pixel's position in the image determines the distance
to the ground point it sees — lower in the frame means nearer. So:

  1. Segment which pixels are floor (see floor_mask).
  2. For each image column, walk UP from the bottom. The first non-floor
     pixel is where an object's base occludes the floor: that pixel's
     ground point is how far away the obstacle is in that direction.
  3. Project that pixel through the camera model to get (bearing, range).
  4. Bin the per-column results into N rays and keep the nearest hit in
     each bin — nearest-wins, because a scan is a safety input.

WHAT THIS INHERENTLY CANNOT DO (read before trusting it)
────────────────────────────────────────────────────────────────
  - NEAR BLIND ZONE. The bottom row of the frame still looks some
    distance ahead, so anything nearer than min_visible_range() is not
    seen at all — and reads as open floor, not as an error. With the
    simulated mounting that radius is ~0.8 units, against gap_follow's
    EMERGENCY_FORWARD of 1.6. A range sensor has no equivalent limit,
    which is why ProximitySensor never surfaced this. See
    min_visible_range() for the full argument and the mitigations.
  - NO PERIPHERAL VISION. A 62 deg camera cannot see beside the robot the
    way a 120 deg fan could. gap_follow's corridor-clearance logic — the
    thing that fixed wall oscillation — was tuned assuming it could. This
    is a real capability loss, and it is the honest one: the physical
    robot cannot see sideways either.
  - FLAT FLOOR ASSUMED. A ramp, a step, or a lip reads as the wrong
    distance, not as an error. Overhangs (a table edge with clear space
    beneath) are invisible: nothing occludes the floor, so the column
    reads clear and the robot drives into the underside.
  - FLOOR-COLOURED OBSTACLES are invisible for the same reason
    perception.py cannot see a red block on a red mat.

CALIBRATION (both simulation and hardware)
────────────────────────────────────────────────────────────────
CameraGeometry needs the camera's height above the floor, its downward
tilt, and its FOV. Height and tilt are physical measurements — take them
once with a ruler and a phone level; every distance this module reports
scales off them, so a 20% height error is a 20% range error. FOV comes
from the lens datasheet. IMX219_STANDARD below matches the Raspberry Pi
Camera v2 sensor used on the JETANK.

TESTING
    py -3.8 test_visual_scan.py    — offline, synthetic frames, no Unity
"""

import math

import numpy as np

# ─── Lens presets ────────────────────────────────────────────────────────────
# (horizontal FOV, vertical FOV) in degrees, for a near-linear (pinhole)
# lens. A fisheye such as the IMX219-160 is NOT usable here directly: its
# barrel distortion breaks the straight-line pixel->angle mapping every
# projection below depends on, so it must be undistorted first.

IMX219_STANDARD = (62.2, 48.8)   # Raspberry Pi Camera v2 / JETANK stock lens


def sim_jetank(width: int = 640, height: int = 480) -> "CameraGeometry":
    """
    Geometry of the simulated RobotCamera, mirroring the constants in
    SceneSetup.cs (CamHeight / CamTiltDeg / CamFovDeg). If you move the
    camera in Unity, change these together — a mismatch produces distances
    that are wrong by a constant factor while still looking plausible,
    which is the hardest kind of error to notice.

    NOTE ON SCALE: the Unity scene is not built at the physical JETANK's
    scale (its obstacles are 1-unit cubes and the chassis is ~2 units
    long, against a real JETANK about 0.2 m long). Distances here are
    therefore in SIM units, and gap_follow's thresholds were tuned in
    those units. Deploying to hardware needs a real measurement of camera
    height and tilt AND a re-tune of gap_follow's distance constants — the
    geometry code is scale-free, but the tuning is not.
    """
    return CameraGeometry(height_m=0.80, tilt_deg=20.0,
                          hfov_deg=IMX219_STANDARD[0],
                          vfov_deg=IMX219_STANDARD[1],
                          width=width, height=height)


class CameraGeometry:
    """
    Pinhole camera mounted at a fixed height, pitched down by `tilt_deg`.

    Angle convention matches gap_follow's scan: bearings in DEGREES,
    negative = left of forward, positive = right.
    """

    def __init__(self, height_m: float, tilt_deg: float,
                 hfov_deg: float = IMX219_STANDARD[0],
                 vfov_deg: float = IMX219_STANDARD[1],
                 width: int = 640, height: int = 480):
        if height_m <= 0:
            raise ValueError("height_m must be positive (camera above the floor)")
        if not 0 < tilt_deg < 90:
            # 0 = looking at the horizon: no ray ever meets the floor, so
            # every distance would be undefined. 90 = straight down.
            raise ValueError("tilt_deg must be between 0 and 90 exclusive")

        self.height_m = float(height_m)
        self.tilt_deg = float(tilt_deg)
        self.hfov_deg = float(hfov_deg)
        self.vfov_deg = float(vfov_deg)
        self.width = int(width)
        self.height = int(height)

        # Focal lengths in pixels, from the FOV each axis subtends.
        self.fx = (self.width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
        self.fy = (self.height / 2.0) / math.tan(math.radians(vfov_deg) / 2.0)
        self.cx = (self.width - 1) / 2.0
        self.cy = (self.height - 1) / 2.0

    def pixel_to_ground(self, u: float, v: float):
        """
        Project one pixel onto the floor plane.

        Returns (bearing_deg, distance_m), or None when the pixel's ray
        never meets the floor ahead — i.e. it points at or above the
        horizon, or behind the camera. Callers must treat None as "no
        information", NOT as "clear".
        """
        # Ray in camera axes: x right, y down, z forward.
        dx = (u - self.cx) / self.fx
        dy = (v - self.cy) / self.fy

        # Re-express with Y up, then pitch the camera down by tilt so +Z
        # tips toward the floor.
        t = math.radians(self.tilt_deg)
        cos_t, sin_t = math.cos(t), math.sin(t)

        X = dx
        Y = -dy * cos_t - sin_t          # world-up component of the ray
        Z = -dy * sin_t + cos_t          # world-forward component

        if Y >= 0:
            return None                   # at or above the horizon
        if Z <= 0:
            return None                   # behind the camera

        # Travel along the ray until it drops to floor level (-height).
        s = -self.height_m / Y
        ground_x = X * s
        ground_z = Z * s

        distance = math.hypot(ground_x, ground_z)
        bearing = math.degrees(math.atan2(ground_x, ground_z))
        return bearing, distance

    def horizon_row(self) -> float:
        """
        Image row where the floor plane vanishes. Everything above this is
        sky/wall and carries no floor distance information at all.

        The horizon sits `tilt_deg` ABOVE the optical axis (the axis points
        that far below horizontal), and "above the axis" means a smaller
        row index — hence the subtraction. Returns a negative row when the
        camera is tilted down far enough that the horizon is off the top of
        the frame, which is the normal case for a floor-facing robot
        camera; callers should clamp to 0 rather than treat it as invalid.
        """
        return self.cy - self.fy * math.tan(math.radians(self.tilt_deg))

    def ground_fov_deg(self) -> float:
        """
        Full angular span the camera actually covers ON THE GROUND, which
        is WIDER than hfov_deg whenever the camera is tilted down.

        A tilted camera's bottom corners look steeply down and out to the
        side, so their ground points sit at bearings beyond half the pixel
        FOV — at 30 deg tilt with a 62 deg lens the corners reach about
        +/-43 deg. Binning a scan over +/-hfov/2 would therefore push every
        near-edge obstacle outside the bin range and silently drop it,
        which on a safety input is the worst possible rounding error.

        Measured at the bottom row, where the fan is widest.
        """
        corner = self.pixel_to_ground(0.0, self.height - 1)
        if corner is None:
            # Bottom row above the horizon — only possible at a tilt so
            # shallow the camera sees essentially no floor. Fall back to
            # the lens FOV rather than raising: the scan will be all
            # max_range anyway.
            return self.hfov_deg
        return 2.0 * abs(corner[0])

    def min_visible_range(self) -> float:
        """
        Nearest ground distance the camera can see, from the bottom row of
        the frame. THERE IS A BLIND ZONE INSIDE THIS RADIUS.

        This is the single most important number to check against
        gap_follow's tuning. An obstacle closer than this is not "far
        away" to the scan — it is absent, indistinguishable from open
        floor, because nothing occludes the floor in any visible row.
        The robot is nearest to things precisely during BACKUP and PIVOT,
        which is exactly when the blind zone is most likely to swallow the
        obstacle being recovered from.

        A range sensor has no such limit, which is why the old
        ProximitySensor never exposed this problem. Mitigations, in order
        of preference: tilt the camera further down (shrinks the blind
        zone, costs forward range), mount it higher, or keep
        EMERGENCY_FORWARD comfortably above this value so the controller
        never willingly approaches the edge of its own vision.
        """
        nearest = self.pixel_to_ground(self.cx, self.height - 1)
        return nearest[1] if nearest else float("inf")


# ─── Floor segmentation ──────────────────────────────────────────────────────

def floor_mask(frame_rgb: np.ndarray,
               sample_rows: float = 0.12,
               sample_cols: float = 0.4,
               hue_tol: int = 12,
               shadow_hue_tol: int = 35,
               shadow_val_drop: int = 25,
               sat_tol: int = 70,
               val_drop_tol: int = 130,
               val_rise_tol: int = 60,
               ignore_bottom_rows: int = 0) -> np.ndarray:
    """
    Boolean mask of "this pixel looks like floor".

    The floor's colour is SAMPLED from the frame rather than hardcoded: a
    patch at the bottom-centre is the ground immediately in front of the
    robot, which is the one region that is floor essentially by
    construction (if the robot is already driving on it). Sampling makes
    this adapt to lighting and floor colour without recalibration, which
    matters far more on hardware than in the sim's fixed lighting.

    `ignore_bottom_rows` skips rows showing the robot's own chassis — on
    the real JETANK the front of the body intrudes into the bottom of the
    frame, and sampling it would make the ROBOT the reference "floor"
    colour and mark the actual floor as an obstacle.

    SHADOWS ARE THE HARD PART. Left unhandled they become phantom
    obstacles: the shadowed floor drops out of the mask, the column walk
    stops at the shadow's leading edge, and the robot reports something
    solid exactly where its own shadow falls — measured on a real render
    as a 0.97-unit return while the range sensor saw open ground to 12.
    Obstacle shadows do the same at whatever distance the obstacle is.

    A shadow is NOT simply the floor colour turned down. It is lit by a
    different light source — ambient sky rather than the sun — so its hue
    shifts as well. On the real renders here the floor sits at hue 56 and
    its shadows at 84-85, a hue distance of ~28, which sails past any
    tolerance tight enough to reject an actual obstacle. Widening the hue
    tolerance globally to cover that would also swallow real objects, so
    the extra latitude is granted ONLY to pixels that are also darker than
    the floor, which is what makes something a shadow rather than a
    differently-coloured thing sitting in the light.

    The two populations separate cleanly enough to tune against. Measured
    over the excluded pixels of a real frame:

        darker than floor (shadows):   hue distance ~17-43, median 28
        not darker (obstacles, sky):   hue distance ~24-58, median 56

    Hence shadow_hue_tol 35 for darkened pixels and hue_tol 12 otherwise;
    35 sits on a plateau, with 40 giving identical results.

    The residual trade is that a genuinely dark, floor-HUED obstacle can
    be absorbed into the floor. That is unavoidable for a colour method
    and is the same limitation that stops perception.py seeing a red block
    on a red mat.

    Assumes a fairly uniform floor. On patterned or heavily reflective
    surfaces, swap this function out — every other function in this
    module takes the mask as input and does not care how it was produced.
    """
    import cv2

    if frame_rgb.ndim != 3 or frame_rgb.shape[2] != 3:
        raise ValueError(f"expected (H, W, 3) RGB frame, got {frame_rgb.shape}")

    h, w = frame_rgb.shape[:2]
    usable_h = h - ignore_bottom_rows
    if usable_h < 2:
        raise ValueError("ignore_bottom_rows leaves no usable image")

    bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    # Reference patch: bottom-centre of the usable region.
    r0 = max(0, usable_h - int(usable_h * sample_rows))
    r1 = usable_h
    half = int(w * sample_cols / 2)
    c0, c1 = max(0, w // 2 - half), min(w, w // 2 + half)
    patch = hsv[r0:r1, c0:c1].reshape(-1, 3)

    # Median, not mean: a few obstacle pixels intruding into the sample
    # patch would drag a mean off the true floor colour, while the median
    # ignores them as long as they are the minority.
    ref = np.median(patch, axis=0)

    hue_diff = np.abs(hsv[:, :, 0].astype(np.int16) - int(ref[0]))
    hue_diff = np.minimum(hue_diff, 180 - hue_diff)   # hue is circular
    sat_diff = np.abs(hsv[:, :, 1].astype(np.int16) - int(ref[1]))

    # Signed, not absolute: positive means DARKER than the reference floor,
    # which is what a shadow looks like and must stay in the mask.
    val_drop = int(ref[2]) - hsv[:, :, 2].astype(np.int16)

    # Only darkened pixels get the wider hue latitude. A bright pixel that
    # is off-hue is an object, not a shadow.
    shadowy = val_drop > shadow_val_drop
    hue_ok = (hue_diff <= hue_tol) | (shadowy & (hue_diff <= shadow_hue_tol))

    mask = (hue_ok
            & (sat_diff <= sat_tol)
            & (val_drop <= val_drop_tol)      # not too dark to be shadow
            & (-val_drop <= val_rise_tol))    # but not brighter than floor
    mask = mask.astype(np.uint8)

    # Close pinholes (specular highlights on the floor) then drop specks,
    # so a single glinting pixel does not read as an obstacle.
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    if ignore_bottom_rows:
        mask[usable_h:, :] = 0

    return mask.astype(bool)


# ─── Scan construction ───────────────────────────────────────────────────────

def free_space_scan(frame_rgb: np.ndarray,
                    geom: CameraGeometry,
                    n_rays: int = 13,
                    max_range: float = 12.0,
                    mask: np.ndarray = None,
                    ignore_bottom_rows: int = 0) -> dict:
    """
    Build a gap_follow-compatible scan from one camera frame.

    Returns {"angles_deg": [...], "distances": [...], "max_range": m} with
    `n_rays` bearings spread evenly across the camera's horizontal FOV,
    ascending left -> right — the same contract gap_follow.get_proximity_scan()
    used to satisfy, so GapFollowController consumes it unchanged.

    Pass `mask` to supply your own floor segmentation (e.g. a
    texture-based one for patterned floors); otherwise floor_mask() runs
    with default settings.
    """
    if mask is None:
        mask = floor_mask(frame_rgb, ignore_bottom_rows=ignore_bottom_rows)

    h, w = mask.shape[:2]
    if (h, w) != (geom.height, geom.width):
        raise ValueError(
            f"frame is {w}x{h} but CameraGeometry is configured for "
            f"{geom.width}x{geom.height} — every bearing would be skewed")

    # Span the GROUND fov, not the lens fov — see ground_fov_deg(). Using
    # the narrower lens figure here would drop near obstacles at the frame
    # edges, exactly the ones worth knowing about.
    span = geom.ground_fov_deg()
    half_fov = span / 2.0
    step = span / (n_rays - 1)
    angles = [-half_fov + i * step for i in range(n_rays)]

    # Nearest-wins accumulator, one slot per output ray.
    distances = [max_range] * n_rays

    # Rows above the horizon hold no floor information; starting the walk
    # below it also stops a distant wall/skybox from being mistaken for a
    # near obstacle. horizon_row() goes negative when the horizon is off
    # the top of the frame (normal for a downward-tilted camera), so clamp.
    top_row = max(0, int(math.floor(geom.horizon_row())) + 1)
    bottom_row = h - 1 - ignore_bottom_rows
    if bottom_row < top_row:
        # Nothing between the horizon and the usable bottom of the frame.
        return {"angles_deg": angles, "distances": distances,
                "max_range": float(max_range)}

    for u in range(w):
        column = mask[:, u]

        # Walk up from the bottom; the first non-floor pixel is the base
        # of whatever is occluding the floor in this direction.
        hit_v = None
        for v in range(bottom_row, top_row - 1, -1):
            if not column[v]:
                hit_v = v
                break

        if hit_v is None:
            continue                      # floor all the way up: nothing here

        projected = geom.pixel_to_ground(u, hit_v)
        if projected is None:
            continue                      # above the horizon: no information

        bearing, distance = projected
        if distance >= max_range:
            continue

        # Clamp rather than discard. ground_fov_deg() is measured at the
        # bottom row so nothing should land outside, but if rounding or an
        # unusual geometry puts a real detection just past the edge, the
        # safe response is to report it at the nearest bearing we can
        # represent — never to throw away a detected obstacle.
        idx = int(round((bearing + half_fov) / step))
        idx = max(0, min(n_rays - 1, idx))
        if distance < distances[idx]:
            distances[idx] = distance

    return {
        "angles_deg": angles,
        "distances": distances,
        "max_range": float(max_range),
    }


# ─── Live capture ────────────────────────────────────────────────────────────

def get_visual_scan(camera, geom: CameraGeometry, n_rays: int = 13,
                    max_range: float = 12.0,
                    ignore_bottom_rows: int = 0) -> dict:
    """
    Grab one frame and turn it into a scan — the drop-in replacement for
    gap_follow.get_proximity_scan().

    `camera` is a jetson_utils videoSource (simulated or real):

        import jetson_utils
        from jetbot_nav import visual_scan

        cam  = jetson_utils.videoSource("csi://0")
        geom = visual_scan.CameraGeometry(height_m=0.12, tilt_deg=20.0)
        scan = visual_scan.get_visual_scan(cam, geom)

    Returns None if no frame was available, matching
    get_proximity_scan()'s failure convention so callers can treat the two
    interchangeably.
    """
    import jetson_utils

    img = camera.Capture()
    if img is None:
        return None
    frame = jetson_utils.cudaToNumpy(img)
    if frame is None:
        return None

    return free_space_scan(frame, geom, n_rays=n_rays, max_range=max_range,
                           ignore_bottom_rows=ignore_bottom_rows)


# ─── Debug view ──────────────────────────────────────────────────────────────

def annotate(frame_rgb: np.ndarray, geom: CameraGeometry, scan: dict,
             mask: np.ndarray = None) -> np.ndarray:
    """
    Draw the detected floor boundary and the resulting ray distances onto a
    copy of the frame, for eyeballing calibration. Tinting the floor mask
    green is usually enough to spot a bad segmentation immediately —
    a wrong tilt or height shows up instead as plausible-looking geometry
    with systematically wrong numbers, so check a known distance by hand.
    """
    import cv2

    out = frame_rgb.copy()
    if mask is not None:
        out[mask] = (0.6 * out[mask] + 0.4 * np.array([0, 255, 0])).astype(np.uint8)

    half_fov = geom.hfov_deg / 2.0
    for angle, dist in zip(scan["angles_deg"], scan["distances"]):
        u = int(geom.cx + geom.fx * math.tan(math.radians(angle)))
        if not 0 <= u < geom.width:
            continue
        clear = dist >= scan["max_range"]
        colour = (0, 200, 0) if clear else (255, 0, 0)
        cv2.line(out, (u, geom.height - 1), (u, geom.height - 30), colour, 2)
        cv2.putText(out, f"{dist:.1f}", (max(0, u - 12), geom.height - 36),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, colour, 1)

    cv2.line(out, (0, int(geom.horizon_row())),
             (geom.width - 1, int(geom.horizon_row())), (255, 255, 0), 1)
    return out
