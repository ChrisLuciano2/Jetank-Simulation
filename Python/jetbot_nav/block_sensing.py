"""
jetbot_nav.block_sensing — locate, size-estimate, and rank blocks from a
camera frame, entirely from what the camera itself can measure.

Not a shim — like the rest of jetbot_nav, this is real code that runs
identically against the simulated camera and the physical JETANK's camera.

HOW SIZE IS ESTIMATED (no ground truth, no hardcoding)
────────────────────────────────────────────────────────────────
1. jetbot_nav.perception finds colour blobs in the raw image (pixel-space
   bounding boxes) — this module doesn't care where the bounding boxes
   came from; any detector producing {x1,y1,x2,y2} works.
2. visual_scan.CameraGeometry.pixel_to_ground() projects the blob's
   BOTTOM-CENTER pixel onto the floor plane — the same technique
   free_space_scan uses for obstacle distance — on the assumption the
   block is resting on the floor. That gives (bearing_deg, distance_m).
3. Real-world width/height are recovered from the blob's pixel span and
   that distance via the pinhole relation

       real_size ~= pixel_size * distance / focal_length_px

   the standard "known distance -> angular size -> real size" inversion.
   Valid when distance is large relative to the block itself and the
   block is reasonably close to the image center — both true for "the
   block in front of you," less true at the edges of frame or far away.

WHAT THIS INHERENTLY CANNOT DO (same family of caveats as visual_scan.py)
────────────────────────────────────────────────────────────────
  - Needs the block resting on visible floor — a stacked or occluded block
    won't project correctly, so this only senses UNPLACED blocks, not the
    ones already in the tower (the build script tracks tower state itself
    once a block is placed — see autonomous_tower_build.py).
  - Distance/size accuracy degrades off-center and at long range, same as
    any monocular estimate.
  - A block-coloured floor breaks colour segmentation exactly like it
    breaks perception.py and floor_mask() elsewhere in this codebase.

TESTING
    py -3.8 test_block_sensing.py    — offline, synthetic frame, no Unity
"""

from . import perception
from .visual_scan import CameraGeometry


def _pixel_span_to_world(pixel_span, distance_m, focal_px):
    """Pinhole inversion: real_size ~= pixel_size * distance / focal_length."""
    return pixel_span * distance_m / focal_px


def sense_blocks(frame_rgb, geom: CameraGeometry, color="red",
                  min_area=80, max_radius_m=None):
    """
    Detect blocks of `color` in frame_rgb and return them sorted LARGEST
    FOOTPRINT FIRST — estimated entirely from the image, nothing here is
    looked up from a scene file or passed in as a known size.

    Returns a list of dicts, each:
        {
            "bearing_deg": ...,   # + = right of forward, matches gap_follow
            "distance_m": ...,    # ground distance to the block's base
            "width_m": ...,       # estimated real-world footprint width
            "height_m": ...,      # estimated real-world height
            "pixel_bbox": (x1, y1, x2, y2),
        }

    Blocks whose base doesn't project onto the floor (at/above the
    horizon — usually a false detection, or one that isn't actually
    resting on the floor) are dropped, not guessed at.

    Blocks whose bounding box touches a frame edge are also dropped —
    pixel_to_ground() below assumes (base_u, y2) is the block's true
    ground-contact point, but a box clipped by the frame edge (typically
    a block near/beyond the edge of the camera's FOV) has its real base
    outside the visible frame. Projecting the clipped edge pixel instead
    of the true base silently understates distance, sometimes by a large
    factor (a block near the FOV edge can read as m of meters closer than
    it really is) -- caught via a ground-truth pose check finding
    navigate_to_bearing()/creep_to_target() reporting "arrived" without
    the chassis having moved at all. Same "don't guess at it" principle
    as the floor-projection check above: if the box isn't fully in frame,
    its true extent isn't known, so don't trust it this cycle -- the
    caller will re-sense next cycle, likely with a better view once the
    robot has turned/moved from an earlier, still-trustworthy detection.

    max_radius_m, if given, drops blocks farther than that — the "sense
    blocks in front of them, within X radius" filter.
    """
    blobs = perception.detect_color_blobs(frame_rgb, color, min_area=min_area)

    blocks = []
    for b in blobs:
        x1, y1, x2, y2 = b["x1"], b["y1"], b["x2"], b["y2"]

        if x1 <= 0 or y1 <= 0 or x2 >= geom.width or y2 >= geom.height:
            continue   # clipped by the frame edge -- true extent unknown

        base_u = (x1 + x2) / 2.0

        projected = geom.pixel_to_ground(base_u, y2)
        if projected is None:
            continue   # base isn't on the floor plane -- can't trust this blob
        bearing, distance = projected

        if max_radius_m is not None and distance > max_radius_m:
            continue

        width_m  = _pixel_span_to_world(x2 - x1, distance, geom.fx)
        height_m = _pixel_span_to_world(y2 - y1, distance, geom.fy)

        blocks.append({
            "bearing_deg": bearing,
            "distance_m": distance,
            "width_m": width_m,
            "height_m": height_m,
            "pixel_bbox": (x1, y1, x2, y2),
        })

    # Largest footprint first -- "largest on the bottom" ordering uses
    # footprint WIDTH rather than height, since a tall-but-narrow block
    # placed first would make a top-heavy, unstable base.
    blocks.sort(key=lambda blk: blk["width_m"], reverse=True)
    return blocks
