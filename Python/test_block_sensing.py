"""
test_block_sensing.py — offline tests for jetbot_nav.block_sensing.

No Unity, no camera: draws synthetic red rectangles at pixel positions
whose ground-plane projection is independently known (reusing the same
hand-derived reference point test_visual_scan.py checks pixel_to_ground
against), then checks sense_blocks() recovers the right distance and
applies the width/height pinhole formula correctly, sorts largest-first,
and respects max_radius_m.

    py -3.8 test_block_sensing.py
"""

import math

import numpy as np

from jetbot_nav.visual_scan import CameraGeometry
from jetbot_nav.block_sensing import sense_blocks

_results = []


def check(label, ok, detail=""):
    _results.append(bool(ok))
    print(f"{'PASS' if ok else 'FAIL'}  {label}" + (f"  {detail}" if detail and not ok else ""))


def close(a, b, tol):
    return a is not None and b is not None and abs(a - b) <= tol


# Same setup as test_visual_scan.py's hand-derived reference: camera 0.5 m
# up, pitched 30 deg down. The bottom-centre pixel's ground distance was
# independently derived there as 0.5 / tan(30 + 24.4 deg) = 0.3573 m.
geom = CameraGeometry(height_m=0.5, tilt_deg=30.0,
                      hfov_deg=62.2, vfov_deg=48.8, width=640, height=480)
BOTTOM_ROW_DISTANCE = 0.5 / math.tan(math.radians(30 + 48.8 / 2))


def make_frame(rects, bg=(80, 130, 80)):
    """rects: list of (x1, y1, x2, y2) in pixels, drawn pure red (255,0,0)."""
    frame = np.full((geom.height, geom.width, 3), bg, dtype=np.uint8)
    for x1, y1, x2, y2 in rects:
        frame[int(y1):int(y2), int(x1):int(x2)] = (255, 0, 0)
    return frame


# ─── 1. Single block: distance matches the independent reference,
#        width/height match the pinhole formula computed separately ────────

x1, y1, x2, y2 = geom.cx - 30, geom.height - 30, geom.cx + 30, geom.height - 1
frame = make_frame([(x1, y1, x2, y2)])
blocks = sense_blocks(frame, geom, color="red", min_area=50)

check("finds exactly one block", len(blocks) == 1, f"got {len(blocks)}")

if blocks:
    blk = blocks[0]
    check("distance matches bottom-row reference (0.3573 m)",
          close(blk["distance_m"], BOTTOM_ROW_DISTANCE, 0.01),
          f"got {blk['distance_m']}")
    check("bearing ~0 (rectangle centred on optical axis)",
          close(blk["bearing_deg"], 0.0, 0.5), f"got {blk['bearing_deg']}")

    # Tolerance, not exact match: detect_color_blobs runs MORPH_OPEN/CLOSE
    # (5x5 kernel) before cv2.boundingRect, which can shift the detected
    # box by a pixel or two versus the nominal drawn rectangle -- that's a
    # real, expected property of the detector, not a bug in the pinhole
    # formula this test is actually checking.
    expected_w = (x2 - x1) * blk["distance_m"] / geom.fx
    expected_h = (y2 - y1) * blk["distance_m"] / geom.fy
    check("width matches independent pinhole computation (+/- 2px)",
          close(blk["width_m"], expected_w, 2 * blk["distance_m"] / geom.fx),
          f"got {blk['width_m']} want {expected_w}")
    check("height matches independent pinhole computation (+/- 2px)",
          close(blk["height_m"], expected_h, 2 * blk["distance_m"] / geom.fy),
          f"got {blk['height_m']} want {expected_h}")
    check("width_m is a plausible small positive size (< 2 m)",
          0 < blk["width_m"] < 2.0, f"got {blk['width_m']}")

# ─── 2. Two blocks of different pixel width -> largest-footprint-first ─────

small = (geom.cx - 200, geom.height - 25, geom.cx - 180, geom.height - 1)   # 20 px wide
big   = (geom.cx + 150, geom.height - 40, geom.cx + 230, geom.height - 1)   # 80 px wide
frame2 = make_frame([small, big])
blocks2 = sense_blocks(frame2, geom, color="red", min_area=50)

check("finds both blocks", len(blocks2) == 2, f"got {len(blocks2)}")
check("sorted largest footprint first",
      len(blocks2) == 2 and blocks2[0]["width_m"] > blocks2[1]["width_m"],
      f"widths: {[b['width_m'] for b in blocks2]}")

# ─── 3. max_radius_m filters out far blocks ────────────────────────────────
# Row 0 (top of frame) is the farthest ground point this geometry can see
# at all -- horizon_row() is negative here (off the top of frame, the
# normal case for a downward-tilted camera per its own docstring), so
# using it directly as a row index would wrap around via Python's
# negative-index slicing and silently draw near the BOTTOM of the frame
# instead. Top-of-frame is the correct "farthest visible" stand-in.

top_row_distance = geom.pixel_to_ground(geom.cx, 0)[1]
far = (geom.cx - 15, 0, geom.cx + 15, 6)
frame3 = make_frame([far])
all_blocks = sense_blocks(frame3, geom, color="red", min_area=5)
near_only  = sense_blocks(frame3, geom, color="red", min_area=5,
                          max_radius_m=top_row_distance * 0.5)

check("far block detected with no radius limit", len(all_blocks) == 1, f"got {len(all_blocks)}")
check("far block dropped once max_radius_m is tighter than its distance",
      len(near_only) == 0,
      f"got {len(near_only)}, block distance {all_blocks[0]['distance_m'] if all_blocks else None}, "
      f"limit {top_row_distance * 0.5}")

# ─── Summary ────────────────────────────────────────────────────────────────

passed = sum(_results)
total = len(_results)
print(f"\n{passed}/{total} checks passed")
if passed != total:
    raise SystemExit(1)
