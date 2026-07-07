"""
test_perception.py — quick manual test for jetbot_nav.perception.

Run against the simulation:
    1. Press Play in Unity (make sure a colored, detectable prop is
       visible to the Main Camera).
    2. cd SenSym-Robot/Python
    3. python test_perception.py red      (or blue / green / yellow)

Run on the physical Jetson: identical command, identical file. The
camera source and color-detection logic are the same either way —
only jetson_utils' internals differ (real CSI camera vs. Unity frame).
"""

import sys
import time

import cv2
import jetson_utils
from jetbot_nav import perception

TARGET_COLOR = sys.argv[1] if len(sys.argv) > 1 else "red"

camera  = jetson_utils.videoSource("csi://0")
display = jetson_utils.videoOutput("display://0")

print(f"[test_perception] Looking for color: {TARGET_COLOR}")
print("[test_perception] Press 'q' in the display window to quit")

try:
    while display.IsStreaming():
        img = camera.Capture()
        if img is None:
            time.sleep(0.05)
            continue

        frame = jetson_utils.cudaToNumpy(img)  # (H, W, 3) RGB

        blob = perception.largest_blob(frame, TARGET_COLOR)
        if blob:
            print(
                f"  {TARGET_COLOR} blob: center=({blob['cx']:.0f}, {blob['cy']:.0f}) "
                f"area={blob['area']:.0f}"
            )
            # Draw the box on the frame before displaying it
            bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            cv2.rectangle(
                bgr,
                (int(blob["x1"]), int(blob["y1"])),
                (int(blob["x2"]), int(blob["y2"])),
                (0, 255, 0), 2,
            )
            frame = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

        display.Render(jetson_utils.cudaFromNumpy(frame))

finally:
    camera.Close()
    display.Close()
