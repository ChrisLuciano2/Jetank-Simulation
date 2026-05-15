r"""
test_all.py -- Quick smoke-test for every shim module.

Run with Unity in Play mode:
    cd SenSym-Robot\SenSym-Robot\Python
    python test_all.py

Tests (pick one by number when prompted):
  1  Motors      — drive forward, turn, stop
  2  Camera      — grab a frame, show it in a cv2 window
  3  Servos      — move arm joints via TTLServo API
  4  Detection   — run the full TRT mock pipeline
  5  All         — run all tests in sequence
"""

import sys, time
import numpy as np

# ── helpers ──────────────────────────────────────────────────────────────────

MENU = """
Select a test:
  1  Motors  (robot.forward / turn / stop)
  2  Camera  (grab frame from Unity)
  3  Servos  (TTLServo arm joints)
  4  Detection (TRT mock pipeline)
  5  All
  q  Quit
> """

def header(name):
    print(f"\n{'='*50}")
    print(f"  TEST: {name}")
    print('='*50)

# ── Test 1 — Motors ──────────────────────────────────────────────────────────

def test_motors():
    header("Motors")
    from jetbot import Robot
    robot = Robot()

    print("Forward 1s ...")
    robot.forward(0.5)
    time.sleep(1)

    print("Turn right 1s ...")
    robot.right(0.4)
    time.sleep(1)

    print("Turn left 1s ...")
    robot.left(0.4)
    time.sleep(1)

    print("Backward 1s ...")
    robot.backward(0.4)
    time.sleep(1)

    print("Stop.")
    robot.stop()
    print("Motors OK")

# ── Test 2 — Camera ──────────────────────────────────────────────────────────

def test_camera():
    header("Camera")
    import cv2
    import jetson_utils

    camera = jetson_utils.videoSource("csi://0")
    img = camera.Capture(timeout=3000)

    if img is None:
        print("FAIL: No frame received from Unity. Is Play mode running?")
        return

    arr = jetson_utils.cudaToNumpy(img)
    print(f"Frame shape: {arr.shape}  dtype: {arr.dtype}")

    # Show in a window for 3 seconds
    bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    cv2.imshow("Unity Camera Frame", bgr)
    print("Frame displayed — press any key to close")
    cv2.waitKey(3000)
    cv2.destroyAllWindows()
    camera.Close()
    print("Camera OK")

# ── Test 3 — Servos ──────────────────────────────────────────────────────────

def test_servos():
    header("Servos")
    from SCSCtrl import TTLServo

    print("Servo 1 (base yaw) → 30°")
    TTLServo.servoAngleCtrl(1, 30, 1, 500)
    time.sleep(0.8)

    print("Servo 2 (shoulder) → 20°")
    TTLServo.servoAngleCtrl(2, 20, 1, 500)
    time.sleep(0.8)

    print("Servo 3 (elbow) → 15°")
    TTLServo.servoAngleCtrl(3, 15, 1, 500)
    time.sleep(0.8)

    print("Servo 4 (gripper) → open (90°)")
    TTLServo.servoAngleCtrl(4, 90, 1, 300)
    time.sleep(0.5)

    print("Servo 4 (gripper) → close (-90°)")
    TTLServo.servoAngleCtrl(4, -90, 1, 300)
    time.sleep(0.5)

    print("Servo 5 (wrist) → 10°")
    TTLServo.servoAngleCtrl(5, 10, 1, 500)
    time.sleep(0.8)

    print("Sync all back to 0°")
    TTLServo.syncCtrl([1, 2, 3, 5], [300, 300, 300, 300], [0, 0, 0, 0])
    time.sleep(1)

    print("Servos OK")

# ── Test 4 — Detection pipeline ──────────────────────────────────────────────

def test_detection():
    header("Detection (TRT mock)")
    import pycuda.driver as cuda
    import pycuda.autoinit
    import tensorrt as trt

    # Mimic detect_trt.py setup
    TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
    input_shape  = (1, 3, 640, 640)
    output_shape = trt.OUTPUT_SHAPE  # (1, 5, 8400)

    with open("dummy.engine", "wb") as f:   # fake engine file
        f.write(b"\x00" * 8)
    with open("dummy.engine", "rb") as f, trt.Runtime(TRT_LOGGER) as runtime:
        engine = runtime.deserialize_cuda_engine(f.read())

    import os; os.remove("dummy.engine")

    context  = engine.create_execution_context()
    stream   = cuda.Stream()

    d_input  = cuda.mem_alloc(trt.volume(input_shape)  * np.float32().nbytes)
    d_output = cuda.mem_alloc(trt.volume(output_shape) * np.float32().nbytes)

    input_np  = np.random.rand(*input_shape).astype(np.float32)
    output_np = np.zeros(output_shape,  dtype=np.float32)

    cuda.memcpy_htod_async(d_input, input_np, stream)
    context.execute_async_v2(
        bindings=[int(d_input), int(d_output)],
        stream_handle=stream.handle
    )
    cuda.memcpy_dtoh_async(output_np, d_output, stream)
    stream.synchronize()   # ← queries Unity for detections here

    # output_np shape (1, 5, 8400); each column is a detection [x1,y1,x2,y2,conf]
    detections = output_np[0].T          # (8400, 5)
    active = detections[detections[:, 4] > 0.5]
    print(f"Detections received from Unity: {len(active)}")
    for i, d in enumerate(active):
        print(f"  [{i}] x1={d[0]:.1f} y1={d[1]:.1f} x2={d[2]:.1f} y2={d[3]:.1f} conf={d[4]:.2f}")
    if len(active) == 0:
        print("  (none — make sure a DetectableObject-tagged object is in camera view)")
    print("Detection pipeline OK")

# ── main ─────────────────────────────────────────────────────────────────────

TESTS = {
    "1": ("Motors",    test_motors),
    "2": ("Camera",    test_camera),
    "3": ("Servos",    test_servos),
    "4": ("Detection", test_detection),
}

if __name__ == "__main__":
    # Allow quick run: python test_all.py 1
    choice = sys.argv[1] if len(sys.argv) > 1 else None

    if choice is None:
        choice = input(MENU).strip().lower()

    if choice == "5" or choice == "all":
        for name, fn in TESTS.values():
            try:
                fn()
            except Exception as e:
                print(f"  ERROR in {name}: {e}")
    elif choice in TESTS:
        name, fn = TESTS[choice]
        try:
            fn()
        except Exception as e:
            print(f"  ERROR: {e}")
            raise
    elif choice == "q":
        sys.exit(0)
    else:
        print("Unknown choice.")
