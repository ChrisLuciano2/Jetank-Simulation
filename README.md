# SenSym Robot Simulator

A Unity + Python simulation environment for the NVIDIA JetBot with a 6-DOF robotic arm.

Students write their robot code **once** using the standard JetBot API, run it against the
Unity simulation to verify it works, then deploy the **exact same file** to the physical
robot — no code changes required.

---

## Project Structure

```
SenSym-Robot/
├── Robots/                          ← Unity project (open this in Unity)
│   └── Assets/
│       └── Scripts/
│           ├── TruckController.cs              ← drives the simulated truck
│           ├── RoboticArmNetworkController.cs  ← drives the simulated arm
│           ├── SimCamera.cs                    ← virtual camera + object detection
│           ├── ProximitySensor.cs              ← 13-ray / 120° scanning "lidar"
│           └── Communication/
│               ├── TcpServer.cs       ← control commands  (port 5555)
│               └── SimQueryServer.cs  ← camera/detection/scan  (port 5556)
│
└── Python/                          ← place your JetBot scripts here
    ├── sim_client.py        ← internal TCP client (shared by all shims)
    ├── jetbot_nav/          ← navigation helpers (NOT a hardware shim)
    │   ├── perception.py    ← OpenCV HSV colour-blob detection
    │   ├── gap_follow.py    ← follow-the-gap obstacle avoidance
    │   └── target_seek.py   ← gap-following biased toward a colour target
    ├── jetbot/              ← drop-in for the real jetbot library
    │   ├── __init__.py
    │   └── robot.py         ← Robot class → "set_motors" TCP command
    ├── SCSCtrl/             ← drop-in for the real SCSCtrl library
    │   ├── __init__.py
    │   └── TTLServo.py      ← TTLServo module → arm joint TCP commands
    ├── jetson_utils/        ← drop-in for NVIDIA jetson-utils
    │   └── __init__.py      ← camera frames fetched from Unity; display via cv2
    ├── tensorrt/            ← drop-in for NVIDIA TensorRT
    │   └── __init__.py      ← mock engine / context / bindings
    ├── pycuda/              ← drop-in for PyCUDA
    │   ├── __init__.py
    │   ├── driver.py        ← mock CUDA allocs; synchronize() → Unity detections
    │   └── autoinit.py      ← no-op (just needs to be importable)
    ├── unity_client.py      ← original SenSym client (unchanged)
    ├── test_robotic_arm.py  ← original SenSym test (unchanged)
    └── test_truck_and_arm.py← original SenSym test (unchanged)
```

---

## Quick Start

### 1 — Launch the Unity simulation

1. Open Unity Hub → **Open Project** → select `SenSym-Robot/Robots/`.
2. Open the main scene (`Assets/Scenes/` → your scene file).
3. **Scene setup checklist** (use Tools → Setup Robot Simulator Scene if available):
   - One `GameObject` with **TcpServer** + **SimQueryServer** attached (e.g. "Managers").
   - The truck `GameObject` has **TruckController** (`robotId = "truck_01"`).
   - The arm root `GameObject` has **RoboticArmController** + **RoboticArmNetworkController** (`robotId = "arm_01"`).
   - The **Main Camera** has **SimCamera** attached.
   - Objects you want Python to "detect" are tagged **DetectableObject**
     (Edit → Project Settings → Tags and Layers → add tag).
4. Enable **Run In Background**: Edit → Project Settings → Player → Resolution and Presentation.
5. Press **Play**. The Console should show:
   ```
   [TcpServer] Started on port 5555
   [SimQueryServer] Listening on port 5556
   ```

### 2 — Run your JetBot code against the simulation

Copy your `.py` file into `SenSym-Robot/Python/`, then:

```bash
cd SenSym-Robot/Python
python your_jetbot_script.py
```

The shim packages (`jetbot/`, `SCSCtrl/`, `jetson_utils/`, `tensorrt/`, `pycuda/`) are in the same folder, so Python finds them automatically — **no pip install needed**.

### 3 — Deploy to the physical JetBot (zero code changes)

```bash
# On the JetBot, from a directory WITHOUT the shim folders:
scp your_jetbot_script.py jetbot@192.168.x.x:~/
ssh jetbot@192.168.x.x
python your_jetbot_script.py   # uses the real libraries installed on the JetBot
```

The real `jetbot`, `SCSCtrl`, `jetson_utils`, `tensorrt`, and `pycuda` libraries
installed on the JetBot image shadow the shims automatically.

---

## Example: Running detect_trt.py in Simulation

`detect_trt.py` (from the JetBot codebase) is **completely unchanged**:

```python
# detect_trt.py  — student's original file, no edits
import cv2, numpy as np, pycuda.driver as cuda, pycuda.autoinit, tensorrt as trt

ENGINE_PATH = '/home/jetbot/Documents/Jetson-Nano-YOLOv8-Setup/best_fp16.engine'
# ... (all original code below)
```

In simulation, the engine file path is ignored (the file doesn't need to exist),
the camera comes from Unity, and detections come from Unity's visibility system.

---

## How the Virtual Camera Works

`SimCamera.cs` runs every Unity frame:

1. **Renders** the 3D scene into a 640 × 480 `RenderTexture`.
2. **Encodes** the pixels as JPEG and caches them as base-64.

When Python calls `camera.Capture()`:

```
Python                      TCP (port 5556)               Unity
────────────────────────────────────────────────────────────────
jetson_utils shim  →  {"command":"get_frame"}  →  SimQueryServer
                   ←  {"status":"ok","jpeg":"..."} ←
                   →  decode base-64 JPEG
                   →  return CudaImageSim (wraps numpy RGB array)
```

`jetson_utils.cudaToNumpy()` simply unwraps the numpy array — no GPU is used.
`display.Render(img)` shows the frame in a local `cv2.imshow()` window.

---

## How Object Detection Works

TensorRT cannot run on a PC.  Instead, Unity reports what is **visible to the camera**:

1. Tag scene props with **DetectableObject** in the Unity Inspector.
2. Each frame, `SimCamera.cs` projects each tagged object's bounding box to screen space.
3. When Python's TRT pipeline reaches `stream.synchronize()`, the `pycuda` shim:
   - sends `{"command":"detect_objects"}` to Unity on port 5556,
   - receives bounding boxes `[x1, y1, x2, y2, conf]` in 640 × 640 pixel space,
   - writes them into the output numpy array in **YOLOv8 TRT format `(1, 5, 8400)`**.
4. The student's unchanged `postprocess()` function parses the results as normal.

The complete mock pipeline — from `cuda.mem_alloc()` through `stream.synchronize()` — is
transparent to the student.  They write the same TRT code; the shim handles the rest.

---

## TCP Protocol Reference

| Port | Direction | Format | Purpose |
|------|-----------|--------|---------|
| 5555 | Python → Unity | JSON (no delimiter) | Control commands |
| 5555 | Unity → Python | `OK\n` | Acknowledgment |
| 5556 | Python → Unity | JSON + `\n` | Data queries |
| 5556 | Unity → Python | JSON + `\n` | Frame / detection response |

### Control commands (port 5555) — complete list

| `command` | Fields | Effect |
|-----------|--------|--------|
| `set_motors` | `left`, `right` (−1…1) | Differential-drive (Robot.set_motors / forward / backward / left / right) |
| `move` | `throttle`, `steering` | Legacy throttle+steering |
| `stop` | — | Stop truck |
| `goto` | `x`, `z` | Navigate to world position |
| `set_position` | `x`, `y`, `z` | Teleport |
| `set_rotation` | `rotation_y` | Set heading (degrees) |
| `arm_set_joint` | `joint_index` (0–5), `angle` | Single arm joint (degrees) |
| `arm_set_joints` | `joints` (6-array) | All joints simultaneously |
| `arm_set_gripper` | `gripper_amount` (0–1) | 0 = closed, 1 = open |
| `arm_home` | — | All joints to 0°, gripper 50% |
| `arm_pose` | `pose_name` | `pickup`, `parked`, `forward` |
| `servo_angle` | `servo_id`, `angle` | TTLServo.servoAngleCtrl shim |
| `servo_sync` | `sync_ids[]`, `sync_angles[]` | TTLServo.syncCtrl shim |
| `servo_stop` | `servo_id` | TTLServo.stopServo shim |

### Query commands (port 5556)

| `command` | Response | Notes |
|-----------|----------|-------|
| `get_frame` | `{"status":"ok","jpeg":"<b64>"}` | 640×480 JPEG, base-64 encoded |
| `detect_objects` | `{"status":"ok","objects":[...]}` | Each: `class`, `x1`, `y1`, `x2`, `y2`, `conf` |
| `get_proximity_scan` | `{"status":"ok","fov":120.0,"count":13,"max_range":12.0,"distances":[...]}` | Ray distances left→right across the fan; a ray reads `max_range` when it hits nothing |

---

## Navigation — `jetbot_nav/`

Unlike `jetbot/`, `SCSCtrl/`, and the rest, **`jetbot_nav/` is not a shim.** It is
real navigation code that runs identically in simulation and on hardware, layered
on top of the shims. Nothing in it is simulation-aware.

| Module | What it does | Needs |
|--------|--------------|-------|
| `perception` | HSV colour thresholding on a raw RGB frame → blob bounding boxes | numpy, opencv-python |
| `visual_scan` | Ground-plane projection: camera frame → free-space distances, in the shape `gap_follow` consumes | numpy, opencv-python |
| `gap_follow` | Follow-the-gap avoidance: marks rays beyond `GAP_THRESHOLD` free, finds contiguous runs wide enough to fit through, steers at the chosen gap's centre. `FORWARD` / `PIVOT` / `BACKUP` / `SEARCH` recovery states. | stdlib only |
| `target_seek` | `SeekingGapFollowController` — biases *which gap* `gap_follow` prefers toward a target's bearing, rather than blending steering values. Target bias is suppressed entirely during recovery states. | stdlib only |

### Hardware parity — the camera is the only sensor

**The JETANK has a camera and nothing else.** No lidar, no ultrasonic. So the
obstacle scan is computed *in Python* from the camera image, by finding where the
floor stops in each image column and projecting that pixel onto the ground plane.
Unity's only job is to hand over a rendered frame, which it already does honestly —
there is no simulation-only sensor in the loop, and the identical code runs on the
Jetson against the identical frame shape.

```python
import jetson_utils
from jetbot_nav import visual_scan

cam  = jetson_utils.videoSource("csi://0")
geom = visual_scan.sim_jetank()             # or CameraGeometry(height_m=…, tilt_deg=…)
scan = visual_scan.get_visual_scan(cam, geom)
```

`gap_follow` is agnostic — it consumes `{"angles_deg", "distances", "max_range"}` and
doesn't care how the numbers were produced, so the controller and its regression
suite stay valid on top of a completely different sensing front-end.

**Calibration is load-bearing.** Camera height, downward tilt and FOV are what turn
pixels into metres. Measure height and tilt once with a ruler and a phone level; a
20% height error is a 20% range error, and it *looks* plausible. The simulated
mounting lives in `SceneSetup.cs` (`CamHeight` / `CamTiltDeg` / `CamFovDeg`) and is
mirrored by `visual_scan.sim_jetank()` — change one, change both.

**Known limits, all inherent to seeing with a camera:**

- **Near blind zone.** Anything closer than `geom.min_visible_range()` (~0.8 units as
  mounted) is invisible and reads as *open floor*, not as an error.
- **No peripheral vision.** ~62° of lens, against the 120° a range fan gave.
- **Flat floor assumed.** Ramps and steps read as the wrong distance; overhangs
  (a table edge with clear space beneath) are invisible entirely.
- **Floor-coloured obstacles** are invisible, for the same reason `perception` can't
  find a red block on a red mat.

`ProximitySensor.cs` still exists and still answers `get_proximity_scan`, but it is
**development ground truth only** — it reports exact geometry, which makes it the
reference to validate `visual_scan` against. Never ship behaviour that depends on it.

### Testing

Both suites are fully offline — no Unity, no hardware:

```bash
py -3.8 test_gap_logic.py
```

`test_gap_logic.py` is a closed-loop test: it contains a miniature 2D simulator with
real raycasting and differential-drive kinematics, so the controller's steering
changes what it senses next. `test_target_seek.py` covers the seeking layer.
`test_visual_scan.py` pins the camera projection against hand-computed distances —
a segmentation bug is loud, but a projection bug returns plausible numbers that are
uniformly wrong. Recorded field failures replay from `test_data/`.

Live tests, with Unity playing:

```bash
py -3.8 test_gap_navigation.py 30 myrun.csv
```

---

## Servo ID → Unity Arm Joint Mapping

| Servo ID | Physical role | Unity `joint_index` |
|----------|--------------|---------------------|
| 1 | Base rotation (pan) | 0 — J1 base yaw |
| 2 | Shoulder linkage | 1 — J2 shoulder pitch |
| 3 | Elbow linkage | 2 — J3 elbow pitch |
| 4 | Claw / gripper | gripper (0 = closed → 1 = open) |
| 5 | Camera tilt | 4 — J5 wrist pitch |

`TTLServo.returnOffset()` and the IK math in `xyInput()` / `xyInputSmooth()` run
entirely in Python (same equations as real hardware) — no extra TCP round-trip.

---

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| `Could not connect control channel` | Press Play in Unity **before** running Python |
| `Could not connect query channel` | Add SimQueryServer script to your scene |
| Camera frames are black | Attach SimCamera to the Main Camera GameObject |
| No detections returned | Tag scene objects with **DetectableObject** |
| Arm doesn't move | Confirm RoboticArmNetworkController is on the arm root |
| `Could not read the proximity scan` | ProximitySensor isn't on the truck — run Tools → Setup Robot Simulator Scene |
| Robot never drives, sits in SEARCH/PIVOT | ProximitySensor `Max Range` is below `gap_follow.GAP_THRESHOLD`, so no ray ever reads "free" (the Console warns about this) |
| Rays pass through obstacles | Obstacles need **Colliders**, not just Renderers, with Is Trigger **off** |
| Tank-turn appears sluggish | The controller adds a tiny forward nudge for zero-throttle spins — this is by design |
| `portClose()` crashes | It is a no-op in simulation; this is expected and safe |

---

## License

MIT License — Free to use and modify for educational purposes.
