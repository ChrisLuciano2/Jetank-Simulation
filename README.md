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
│           └── Communication/
│               ├── TcpServer.cs       ← control commands  (port 5555)
│               └── SimQueryServer.cs  ← camera/detection  (port 5556)
│
└── Python/                          ← place your JetBot scripts here
    ├── sim_client.py        ← internal TCP client (shared by all shims)
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
| Tank-turn appears sluggish | The controller adds a tiny forward nudge for zero-throttle spins — this is by design |
| `portClose()` crashes | It is a no-op in simulation; this is expected and safe |

---

## License

MIT License — Free to use and modify for educational purposes.
