# JETANK Simulation — Project Status

This document describes the **tower-building project** built on top of the
simulation framework — what it's trying to do, how it's put together, what
currently works, and what doesn't yet. For the underlying Unity/Python
simulation mechanics (TCP protocol, camera/sensor shims, how to open the
Unity project), see [`README.md`](README.md) — that document is generic to
the simulator itself and won't be duplicated here.

---

## 1. Goal

Two simulated JETANK robots (each a small tracked chassis with a 6-DOF
robotic arm) work together to build a tower out of red blocks scattered in
their shared workspace:

1. Each robot **senses** red blocks with its camera (no other sensors —
   this constraint is deliberate, see Section 2).
2. They **sort by size** — largest block goes on the bottom, progressively
   smaller blocks stack on top.
3. They **cooperate** to build one shared tower at a common marker
   location, without ever colliding with each other.
4. Critically: **the exact same Python code must run unmodified on the
   real physical JETANK hardware**, not just in simulation. Nothing in the
   robot-control code may assume it's running in Unity.

This is meant as both a capstone robotics exercise and a proof that a
camera-only, no-lidar, no-encoder, no-IMU robot can still do useful
autonomous manipulation and multi-robot coordination — every design
decision in the codebase traces back to that hardware constraint.

---

## 2. Architecture Overview

```
┌─────────────────────────┐         TCP (JSON)         ┌──────────────────────────┐
│   Unity simulation        │◄──────────────────────────►│   Python control code     │
│   (Robots/)                │   port 5555: commands       │   (Python/)               │
│                             │   port 5556: queries        │                          │
│  - Two JETANK robots        │                             │  - jetbot/, SCSCtrl/,     │
│  - Red blocks, build marker │                             │    jetson_utils/ (shims — │
│  - Physics, camera render   │                             │    identical API to the   │
└─────────────────────────┘                             │    real hardware libs)    │
                                                          │  - jetbot_nav/ (real nav  │
                                                          │    logic, NOT a shim)     │
                                                          │  - autonomous_tower_      │
                                                          │    build.py (orchestrator)│
                                                          │  - robot_coord/ (MQTT      │
                                                          │    coordination layer)    │
                                                          └──────────────────────────┘
                                                                       │
                                                                       │ MQTT (port 1883)
                                                                       ▼
                                                          ┌──────────────────────────┐
                                                          │  Broker (mosquitto)        │
                                                          │  Robot A ⇄ Robot B state   │
                                                          │  exchange for pacing/      │
                                                          │  build-zone handoff        │
                                                          └──────────────────────────┘
```

Each robot runs its own instance of `autonomous_tower_build.py` (one
process per robot — `ROBOT_ID=robot_a` / `ROBOT_ID=robot_b`), each
connecting to the same Unity simulation (which hosts both simulated
robots) and the same MQTT broker.

### Why no shared coordinate system

The real hardware has **a camera and nothing else** — no lidar, no wheel
encoders, no IMU. That means neither robot can navigate to a hardcoded
world coordinate ("drive to (2, 0)"), because neither has any way to know
where it currently is in a shared frame. Every navigation target in this
codebase — a block, the other robot, the build-zone marker — is something
the camera can directly perceive (bearing + estimated distance), never an
absolute coordinate.

### Two independent collision-avoidance layers

1. **Reactive / hard** (`jetbot_nav.gap_follow`): the actual safety
   guarantee. Fed by the camera's own free-space scan, it steers around or
   stops for anything blocking the floor — including the other robot's
   body, which occludes the floor exactly like any obstacle. This works
   even if the network, the MQTT broker, or the other robot's whole
   process dies, because it doesn't depend on cooperation.
2. **Cooperative / soft** (`robot_coord`): bounded-slack pacing (a robot
   won't get more than `MAX_SLACK` blocks ahead of its partner) and
   build-zone-clear signaling over MQTT, so in the common case the two
   robots don't even attempt to enter the build zone at the same time —
   this is what makes runs efficient, not what makes them safe.

### The pick-and-place loop (`autonomous_tower_build.py`)

For each block, per robot, repeated until no more blocks are sensed:

1. **Sense** — `jetbot_nav.block_sensing` finds red blobs in the camera
   frame, estimates each one's bearing/distance/size.
2. **Navigate** — `navigate_to_bearing()` drives toward the largest sensed
   block using gap-following biased toward that bearing.
3. **Creep** — `creep_to_target()` takes over for the final approach,
   since general obstacle-avoidance driving isn't precise enough for the
   arm to physically reach the target.
4. **Grab** — `jetbot_nav.arm_ops.reach_and_grab()` reaches down and
   closes the gripper, confirming a real grab via simulator feedback (see
   Section 4 for the reliability history here).
5. **Stow** — lift the block clear of the floor before driving.
6. **Find the build marker** — same bearing-based sensing, targeting a
   different color (`BUILD_MARKER_COLOR`).
7. **Navigate + creep** to the marker, coordinating over MQTT so the two
   robots don't collide at the shared build zone.
8. **Place** — `arm_ops.place_at_height()` releases the block at a height
   computed from the sum of already-placed blocks' *sensed* heights (never
   hardcoded).

Nothing about which blocks exist, their sizes, or the current tower height
is hardcoded anywhere — all of it is sensed or computed from what was
sensed, every cycle.

---

## 3. Repository Map (project-specific files)

On top of the generic simulator structure already documented in
`README.md`:

| Path | What it is |
|---|---|
| `Python/autonomous_tower_build.py` | The orchestrator — the sense→navigate→grab→place loop described above. Run one copy per robot. |
| `Python/jetbot_nav/arm_ops.py` | Pick/place arm control built on the `TTLServo` shim. **Read this file's module docstring before touching arm timing constants** — it documents several non-obvious, hard-won findings (see Section 5). |
| `Python/jetbot_nav/block_sensing.py` | Camera-based red-block detection: bearing, distance, size estimation. |
| `Python/robot_coord/` | MQTT-based two-robot coordination layer (pacing + build-zone handoff). Not a hardware shim — install `paho-mqtt<2.0` on the dev PC and both physical robots. |
| `Python/sim_robot_id.py` | Maps `ROBOT_ID` env var (`robot_a`/`robot_b`) to the correct simulated robot in Unity. Simulation-only. |
| `Robots/Assets/Scripts/RoboticArmController.cs` | Unity-side arm physics: joint interpolation, gripper open/close, and the physical grab check (`TryGrab()`). |
| `Robots/Assets/Scripts/GraspableBlock.cs` | Marks a block as grabbable and exposes its ground-truth position for diagnostics (`get_block_pose` — simulation-only, never used by real navigation code). |
| `Python/diag_*.py` | Diagnostic/calibration scripts used to find and verify the fixes below. Not part of the production robot code path. See Section 6. |

---

## 4. Current Status

| Subsystem | Status |
|---|---|
| Sensing (block/marker detection) | **Solid.** Camera-based color-blob detection has been the most reliable part of the pipeline throughout the project's history. |
| Navigation, obstacle avoidance, bearing alignment | **Solid.** `gap_follow`/`target_seek` are well-tested (see `README.md`'s navigation section for the offline test suites). |
| Arm reach & grab | **Substantially improved, not fully reliable.** See Section 5 — currently succeeds roughly 5 out of 6 attempts under the most recent fix, with one remaining, not-yet-fully-explained failure mode. A further fix (wider fallback angle set) has been applied but not yet verified across repeated trials. |
| Placement | **Working** in every full-cycle trial that got past the grab step (heights consistently 0.070–0.072m for a single block). Multi-block stacking is not yet exercised — every successful trial so far has placed exactly one block before the scene ran out of visible blocks. |
| Two-robot simultaneous operation | **Not yet exercised at all.** Every test so far has run a single robot solo (`Coordinator(solo_ok=True)`). The MQTT pacing/hand-off logic exists and is used, but has never been tested with two robots actually running concurrently. |
| Real-hardware validation | **Not done.** Everything above has only been run in the Unity simulation. The codebase is written so the same files should run unmodified on the physical JETANKs, but that has not been verified. |

**Bottom line:** the project can now reliably (most of the time) run a
complete sense → navigate → grab → carry → place cycle for one block, with
one robot, in simulation. That is a meaningful, recent milestone — as
recently as a few sessions ago, this had *never* succeeded even once.
Multi-block towers, two-robot coordination, and hardware deployment are
all still ahead.

---

## 5. Known Issues & Bug History

This section exists so the next engineer doesn't have to rediscover
things that have already been investigated, including wrong turns that
were ruled out.

### 5.1 (Fixed) Arm's inverse-kinematics can't reach the floor

`TTLServo.xyInput()` — the standard IK entry point, shared with the real
hardware code — has a 2-link arm model that geometrically cannot compute a
pose reaching all the way to floor level (confirmed by exhaustive sweep;
the gripper never got below y=0.674m through that path). **Fix:**
`reach_and_grab()` bypasses `xyInput()` entirely for picking, commanding
the shoulder/elbow servos' raw angles directly instead — the same
primitive already used for base yaw and the gripper, just skipping
`xyInput()`'s limited convenience wrapper for this one case. This does
not affect placing, which happens above floor level and doesn't hit this
limitation.

### 5.2 (Fixed) Physical grab check used the wrong reference point

The code deciding whether a grab physically registers
(`RoboticArmController.TryGrab()`) was measuring distance from the arm's
IK-reference point (`j5_wrist`), not from where the gripper fingers
actually are — a difference of roughly 120–290mm. Every angle search done
before this was fixed could never have found a working configuration for
a floor-level block, no matter how wide the search was, because it was
checking proximity from the wrong point entirely. **Fix:** added
`GetGripperPosition()` (midpoint of the two gripper fingers), used only
for this physical check; the existing IK-reference point is untouched and
still used for reporting arm state.

### 5.3 (Fixed) Grabbed blocks were dragged underground

Even after 5.1–5.2, blocks would visibly sink below the floor immediately
after a successful grab (observed up to 241mm, ending underground). The
first hypothesis — a physics/gravity issue, block's `Rigidbody` fighting
the grab's parenting — **was wrong and was caught before it cost a test
cycle**: the blocks have no `Rigidbody` component at all (checked
directly in the scene file). The real cause: the code was only waiting
0.8 seconds for the arm's shoulder/elbow to finish moving before closing
the gripper, but the arm's actual joint speed needs over a second for a
typical move. The arm was still physically swinging when the gripper
closed and grabbed the block, and kept swinging afterward — dragging the
freshly-grabbed block down with it. **Fix:** the wait time
(`arm_ops.MOVE_SETTLE_S`) was raised from 0.8s to 1.5s. Confirmed directly
via the simulator's arm-state query that the joints now fully arrive
before the gripper closes, and confirmed the dragging is completely gone
(0.0mm displacement measured on a clean grab afterward).

### 5.4 (Partially understood, not fully fixed) Intermittent total grab failure

Independent of the above, `reach_and_grab()` tries a short list of
several confirmed-working `(shoulder, elbow)` angle pairs in sequence
(since a single fixed pair was found to succeed once and fail on an
apparently identical next attempt). Even with the fix in 5.3 applied,
this can still fail against **every** candidate in the list, roughly 1 in
6 times measured across repeated trials. This is the one remaining known
reliability gap.

What's been ruled out or tested:
- **Not the same bug as 5.3** — reproduced fresh with 5.3's fix already in
  place.
- **Not simply "not enough elapsed time"** — every test run *with*
  detailed position-logging attached (which adds extra network
  round-trips, hence extra elapsed time) succeeded; a version with an
  equivalent plain pause instead of the logging **still failed** on its
  first try, directly showing that the extra wall-clock time itself isn't
  what matters.
- **The underlying calibration data may itself be noisy.** The list of
  "confirmed working" angle pairs was found by sweeping a grid and
  checking which ones registered a grab — but that sweep's own results
  don't form a clean, physically-sensible pattern (e.g., one angle pair
  works, the next doesn't, then a nearby one works again), suggesting the
  calibration itself may have been affected by similar settle-timing
  sensitivity, making some "confirmed working" pairs less trustworthy
  than they appear.
- **Applied, not yet re-verified:** widened the fallback list from 6 to
  all 10 pairs the original calibration sweep found working, and
  increased the gripper's own settle time for extra margin. This should
  reduce the failure rate further by giving the arm more real options to
  try, but has not yet been tested across repeated trials.

**For the next engineer picking this up:** don't assume this is solved.
Test it across many repeated runs before trusting it, and see the
project's git history / commit messages for the exact diagnostic scripts
used to investigate it (`diag_grab_pose_check.py`,
`diag_full_cycle_instrumented.py`) if picking this investigation back up.

### 5.5 (Noted, not investigated) Placement approach hits a hard joint limit every time

Every successful full-cycle test run has hit the same warning
(`servo 2 angle -50.8°, clamped to -20°`) while navigating toward the
build marker to place a block. It has recovered and placed successfully
every time this was observed, so it isn't currently blocking anything —
but it happens consistently, not randomly, which suggests a real,
probably easy-to-find bug in whatever computes the approach angle for
placement specifically (separate from the pickup-side issues above).
Worth a look once Section 5.4 is settled.

---

## 6. Diagnostic Tooling

None of these are part of the production robot code path — they're
development/investigation tools, run manually:

| Script | Purpose |
|---|---|
| `diag_full_cycle.py` | Runs one complete `autonomous_tower_build.build_tower()` cycle solo, for end-to-end testing. |
| `diag_grab_pose_check.py` | Runs the real sense→navigate→creep pipeline, then manually steps through each grab attempt, comparing the block's real ground-truth position against the gripper's real position at every step — the tool that found the 5.3 bug. |
| `diag_full_cycle_instrumented.py` | Same idea as above but patches the instrumentation directly into a real `build_tower()` run, so the exact production code path is being measured. |
| `diag_floor_reach_yaw0.py` | The original angle-calibration sweep that produced `arm_ops.FLOOR_REACH_CANDIDATES`. |
| `get_block_pose` (Unity query, simulation-only) | Ground-truth block position, for grading a grab attempt against reality. **Never call this from `jetbot_nav/` or any production code** — real hardware has no way to know a block's true position independent of its own camera, and code that quietly depended on this would silently break on the real robot. |

---

## 7. Environment Notes

- **A local MQTT broker (mosquitto) must be running on port 1883** for
  `robot_coord` to work, even when testing a single robot solo.
- **Synthetic/automated mouse clicks do not reliably work against the
  Unity Editor** in this development environment — this affects anyone
  trying to script Unity interactions (e.g., resetting the scene between
  test runs). A real, physical click from a human is currently the only
  reliable way to press Play/Stop in Unity. This is a real, confirmed
  environment quirk, not a one-off mistake — worth knowing before
  assuming test automation can drive Unity end-to-end.
- Nothing in this project was committed to version control until
  recently — check `git log` for the actual commit history rather than
  assuming any particular state predates a given commit.

---

## 8. Recommended Next Steps (priority order)

1. **Verify the Section 5.4 fix** (widened fallback angles) across many
   repeated full-cycle trials before trusting it.
2. **Multi-block towers**: every successful run so far has placed exactly
   one block. Test a full multi-block stacking sequence.
3. **Two-robot simultaneous operation**: completely unexercised. Run two
   robot processes against the same simulation at once and verify the
   coordination layer (pacing, build-zone hand-off) actually works, not
   just that it exists.
4. **Section 5.5** (placement approach joint-limit warning) — likely a
   quick, real fix once someone looks.
5. **Real hardware validation** — the entire project's premise depends on
   the same code running unmodified on physical JETANKs. This has never
   been attempted yet.
