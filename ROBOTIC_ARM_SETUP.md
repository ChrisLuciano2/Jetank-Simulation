# Robotic Arm Python Control - Setup Guide

This guide will help you set up Python control for the robotic arm in Unity.

## Prerequisites

- Unity project with TcpServer already set up (working for truck control)
- Robotic arm model imported and added to the scene
- RoboticArmController.cs attached to the arm
- RoboticArmDemo.cs (can be disabled once network control is working)

## Unity Setup

### Step 1: Add Network Controller to Robotic Arm

1. **Locate the robotic arm** in your Unity scene hierarchy
   - It should be the GameObject with `RoboticArmController` attached
   - Usually named "base_plate" or "robotic_arm_6dof"

2. **Add the RoboticArmNetworkController script**:
   - Select the robotic arm GameObject
   - Click "Add Component" in the Inspector
   - Search for "RoboticArmNetworkController"
   - Add it to the GameObject

3. **Configure the Robot ID**:
   - In the Inspector, find the "RoboticArmNetworkController" component
   - Set the "Robot Id" field to `arm_01` (or your preferred ID)
   - This ID will be used from Python to control this specific arm

4. **Disable or remove RoboticArmDemo** (optional):
   - If you have `RoboticArmDemo` attached, you can:
     - Uncheck it to disable
     - Or remove it entirely
   - This prevents the demo from interfering with network control

### Step 2: Verify TcpServer

1. **Check that TcpServer exists** in your scene:
   - Look for a GameObject with the "TcpServer" component
   - Usually named "NetworkManager" or "TcpServer"

2. **Verify settings**:
   - Port should be `5555` (default)
   - "Auto Start" should be checked

### Step 3: Create Meta File for Network Controller

Unity needs a .meta file for the new script. This will be created automatically when Unity detects the new file.

1. **Switch to Unity**
2. **Wait for Unity to compile** (see bottom-right progress bar)
3. **Check Console** for any errors

If you see errors about namespaces:
- Make sure `RoboticArmNetworkController.cs` is in the `Assets/Scripts/` folder
- Check that `TcpServer.cs` is in `Assets/Scripts/Communication/`

## Python Setup

### Step 4: Install Dependencies

From the `Python/` directory:

```bash
cd Python
pip install -r requirements.txt
```

### Step 5: Test the Connection

1. **Start Unity** and enter Play mode
2. **Run the test script**:

```bash
python test_robotic_arm.py
```

3. **Choose a test**:
   - Option 1: Quick test (~30 seconds)
   - Option 2: Full test suite (~60 seconds)

4. **Observe the robotic arm** in Unity:
   - It should respond to each command
   - Check Unity Console for debug messages
   - Each command should print confirmation

## Expected Behavior

### Quick Test
- Base rotates left and right
- Gripper opens and closes
- Arm returns to home position

### Full Test Suite
1. ✓ Arm moves to home position
2. ✓ Each joint moves individually (6 joints)
3. ✓ Gripper opens, closes, and sets to 50%
4. ✓ Arm moves through preset poses (pickup, forward, parked)
5. ✓ All joints move simultaneously
6. ✓ Arm returns to home

## Troubleshooting

### "Could not connect to Unity"

**Solution:**
- Make sure Unity is running and in Play mode
- Check that TcpServer is in the scene and active
- Verify port 5555 is not blocked by firewall

### "No response from arm"

**Check:**
1. RoboticArmNetworkController is attached to the arm
2. Robot ID in Unity matches the ID in Python (default: "arm_01")
3. Unity Console shows: `[arm_01] Connected to TcpServer`
4. Unity Console shows command messages when Python sends them

### "Joint not moving smoothly"

**Adjust settings** on RoboticArmController:
- Increase `jointSpeed` for faster movement (default: 90°/sec)
- Ensure `useSmoothMotion` is checked
- Check that joint limits aren't being exceeded

### "Script compilation errors in Unity"

**Common fixes:**
1. Make sure all scripts are in correct folders:
   - `RoboticArmController.cs` → `Assets/Scripts/`
   - `RoboticArmNetworkController.cs` → `Assets/Scripts/`
   - `TcpServer.cs` → `Assets/Scripts/Communication/`

2. Check namespace declarations match

## Using the API in Your Own Scripts

### Example: Simple Arm Control

```python
from unity_client import UnityClient
import time

# Connect to Unity
client = UnityClient()
client.connect()

# Control the arm
client.arm_go_home("arm_01")
time.sleep(2)

client.arm_set_joint("arm_01", joint_index=0, angle=45)
time.sleep(1)

client.arm_set_gripper("arm_01", open_amount=1.0)
time.sleep(1)

client.disconnect()
```

### Example: Complex Movement Sequence

```python
# Move to pickup position
client.arm_pose("arm_01", "pickup")
time.sleep(3)

# Close gripper to grab object
client.arm_set_gripper("arm_01", 0.2)
time.sleep(1)

# Move all joints to custom position
client.arm_set_joints("arm_01", [30, -45, 60, 0, -30, 90])
time.sleep(3)

# Open gripper to release
client.arm_set_gripper("arm_01", 1.0)
```

## Joint Reference

| Index | Joint Name | Axis | Range | Description |
|-------|------------|------|-------|-------------|
| 0 | Base (J1) | Y | -180° to 180° | Base rotation/yaw |
| 1 | Shoulder (J2) | X | -90° to 90° | Shoulder pitch |
| 2 | Elbow (J3) | X | -135° to 135° | Elbow pitch |
| 3 | Wrist Roll (J4) | Y | -180° to 180° | Wrist roll |
| 4 | Wrist Pitch (J5) | X | -120° to 120° | Wrist pitch |
| 5 | Tool Roll (J6) | Y | -180° to 180° | End effector roll |

## Preset Poses

| Pose Name | Description | Use Case |
|-----------|-------------|----------|
| `"pickup"` | Arm extended downward, gripper open | Ready to pick up objects |
| `"parked"` | Arm folded compactly, gripper closed | Safe storage position |
| `"forward"` | Arm extended straight forward | Reaching or placing objects |

## Next Steps

Once the basic arm control is working:

1. **Test concurrent control**: Run both truck and arm from the same Python script
2. **Mount arm on truck**: Position the arm as a child of the truck GameObject
3. **Coordinate movements**: Create sequences where truck drives and arm manipulates objects

## Advanced: Mounting Arm on Truck

To mount the arm on top of the truck (for later):

1. In Unity Hierarchy, drag the arm GameObject onto the truck GameObject
2. Adjust the arm's local position to sit on top of the truck
3. Now when the truck moves, the arm moves with it
4. You can control both independently from Python using their robot IDs

Example:
```python
# Move truck forward
client.move("truck_01", throttle=0.5, steering=0)

# While truck is moving, operate the arm
client.arm_pose("arm_01", "pickup")
```
