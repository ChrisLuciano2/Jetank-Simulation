# Quick Start Guide

Get up and running in 5 minutes!

## Step 1: Open Unity Project

1. Open Unity Hub
2. Add project: `AIT-Robot-Simulator/Robots`
3. Open the project

## Step 2: Auto-Setup Scene

In Unity Editor:
1. Go to menu: **Tools > Setup Robot Simulator Scene**
2. Click "OK" on the dialog
3. Save scene: **Ctrl+S** (Windows) or **Cmd+S** (Mac)

## Step 3: Enable Run In Background

1. Edit > Project Settings > Player
2. Expand "Resolution and Presentation"
3. Check **"Run In Background"**
4. Close settings window

## Step 4: Start Unity

Press the **Play** button ▶️ in Unity Editor

**Verify**: Unity Console should show:
```
[TcpServer] Started on port 5555
[truck_01] TruckController started
[truck_01] Connected to TcpServer
```

## Step 5: Run Python Test

Open Terminal/Command Prompt:

```bash
cd AIT-Robot-Simulator/Python
python3 unity_client.py
```

**Expected Output:**
```
Connected to Unity at 127.0.0.1:5555

=== Robot Control Demo ===
Commands will be sent to Unity...

Moving forward...
Turning right...
Moving backward...
Stopping...
Going to position (5, 5)...

Demo complete!
Disconnected from Unity
```

## ✅ Success!

You should see a **blue cube truck** moving in Unity!

- If the truck doesn't move, **click on the Unity window** to bring it to focus
- Check Unity Console for any error messages

## Next Steps

- Read [README.md](README.md) for full documentation
- See [SETUP_CHECKLIST.md](SETUP_CHECKLIST.md) for troubleshooting
- Modify `unity_client.py` to create custom behaviors
- Add your own truck models in Unity

---

**Need Help?**
- Unity Console has error messages? Check SETUP_CHECKLIST.md
- Connection refused? Make sure Unity is in Play mode first
- Truck not visible? Double-click "Truck_01" in Unity Hierarchy
