# Setup Checklist

Use this checklist to verify your robot simulator is correctly configured.

## ✅ Initial Setup (One-time)

### Unity Setup
- [ ] Unity project opens without errors
- [ ] Run **Tools > Setup Robot Simulator Scene**
- [ ] Save the scene (Ctrl+S / Cmd+S)
- [ ] Enable **Run In Background**:
  - [ ] Edit > Project Settings > Player
  - [ ] Resolution and Presentation section
  - [ ] Check "Run In Background"

### Scene Verification
- [ ] Scene contains **TcpServer** GameObject
- [ ] Scene contains **Truck_01** GameObject (blue cube)
- [ ] Truck_01 has **TruckController** component attached
- [ ] TruckController settings:
  - [ ] Move Speed: 5
  - [ ] Turn Speed: 90
  - [ ] Acceleration: 10
  - [ ] Robot ID: "truck_01"

### Python Setup
- [ ] Navigate to `Python/` directory
- [ ] Verify `unity_client.py` exists
- [ ] Python 3 is installed (`python3 --version`)

## ✅ Before Each Test Session

### Unity
- [ ] Unity Editor is open with `MainScene`
- [ ] No console errors (check Console tab)
- [ ] Press **Play** button
- [ ] Console shows: `[TcpServer] Started on port 5555`
- [ ] Console shows: `[truck_01] TruckController started`
- [ ] Console shows: `[truck_01] Connected to TcpServer`

### Python
- [ ] Terminal/Command Prompt is open
- [ ] Current directory is `Python/`
- [ ] Ready to run: `python3 unity_client.py`

## ✅ Test Run Verification

### Run Test
- [ ] Unity is in Play mode
- [ ] Run: `python3 unity_client.py`
- [ ] Python shows: `Connected to Unity at 127.0.0.1:5555`

### Expected Behavior
- [ ] Unity Console shows: `[TcpServer] Client connected`
- [ ] Python shows demo messages (Moving forward, Turning right, etc.)
- [ ] **Blue cube truck moves in Unity Scene/Game view**
- [ ] Truck moves forward for 2 seconds
- [ ] Truck turns right while moving
- [ ] Truck moves backward
- [ ] Truck stops
- [ ] Truck navigates to position (5, 5)
- [ ] Python shows: `Demo complete!`

### Verify Unity State
- [ ] Truck is at a different position than start
- [ ] No errors in Unity Console
- [ ] Python disconnected cleanly

## 🚨 Common Issues

### Issue: "Connection refused"
**Fix:** Press Play in Unity first, then run Python script

### Issue: Truck doesn't move
**Fix:** Click on Unity window to bring it to focus

### Issue: "TcpServer not found"
**Fix:** Run **Tools > Setup Robot Simulator Scene** and save

### Issue: No blue cube visible
**Fix:** Check Scene view (not just Game view). If missing, run setup script again.

### Issue: Truck moves but can't see it
**Fix:** In Scene view, double-click "Truck_01" in Hierarchy to focus camera on it

## ✅ Ready for Development

If all items are checked, your simulator is ready! You can now:
- [ ] Modify `unity_client.py` for custom behaviors
- [ ] Add your own truck models in Unity
- [ ] Extend `TruckController.cs` with new commands
- [ ] Create multi-robot scenarios

---

**Last Updated:** 2026-02-05
