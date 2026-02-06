# AIT Robot Simulator

A basic Unity-Python robot simulator for testing truck control via TCP communication.

## Features

- **TCP Communication**: Python client connects to Unity via TCP (port 5555)
- **Basic Truck Control**: Move, turn, goto position, teleport
- **Simple Architecture**: Easy to understand and extend
- **Quick Setup**: Automated scene setup via Unity Editor script

## Project Structure

```
AIT-Robot-Simulator/
├── Python/
│   ├── unity_client.py          # Python client with UnityClient class
│   └── requirements.txt         # Python dependencies (none required)
├── Robots/                      # Unity project
│   └── Assets/
│       ├── Scenes/
│       │   └── MainScene.unity  # Main test scene
│       └── Scripts/
│           ├── TruckController.cs         # Truck movement controller
│           ├── Communication/
│           │   └── TcpServer.cs          # TCP server for Python communication
│           └── Editor/
│               └── SceneSetup.cs         # Auto-setup utility
└── README.md
```

## Quick Start

### 1. Unity Setup

1. Open the `Robots` project in Unity Editor
2. Go to **Tools > Setup Robot Simulator Scene**
3. Save the scene (Ctrl+S / Cmd+S)
4. **Enable Run In Background**: Edit > Project Settings > Player > Resolution and Presentation > Check "Run In Background"
5. Press **Play**

### 2. Run Python Client

```bash
cd Python
python3 unity_client.py
```

You should see the blue cube truck moving in Unity!

## Commands

The `UnityClient` class provides these methods:

| Method | Description |
|--------|-------------|
| `connect()` | Connect to Unity server |
| `disconnect()` | Disconnect from server |
| `move(robot_id, throttle, steering)` | Control throttle (-1 to 1) and steering (-1 to 1) |
| `stop(robot_id)` | Stop the robot |
| `goto(robot_id, x, z)` | Navigate to position |
| `set_position(robot_id, x, y, z)` | Teleport to position |
| `set_rotation(robot_id, rotation_y)` | Set rotation in degrees |

## Python Example

```python
from unity_client import UnityClient

client = UnityClient()
if client.connect():
    # Move forward
    client.move("truck_01", throttle=0.5, steering=0.0)

    # Turn right
    client.move("truck_01", throttle=0.5, steering=0.5)

    # Go to position
    client.goto("truck_01", x=10.0, z=5.0)

    # Stop
    client.stop("truck_01")

    client.disconnect()
```

## Unity C# API

### TcpServer

Receives and processes commands from Python. Automatically starts on port 5555.

### TruckController

Handles robot movement based on received commands. Supports:
- Direct control (throttle/steering)
- Position-based navigation (goto)
- Teleportation (set_position/set_rotation)

## Troubleshooting

### Truck doesn't move
- **Unity not focused**: Click on Unity window after starting Python script
- **Enable Run In Background**: Edit > Project Settings > Player > Resolution and Presentation
- **Check Console**: Look for "[TcpServer] Client connected" and "[truck_01] Connected to TcpServer"

### Connection refused
- **Unity not running**: Press Play in Unity Editor first
- **Port already in use**: Check if another instance is running

### Truck not in scene
- Run **Tools > Setup Robot Simulator Scene** in Unity Editor
- Or manually add TcpServer GameObject and Truck with TruckController component

## Next Steps

This is a minimal baseline for testing. To extend:
1. Clone this repository for your expanded project
2. Add custom truck models (replace the primitive cube)
3. Implement sensors (raycasts, collision detection)
4. Add multi-robot support
5. Create task-based behaviors (state machines, pathfinding)

## License

MIT License - Free to use and modify for educational purposes.
