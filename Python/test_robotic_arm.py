"""
Test script for controlling the robotic arm from Python.

Make sure Unity is running with:
1. TcpServer in the scene
2. Robotic arm with both RoboticArmController and RoboticArmNetworkController attached
"""

import time
from unity_client import UnityClient


def test_arm_movements(client: UnityClient, arm_id: str = "arm_01"):
    """Test various robotic arm movements."""

    print("\n=== Robotic Arm Control Test ===\n")

    # Test 1: Move to home position
    print("Test 1: Moving to home position...")
    client.arm_go_home(arm_id)
    time.sleep(3)

    # Test 2: Test individual joint movements
    print("\nTest 2: Testing individual joints...")

    print("  - Rotating base (J1) to 45°...")
    client.arm_set_joint(arm_id, joint_index=0, angle=45)
    time.sleep(2)

    print("  - Moving shoulder (J2) to -30°...")
    client.arm_set_joint(arm_id, joint_index=1, angle=-30)
    time.sleep(2)

    print("  - Moving elbow (J3) to 60°...")
    client.arm_set_joint(arm_id, joint_index=2, angle=60)
    time.sleep(2)

    print("  - Rotating wrist roll (J4) to 90°...")
    client.arm_set_joint(arm_id, joint_index=3, angle=90)
    time.sleep(2)

    print("  - Moving wrist pitch (J5) to -20°...")
    client.arm_set_joint(arm_id, joint_index=4, angle=-20)
    time.sleep(2)

    print("  - Rotating tool (J6) to 180°...")
    client.arm_set_joint(arm_id, joint_index=5, angle=180)
    time.sleep(2)

    # Test 3: Reset to home
    print("\nTest 3: Returning to home...")
    client.arm_go_home(arm_id)
    time.sleep(3)

    # Test 4: Test gripper
    print("\nTest 4: Testing gripper...")
    print("  - Opening gripper fully...")
    client.arm_set_gripper(arm_id, open_amount=1.0)
    time.sleep(2)

    print("  - Closing gripper...")
    client.arm_set_gripper(arm_id, open_amount=0.0)
    time.sleep(2)

    print("  - Setting gripper to 50%...")
    client.arm_set_gripper(arm_id, open_amount=0.5)
    time.sleep(2)

    # Test 5: Test preset poses
    print("\nTest 5: Testing preset poses...")

    print("  - Moving to pickup pose...")
    client.arm_pose(arm_id, "pickup")
    time.sleep(3)

    print("  - Moving to reach forward pose...")
    client.arm_pose(arm_id, "forward")
    time.sleep(3)

    print("  - Moving to parked pose...")
    client.arm_pose(arm_id, "parked")
    time.sleep(3)

    # Test 6: Set all joints simultaneously
    print("\nTest 6: Setting all joints simultaneously...")
    angles = [30, -45, 70, -30, 25, 90]
    print(f"  - Setting joints to: {angles}")
    client.arm_set_joints(arm_id, angles)
    time.sleep(3)

    # Final: Return to home
    print("\nReturning to home position...")
    client.arm_go_home(arm_id)
    time.sleep(2)

    print("\n=== Test Complete! ===")


def quick_test(client: UnityClient, arm_id: str = "arm_01"):
    """Quick test to verify arm responds to commands."""

    print("\n=== Quick Arm Test ===\n")

    print("1. Moving base to 45°...")
    client.arm_set_joint(arm_id, 0, 45)
    time.sleep(2)

    print("2. Moving base to -45°...")
    client.arm_set_joint(arm_id, 0, -45)
    time.sleep(2)

    print("3. Returning to home...")
    client.arm_go_home(arm_id)
    time.sleep(2)

    print("4. Opening gripper...")
    client.arm_set_gripper(arm_id, 1.0)
    time.sleep(1)

    print("5. Closing gripper...")
    client.arm_set_gripper(arm_id, 0.0)
    time.sleep(1)

    print("\n=== Quick Test Complete! ===")


def main():
    """Run robotic arm tests."""

    # Create client and connect
    client = UnityClient()

    if not client.connect():
        print("Could not connect to Unity. Make sure:")
        print("1. Unity is running")
        print("2. The scene has a TcpServer GameObject")
        print("3. The robotic arm has RoboticArmNetworkController attached")
        return

    try:
        # Choose which test to run
        print("\nSelect test to run:")
        print("1. Quick test (30 seconds)")
        print("2. Full test suite (60+ seconds)")

        choice = input("Enter choice (1 or 2): ").strip()

        if choice == "1":
            quick_test(client)
        else:
            test_arm_movements(client)

    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\nError during test: {e}")
    finally:
        # Return arm to home position before disconnecting
        print("\nReturning arm to home position...")
        client.arm_go_home("arm_01")
        time.sleep(1)
        client.disconnect()


if __name__ == "__main__":
    main()
