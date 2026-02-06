"""
Control both the truck and robotic arm together.
Demonstrates coordinated movement.
"""

import time
from unity_client import UnityClient


def demo_coordinated_control(client: UnityClient):
    """Demo: Drive truck while operating the arm."""

    print("\n=== Truck + Arm Coordinated Control Demo ===\n")

    # Initialize both to safe positions
    print("1. Initializing truck and arm...")
    client.stop("truck_01")
    client.arm_go_home("arm_01")
    time.sleep(2)

    # Drive forward while moving arm to pickup position
    print("\n2. Driving forward while arm moves to pickup pose...")
    client.move("truck_01", throttle=0.3, steering=0)
    client.arm_pose("arm_01", "pickup")
    time.sleep(3)

    # Stop truck, close gripper (simulate picking up object)
    print("\n3. Stopping truck and closing gripper...")
    client.stop("truck_01")
    client.arm_set_gripper("arm_01", 0.0)  # Close gripper
    time.sleep(2)

    # Move arm to parked position
    print("\n4. Moving arm to parked position...")
    client.arm_pose("arm_01", "parked")
    time.sleep(3)

    # Drive to new location
    print("\n5. Driving to new location...")
    client.move("truck_01", throttle=0.4, steering=0)
    time.sleep(2)

    # Turn right
    print("\n6. Turning right...")
    client.move("truck_01", throttle=0.3, steering=0.5)
    time.sleep(1.5)

    # Stop and deploy arm
    print("\n7. Stopping and deploying arm...")
    client.stop("truck_01")
    client.arm_pose("arm_01", "forward")
    time.sleep(2)

    # Open gripper (simulate placing object)
    print("\n8. Opening gripper...")
    client.arm_set_gripper("arm_01", 1.0)
    time.sleep(1)

    # Return arm to home
    print("\n9. Returning arm to home...")
    client.arm_go_home("arm_01")
    time.sleep(2)

    # Back up truck
    print("\n10. Backing up truck...")
    client.move("truck_01", throttle=-0.3, steering=0)
    time.sleep(2)

    # Final stop
    print("\n11. Stopping all...")
    client.stop("truck_01")
    time.sleep(0.5)

    print("\n=== Demo Complete! ===")


def interactive_control(client: UnityClient):
    """Interactive control mode for testing."""

    print("\n=== Interactive Control Mode ===")
    print("\nCommands:")
    print("  Truck: w/s (forward/back), a/d (turn left/right), x (stop)")
    print("  Arm: 0-5 (move joint), g (gripper), h (home), p (pickup pose)")
    print("  q (quit)")
    print()

    while True:
        cmd = input("Command: ").strip().lower()

        if cmd == 'q':
            break
        elif cmd == 'w':
            client.move("truck_01", 0.5, 0)
            print("Truck moving forward")
        elif cmd == 's':
            client.move("truck_01", -0.3, 0)
            print("Truck moving backward")
        elif cmd == 'a':
            client.move("truck_01", 0.3, -0.7)
            print("Truck turning left")
        elif cmd == 'd':
            client.move("truck_01", 0.3, 0.7)
            print("Truck turning right")
        elif cmd == 'x':
            client.stop("truck_01")
            print("Truck stopped")
        elif cmd == 'h':
            client.arm_go_home("arm_01")
            print("Arm moving to home")
        elif cmd == 'p':
            client.arm_pose("arm_01", "pickup")
            print("Arm moving to pickup pose")
        elif cmd == 'g':
            amount = input("Gripper amount (0-1): ")
            client.arm_set_gripper("arm_01", float(amount))
        elif cmd in '012345':
            angle = input(f"Angle for joint {cmd}: ")
            client.arm_set_joint("arm_01", int(cmd), float(angle))
            print(f"Joint {cmd} moving to {angle}°")
        else:
            print("Unknown command")


def main():
    """Run combined truck and arm demo."""

    client = UnityClient()

    if not client.connect():
        print("Could not connect to Unity.")
        print("Make sure Unity is running with both truck and arm in the scene.")
        return

    try:
        print("\nSelect mode:")
        print("1. Coordinated demo (automated)")
        print("2. Interactive control")

        choice = input("Choice (1 or 2): ").strip()

        if choice == "1":
            demo_coordinated_control(client)
        else:
            interactive_control(client)

    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        print("\nStopping all robots...")
        client.stop("truck_01")
        client.arm_go_home("arm_01")
        time.sleep(0.5)
        client.disconnect()


if __name__ == "__main__":
    main()
