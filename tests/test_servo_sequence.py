#!/usr/bin/env python3
"""
Servo Test - Sequential Payload Drop Test

Tests all 6 payload servos (AUX1-6 / SERVO9-14) one at a time.
Use this to verify your payload drop mechanism on the ground.

SAFETY: Run this test with the drone DISARMED and on the ground!

Usage:
    python test_servo_sequence.py                    # Use default connection
    python test_servo_sequence.py --port /dev/ttyACM0
    python test_servo_sequence.py --port COM3        # Windows
"""

import argparse
import sys
import time

# Add parent directory to path for imports
sys.path.insert(0, '..')

from config import MissionConfig
from drone_controller import DroneController


def test_servo_sequence(connection: str, baud: int = 57600):
    """
    Test all 6 payload servos sequentially.
    
    Args:
        connection: MAVLink connection string
        baud: Baud rate for serial connection
    """
    config = MissionConfig()
    
    print()
    print("=" * 60)
    print("  SERVO SEQUENCE TEST - 6 PAYLOAD DROPS")
    print("=" * 60)
    print()
    print("  This test will trigger each servo one at a time:")
    print()
    for i, channel in enumerate(config.PAYLOAD_SERVO_CHANNELS):
        print(f"    Slot {i+1}: SERVO{channel} (AUX{channel - 8})")
    print()
    print(f"  Drop PWM:   {config.DROP_SERVO_PWM}")
    print(f"  Closed PWM: {config.LOAD_SERVO_PWM}")
    print(f"  Hold time:  {config.DROP_DURATION}s per servo")
    print()
    print("=" * 60)
    print()
    
    # Safety warning
    print("!" * 60)
    print("  SAFETY WARNING")
    print("!" * 60)
    print("  - Ensure drone is DISARMED")
    print("  - Keep hands clear of payload mechanism")
    print("  - Have LiPo connected for servo power")
    print("!" * 60)
    print()
    
    response = input("Press ENTER to continue (or 'q' to quit): ").strip().lower()
    if response == 'q':
        print("Test cancelled.")
        return
    
    # Connect to drone
    print(f"\nConnecting to {connection}...")
    drone = DroneController(connection, baud=baud, name="ServoTest")
    
    if not drone.connect(timeout=10.0):
        print("ERROR: Failed to connect to flight controller!")
        print("Check your connection string and ensure FC is powered.")
        return
    
    print("Connected successfully!")
    print()
    
    # Reset all servos to closed position first
    print("Resetting all servos to CLOSED position...")
    for channel in config.PAYLOAD_SERVO_CHANNELS:
        drone.set_servo(channel, config.LOAD_SERVO_PWM)
        time.sleep(0.1)
    print("All servos reset to closed.\n")
    
    time.sleep(1.0)
    
    # Test each servo
    print("=" * 60)
    print("  STARTING SERVO SEQUENCE TEST")
    print("=" * 60)
    print()
    
    for i, channel in enumerate(config.PAYLOAD_SERVO_CHANNELS):
        slot_num = i + 1
        aux_num = channel - 8
        
        print(f"[{slot_num}/6] Testing SERVO{channel} (AUX{aux_num})...")
        
        # Prompt before each drop
        input(f"  Press ENTER to trigger payload {slot_num}... ")
        
        # Trigger servo to drop position
        print(f"  -> Opening (PWM: {config.DROP_SERVO_PWM})...")
        drone.set_servo(channel, config.DROP_SERVO_PWM)
        
        # Hold for drop duration
        time.sleep(config.DROP_DURATION)
        
        # Return to closed position
        print(f"  -> Closing (PWM: {config.LOAD_SERVO_PWM})...")
        drone.set_servo(channel, config.LOAD_SERVO_PWM)
        
        print(f"  -> Payload {slot_num} complete!")
        print()
        
        time.sleep(0.5)
    
    print("=" * 60)
    print("  SERVO SEQUENCE TEST COMPLETE")
    print("=" * 60)
    print()
    print("All 6 servos tested successfully!")
    print()
    
    # Offer to run automatic sequence
    print("-" * 60)
    response = input("Run automatic sequence (all 6 without prompts)? [y/N]: ").strip().lower()
    
    if response == 'y':
        print("\nRunning automatic sequence in 3 seconds...")
        time.sleep(3)
        
        for i, channel in enumerate(config.PAYLOAD_SERVO_CHANNELS):
            slot_num = i + 1
            print(f"[{slot_num}/6] SERVO{channel} -> DROP", end="", flush=True)
            drone.set_servo(channel, config.DROP_SERVO_PWM)
            time.sleep(config.DROP_DURATION)
            drone.set_servo(channel, config.LOAD_SERVO_PWM)
            print(" -> CLOSED")
            time.sleep(0.3)
        
        print("\nAutomatic sequence complete!")
    
    # Reset all servos
    print("\nResetting all servos to CLOSED...")
    for channel in config.PAYLOAD_SERVO_CHANNELS:
        drone.set_servo(channel, config.LOAD_SERVO_PWM)
        time.sleep(0.1)
    
    print("All servos reset.")
    
    # Disconnect
    drone.disconnect()
    print("\nTest complete. Disconnected from flight controller.")


def main():
    parser = argparse.ArgumentParser(
        description='Test 6 payload servos sequentially',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    
    parser.add_argument(
        '--port', '-p',
        default=None,
        help='Connection string (e.g., /dev/ttyACM0, COM3, udpin:127.0.0.1:14550)'
    )
    parser.add_argument(
        '--baud', '-b',
        type=int,
        default=57600,
        help='Baud rate for serial connection (default: 57600)'
    )
    
    args = parser.parse_args()
    
    # Use config default if no port specified
    if args.port is None:
        config = MissionConfig()
        connection = config.DELIVERY_CONNECTION
        print(f"Using default connection from config: {connection}")
    else:
        connection = args.port
    
    try:
        test_servo_sequence(connection, args.baud)
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user.")
    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
