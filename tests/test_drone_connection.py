#!/usr/bin/env python3
"""
Test 1: Dual Drone Connection Test

Tests:
- Connect to both Scout and Delivery drones
- Set both to GUIDED mode
- Display connection status and GPS info
"""

import sys
import time
import logging

sys.path.insert(0, '..')
from config import MissionConfig
from drone_controller import DroneController

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger('DroneConnectionTest')


def test_drone_connection(name: str, connection: str, baud: int) -> bool:
    """
    Test connection to a single drone.
    
    Args:
        name: Drone name
        connection: Connection string
        baud: Baud rate
        
    Returns:
        True if all tests pass
    """
    print(f"\n{'='*60}")
    print(f"Testing: {name}")
    print(f"Connection: {connection}")
    print('='*60)
    
    drone = DroneController(connection, baud=baud, name=name)
    
    # Test 1: Connect
    print("\n[1] Connecting...")
    if not drone.connect(timeout=15.0):
        print("    FAILED: Could not connect")
        return False
    print("    PASSED: Connected")
    
    # Test 2: Get GPS
    print("\n[2] Getting GPS position...")
    location = drone.get_location()
    if location:
        lat, lon, alt = location
        print(f"    PASSED: Position ({lat:.6f}, {lon:.6f}, {alt:.1f}m)")
    else:
        print("    WARNING: No GPS position (may need time to acquire)")
    
    # Test 3: Check battery
    print("\n[3] Checking battery...")
    battery_ok, voltage, percent = drone.check_battery()
    if voltage > 0:
        status = "OK" if battery_ok else "LOW"
        print(f"    {status}: {voltage:.1f}V, {percent}%")
    else:
        print("    WARNING: Could not read battery")
    
    # Test 4: Set GUIDED mode
    print("\n[4] Setting GUIDED mode...")
    if drone.set_mode('GUIDED'):
        print("    PASSED: Mode set to GUIDED")
    else:
        print("    FAILED: Could not set GUIDED mode")
        drone.disconnect()
        return False
    
    # Test 5: Verify mode
    print("\n[5] Verifying mode...")
    time.sleep(1)
    print("    Mode verification requires heartbeat check")
    
    # Cleanup
    print("\n[6] Disconnecting...")
    drone.disconnect()
    print("    PASSED: Disconnected")
    
    return True


def main():
    print("="*60)
    print("DUAL DRONE CONNECTION TEST")
    print("="*60)
    
    config = MissionConfig()
    
    print(f"\nScout connection: {config.SCOUT_CONNECTION}")
    print(f"Delivery connection: {config.DELIVERY_CONNECTION}")
    
    # Prompt for confirmation
    print("\nMake sure both telemetry radios are connected.")
    input("Press ENTER to start tests...")
    
    results = {}
    
    # Test Scout drone
    results['Scout'] = test_drone_connection(
        config.SCOUT_NAME,
        config.SCOUT_CONNECTION,
        config.SCOUT_BAUD
    )
    
    # Test Delivery drone
    results['Delivery'] = test_drone_connection(
        config.DELIVERY_NAME,
        config.DELIVERY_CONNECTION,
        config.DELIVERY_BAUD
    )
    
    # Summary
    print("\n" + "="*60)
    print("TEST SUMMARY")
    print("="*60)
    
    all_passed = True
    for name, passed in results.items():
        status = "PASSED" if passed else "FAILED"
        print(f"  {name}: {status}")
        if not passed:
            all_passed = False
    
    print("="*60)
    
    if all_passed:
        print("\nAll tests PASSED - Ready for mission")
    else:
        print("\nSome tests FAILED - Check connections")
    
    return 0 if all_passed else 1


if __name__ == "__main__":
    sys.exit(main())
