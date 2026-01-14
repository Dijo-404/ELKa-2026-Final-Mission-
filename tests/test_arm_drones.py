#!/usr/bin/env python3
"""
Test: Arm Both Drones in Stabilize Mode

Cross-platform compatible (Windows/Linux/macOS)
Auto-detects drones by battery voltage:
- Scout (4S): 14-17V  
- Delivery (6S): 21-25V

This script:
1. Auto-detects and connects to both drones
2. Sets STABILIZE mode
3. Force arms both drones (bypasses pre-arm checks)
4. Applies 10% throttle for 3 seconds
5. Force disarms and exits

WARNING: Props will spin! Ensure drones are secured or props removed.
"""

import sys
import time
import platform

sys.path.insert(0, '..')
from pymavlink import mavutil


def get_serial_ports():
    """
    Get list of available serial ports (cross-platform).
    Works on Windows, Linux, and macOS.
    """
    ports = []
    system = platform.system()
    
    if system == 'Windows':
        # Windows: COM1, COM2, etc.
        import serial.tools.list_ports
        for port in serial.tools.list_ports.comports():
            ports.append(port.device)
    else:
        # Linux/macOS: /dev/tty*
        import glob
        # Linux
        ports.extend(glob.glob('/dev/ttyACM*'))
        ports.extend(glob.glob('/dev/ttyUSB*'))
        # macOS
        ports.extend(glob.glob('/dev/tty.usbserial*'))
        ports.extend(glob.glob('/dev/tty.usbmodem*'))
    
    return sorted(ports)


def identify_drone(voltage: float) -> str:
    """Identify drone type by battery voltage."""
    if voltage > 20:
        return "DELIVERY"  # 6S battery
    elif voltage > 10:
        return "SCOUT"     # 4S battery
    else:
        return "UNKNOWN"


def scan_drones():
    """Scan all ports and identify drones by voltage."""
    ports = get_serial_ports()
    
    if not ports:
        print("No serial ports found!")
        print("\nTroubleshooting:")
        if platform.system() == 'Windows':
            print("  - Check Device Manager for COM ports")
            print("  - Install drivers for your telemetry radio")
            print("  - Run: python -m serial.tools.list_ports")
        else:
            print("  - Check if transmitters are plugged in: ls /dev/tty*")
            print("  - You may need permission: sudo usermod -a -G dialout $USER")
        return {}
    
    print(f"Scanning ports: {ports}")
    drones = {}
    
    for port in ports:
        try:
            mav = mavutil.mavlink_connection(port, baud=57600)
            msg = mav.wait_heartbeat(timeout=5)
            
            if not msg:
                continue
            
            batt = mav.recv_match(type='SYS_STATUS', blocking=True, timeout=2)
            voltage = batt.voltage_battery / 1000.0 if batt else 0
            
            drone_type = identify_drone(voltage)
            print(f"  {port}: {drone_type} ({voltage:.1f}V)")
            
            drones[drone_type] = {'mav': mav, 'port': port, 'voltage': voltage}
            
        except Exception as e:
            print(f"  {port}: Error - {str(e)[:30]}")
    
    return drones


def set_mode(mav, mode: str, name: str) -> bool:
    """Set flight mode."""
    mode_map = mav.mode_mapping()
    if mode not in mode_map:
        print(f"[{name}] Unknown mode: {mode}")
        return False
    
    mav.set_mode(mode_map[mode])
    time.sleep(0.5)
    print(f"[{name}] Mode: {mode}")
    return True


def arm_drone(mav, name: str) -> bool:
    """Force arm the drone (bypasses pre-arm checks)."""
    print(f"[{name}] Force arming...")
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,      # confirmation
        1,      # arm
        21196,  # force arm magic number (bypasses pre-arm checks)
        0, 0, 0, 0, 0
    )
    
    msg = mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=3.0)
    if msg and msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print(f"[{name}] ARMED (force)")
        return True
    else:
        print(f"[{name}] Arm failed")
        return False


def disarm_drone(mav, name: str):
    """Force disarm the drone."""
    # First set throttle to zero
    mav.mav.rc_channels_override_send(
        mav.target_system,
        mav.target_component,
        0, 0, 1000, 0, 0, 0, 0, 0  # 1000 = zero throttle
    )
    time.sleep(0.3)
    
    # Force disarm (param2=21196 for force)
    print(f"[{name}] Force disarming...")
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,      # confirmation
        0,      # disarm
        21196,  # force disarm magic number
        0, 0, 0, 0, 0
    )
    
    msg = mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=2.0)
    if msg and msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print(f"[{name}] DISARMED")
    else:
        print(f"[{name}] Disarm may have failed - check drone")
    
    # Release RC override
    mav.mav.rc_channels_override_send(
        mav.target_system,
        mav.target_component,
        0, 0, 0, 0, 0, 0, 0, 0
    )


def set_throttle(mav, throttle_percent: int):
    """Set RC throttle override."""
    pwm = 1000 + int(10 * throttle_percent)
    mav.mav.rc_channels_override_send(
        mav.target_system,
        mav.target_component,
        0, 0, pwm, 0, 0, 0, 0, 0
    )


def release_throttle(mav):
    """Set throttle to zero (1000 PWM)."""
    mav.mav.rc_channels_override_send(
        mav.target_system,
        mav.target_component,
        0, 0, 1000, 0, 0, 0, 0, 0  # 1000 = minimum/zero throttle
    )


def main():
    print("="*60)
    print("DUAL DRONE ARM TEST - STABILIZE + 10% THROTTLE")
    print(f"Platform: {platform.system()}")
    print("="*60)
    print("\nWARNING: This will spin props!")
    print("Ensure drones are secured or props removed.\n")
    
    # Scan and detect drones
    print("Detecting drones by battery voltage...")
    drones = scan_drones()
    
    if not drones:
        print("\nNo drones found!")
        return 1
    
    # Summary
    print("\nDetected:")
    for name in ['SCOUT', 'DELIVERY']:
        if name in drones:
            print(f"  {name}: {drones[name]['port']} ({drones[name]['voltage']:.1f}V)")
        else:
            print(f"  {name}: NOT FOUND")
    
    if len(drones) < 2:
        print("\nWARNING: Not all drones detected!")
    
    # Confirm
    response = input("\nType 'YES' to arm and test: ").strip()
    if response != 'YES':
        print("Aborted.")
        for info in drones.values():
            info['mav'].close()
        return 1
    
    try:
        # Set STABILIZE mode
        print("\n--- Setting STABILIZE mode ---")
        for name, info in drones.items():
            set_mode(info['mav'], 'STABILIZE', name)
        
        time.sleep(1)
        
        # Arm
        print("\n--- Force Arming ---")
        armed = []
        for name, info in drones.items():
            if arm_drone(info['mav'], name):
                armed.append(name)
        
        if not armed:
            print("\nNo drones armed!")
            return 1
        
        # Throttle
        print("\n--- Applying 10% throttle for 3 seconds ---")
        for name in armed:
            set_throttle(drones[name]['mav'], 10)
        
        for i in range(3, 0, -1):
            print(f"  {i}...")
            time.sleep(1)
        
        # Release
        print("\n--- Releasing throttle ---")
        for name in armed:
            release_throttle(drones[name]['mav'])
        
        time.sleep(0.5)
        
        # Disarm
        print("\n--- Force Disarming ---")
        for name in armed:
            disarm_drone(drones[name]['mav'], name)
    
    except KeyboardInterrupt:
        print("\n\nInterrupted! Emergency disarm...")
        for name, info in drones.items():
            release_throttle(info['mav'])
            disarm_drone(info['mav'], name)
    
    finally:
        for info in drones.values():
            info['mav'].close()
    
    print("\n" + "="*60)
    print("TEST COMPLETE")
    print("="*60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
