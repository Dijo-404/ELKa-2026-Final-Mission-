#!/usr/bin/env python3
"""
Test 1: Dual Drone Connection Test

Cross-platform compatible (Windows/Linux/macOS)
Auto-detects and identifies drones by battery voltage:
- Scout (4S): 14-17V
- Delivery (6S): 21-25V

Tests:
- Scans all available serial ports
- Connects and identifies each drone
- Displays battery, GPS, and system info
- Sets GUIDED mode on both
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


def scan_and_connect():
    """Scan all ports and connect to drones."""
    ports = get_serial_ports()
    
    if not ports:
        print("No serial ports found!")
        print("\nTroubleshooting:")
        if platform.system() == 'Windows':
            print("  - Check Device Manager for COM ports")
            print("  - Install drivers for your telemetry radio")
        else:
            print("  - Check if transmitters are plugged in: ls /dev/tty*")
            print("  - You may need permission: sudo usermod -a -G dialout $USER")
        return {}
    
    print(f"Found ports: {ports}")
    
    drones = {}
    
    for port in ports:
        print(f"\n[{port}] Scanning...")
        try:
            mav = mavutil.mavlink_connection(port, baud=57600)
            msg = mav.wait_heartbeat(timeout=5)
            
            if not msg:
                print(f"  No heartbeat")
                continue
            
            sysid = mav.target_system
            
            # Get battery
            batt = mav.recv_match(type='SYS_STATUS', blocking=True, timeout=2)
            voltage = batt.voltage_battery / 1000.0 if batt else 0
            
            # Identify drone
            drone_type = identify_drone(voltage)
            
            # Get GPS
            gps = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
            if gps and gps.lat != 0:
                lat, lon = gps.lat / 1e7, gps.lon / 1e7
                gps_status = f"({lat:.6f}, {lon:.6f})"
            else:
                gps_status = "No fix"
            
            print(f"  {drone_type} detected!")
            print(f"  Battery: {voltage:.1f}V")
            print(f"  GPS: {gps_status}")
            print(f"  System ID: {sysid}")
            
            drones[drone_type] = {
                'port': port,
                'mav': mav,
                'voltage': voltage,
                'sysid': sysid
            }
            
        except Exception as e:
            print(f"  Error: {str(e)[:50]}")
    
    return drones


def test_guided_mode(drones: dict):
    """Test setting GUIDED mode on both drones."""
    print("\n" + "="*50)
    print("TESTING GUIDED MODE")
    print("="*50)
    
    for name, info in drones.items():
        mav = info['mav']
        print(f"\n[{name}] Setting GUIDED mode...")
        
        mode_map = mav.mode_mapping()
        if 'GUIDED' in mode_map:
            mav.set_mode(mode_map['GUIDED'])
            time.sleep(0.5)
            print(f"[{name}] Mode set to GUIDED")
        else:
            print(f"[{name}] GUIDED mode not available")


def main():
    print("="*60)
    print("DUAL DRONE AUTO-DETECTION TEST")
    print(f"Platform: {platform.system()}")
    print("="*60)
    print("\nIdentification by battery voltage:")
    print("  Scout:    4S LiPo (14-17V)")
    print("  Delivery: 6S LiPo (21-25V)")
    
    # Scan and connect
    drones = scan_and_connect()
    
    if not drones:
        print("\nNo drones found!")
        return 1
    
    # Summary
    print("\n" + "="*60)
    print("DETECTION SUMMARY")
    print("="*60)
    
    for name in ['SCOUT', 'DELIVERY']:
        if name in drones:
            info = drones[name]
            print(f"  {name}: {info['port']} ({info['voltage']:.1f}V)")
        else:
            print(f"  {name}: NOT FOUND")
    
    # Check if both found
    if 'SCOUT' in drones and 'DELIVERY' in drones:
        print("\n  Both drones detected!")
        
        # Update config suggestion
        print("\n  Suggested config.py settings:")
        print(f"    SCOUT_CONNECTION = \"{drones['SCOUT']['port']}\"")
        print(f"    DELIVERY_CONNECTION = \"{drones['DELIVERY']['port']}\"")
    else:
        print("\n  WARNING: Not all drones detected!")
    
    # Test GUIDED mode
    if len(drones) > 0:
        response = input("\nTest GUIDED mode on connected drones? (y/n): ").strip().lower()
        if response == 'y':
            test_guided_mode(drones)
    
    # Cleanup
    print("\n" + "="*60)
    print("Disconnecting...")
    for name, info in drones.items():
        info['mav'].close()
        print(f"  [{name}] Disconnected")
    
    print("="*60)
    print("TEST COMPLETE")
    print("="*60)
    
    return 0 if len(drones) == 2 else 1


if __name__ == "__main__":
    sys.exit(main())
