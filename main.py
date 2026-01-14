#!/usr/bin/env python3
"""
Multi-Drone Mission Orchestrator

Main entry point for the dual-drone mission:
1. Scout drone surveys area and detects humans
2. Delivery drone drops payloads at detected locations

Usage:
    python main.py                      # Full mission
    python main.py --scout-only         # Scout mission only
    python main.py --delivery-only      # Delivery mission only
    python main.py --simulate           # Simulation mode (no real drones)
"""

import argparse
import logging
import sys
import time
from datetime import datetime

from config import MissionConfig, get_config
from scout_mission import ScoutMission
from delivery_mission import DeliveryMission

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger('MissionOrchestrator')


def print_banner():
    """Print mission banner."""
    print()
    print("=" * 70)
    print("  MULTI-DRONE MISSION ORCHESTRATOR")
    print("  ELKa 2026 Final Mission")
    print("=" * 70)
    print(f"  Start Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 70)
    print()


def print_config(config: MissionConfig):
    """Print current configuration."""
    print("CONFIGURATION")
    print("-" * 50)
    print(f"  Scout Connection:    {config.SCOUT_CONNECTION}")
    print(f"  Delivery Connection: {config.DELIVERY_CONNECTION}")
    print(f"  Scout Altitude:      {config.SCOUT_ALTITUDE}m")
    print(f"  Delivery Altitude:   {config.DELIVERY_ALTITUDE}m")
    print(f"  Payload Capacity:    {config.PAYLOAD_CAPACITY}")
    print(f"  KML File:            {config.KML_FILE}")
    print(f"  RTSP URL:            {config.RTSP_URL}")
    print("-" * 50)
    print()


# =============================================================================
# SAFETY CRITICAL: Pre-Flight Status and Confirmation
# =============================================================================

def print_failsafe_warning():
    """Print critical failsafe parameter reminder."""
    print()
    print("+" + "=" * 78 + "+")
    print("|" + " FAILSAFE PARAMETER CHECK - VERIFY BEFORE FLIGHT ".center(78) + "|")
    print("+" + "=" * 78 + "+")
    print("|" + "".center(78) + "|")
    print("|" + "  VERIFY THESE PARAMETERS IN MISSION PLANNER / QGROUNDCONTROL:".ljust(78) + "|")
    print("|" + "".center(78) + "|")
    print("|" + "    FS_GCS_ENABL  = 3  (RTL on GCS connection loss)".ljust(78) + "|")
    print("|" + "    FS_BATT_ENABLE = 1  (Land on critical battery)".ljust(78) + "|")
    print("|" + "    FS_THR_ENABLE  = 1  (RTL on RC signal loss)".ljust(78) + "|")
    print("|" + "".center(78) + "|")
    print("|" + "  WARNING: If USB cable disconnects, FS_GCS_ENABL triggers RTL!".ljust(78) + "|")
    print("|" + "  This is your ONLY protection against a flyaway on GCS failure.".ljust(78) + "|")
    print("|" + "".center(78) + "|")
    print("+" + "=" * 78 + "+")
    print()


def print_preflight_status(drone_name: str, status: dict) -> bool:
    """
    Print comprehensive pre-flight status report for a drone.
    
    Args:
        drone_name: Name of the drone (Scout/Delivery)
        status: Status dict from drone.get_preflight_status()
        
    Returns:
        True if all checks pass
    """
    print()
    print("+" + "-" * 58 + "+")
    print("|" + f" PRE-FLIGHT STATUS: {drone_name} ".center(58) + "|")
    print("+" + "-" * 58 + "+")
    
    all_ok = True
    
    # Battery
    voltage = status.get('battery_voltage', 0.0)
    percent = status.get('battery_percent', -1)
    battery_ok = status.get('battery_ok', False)
    
    if percent >= 0:
        battery_str = f"{voltage:.1f}V ({percent}%)"
    else:
        battery_str = f"{voltage:.1f}V"
    
    battery_status = "OK" if battery_ok else "LOW"
    print(f"|  BATTERY: {battery_str.ljust(20)} [{battery_status}]".ljust(58) + " |")
    if not battery_ok:
        all_ok = False
    
    # GPS
    fix_type = status.get('gps_fix_type', 0)
    satellites = status.get('gps_satellites', 0)
    gps_ok = status.get('gps_ok', False)
    
    fix_names = {0: "No GPS", 1: "No Fix", 2: "2D Fix", 3: "3D Fix", 
                 4: "DGPS", 5: "RTK Float", 6: "RTK Fixed"}
    fix_name = fix_names.get(fix_type, f"Unknown({fix_type})")
    
    gps_status = "3D FIX" if gps_ok else "NO FIX"
    print(f"|  GPS:     {satellites} sats, {fix_name.ljust(12)} [{gps_status}]".ljust(58) + " |")
    if not gps_ok:
        all_ok = False
    
    # EKF
    ekf_ok = status.get('ekf_ok', False)
    ekf_flags = status.get('ekf_flags', 0)
    
    ekf_status = "HEALTHY" if ekf_ok else "UNHEALTHY"
    print(f"|  EKF:     flags=0x{ekf_flags:04x}".ljust(33) + f"[{ekf_status}]".ljust(25) + " |")
    if not ekf_ok:
        # EKF may not report on all controllers, so warn but don't fail
        logger.warning(f"EKF status not confirmed for {drone_name}")
    
    # Mode
    mode = status.get('mode', 'UNKNOWN')
    armed = status.get('armed', False)
    armed_str = "ARMED" if armed else "DISARMED"
    
    print(f"|  MODE:    {mode.ljust(20)} [{armed_str}]".ljust(58) + " |")
    
    # Overall status
    print("+" + "-" * 58 + "+")
    
    if all_ok:
        print("|" + " STATUS: READY FOR FLIGHT ".center(58, '*') + "|")
    else:
        print("|" + " STATUS: NOT READY - FIX ISSUES ABOVE ".center(58, '!') + "|")
    
    print("+" + "-" * 58 + "+")
    print()
    
    return all_ok


def confirm_arm_command() -> bool:
    """
    Require user to type 'ARM' to confirm mission start.
    
    SAFETY CRITICAL: This is the final gate before arming.
    
    Returns:
        True if user confirms, False otherwise
    """
    print()
    print("+" + "=" * 58 + "+")
    print("|" + " FINAL CONFIRMATION REQUIRED ".center(58) + "|")
    print("+" + "=" * 58 + "+")
    print("|" + "".center(58) + "|")
    print("|" + " You are about to arm and fly real drones.".center(58) + "|")
    print("|" + " Ensure all safety checks are GREEN above.".center(58) + "|")
    print("|" + " Stand clear of propellers.".center(58) + "|")
    print("|" + "".center(58) + "|")
    print("+" + "=" * 58 + "+")
    print()
    
    try:
        response = input(">>> Type 'ARM' to confirm mission start (or 'q' to quit): ").strip().upper()
        
        if response == 'ARM':
            print("\n*** MISSION CONFIRMED - ARMING SEQUENCE INITIATED ***\n")
            logger.info("User confirmed ARM command")
            return True
        elif response == 'Q':
            print("\nMission cancelled by user.")
            return False
        else:
            print(f"\nInvalid response: '{response}'. Mission cancelled.")
            print("You must type exactly 'ARM' to proceed.")
            return False
    
    except KeyboardInterrupt:
        print("\n\nMission cancelled by user.")
        return False


def run_full_mission(config: MissionConfig) -> bool:
    """
    Execute full dual-drone mission.
    
    Args:
        config: Mission configuration
        
    Returns:
        True if mission completed successfully
    """
    # =========================================================================
    # PHASE 1: SCOUT MISSION
    # =========================================================================
    print()
    print("╔" + "═" * 68 + "╗")
    print("║" + "  PHASE 1: SCOUT MISSION".center(68) + "║")
    print("╚" + "═" * 68 + "╝")
    print()
    
    scout = ScoutMission(config)
    
    if not scout.setup():
        logger.error("Scout mission setup failed")
        return False
    
    targets = scout.run()
    
    if not targets:
        logger.warning("No targets detected during scout mission")
        print("\nNo humans detected. Delivery mission not required.")
        return True
    
    print(f"\nScout mission complete. {len(targets)} targets detected.")
    
    # Small delay between missions
    print("\nPreparing for delivery mission...")
    time.sleep(3)
    
    # =========================================================================
    # PHASE 2: DELIVERY MISSION
    # =========================================================================
    print()
    print("╔" + "═" * 68 + "╗")
    print("║" + "  PHASE 2: DELIVERY MISSION".center(68) + "║")
    print("╚" + "═" * 68 + "╝")
    print()
    
    delivery = DeliveryMission(config)
    
    if not delivery.setup():
        logger.error("Delivery mission setup failed")
        return False
    
    if not delivery.load_targets():
        logger.error("Failed to load targets for delivery")
        return False
    
    deliveries = delivery.run()
    
    print(f"\nDelivery mission complete. {deliveries}/{len(targets)} deliveries made.")
    
    return True


def run_scout_only(config: MissionConfig) -> bool:
    """
    Execute only the Scout mission.
    
    Args:
        config: Mission configuration
        
    Returns:
        True if mission completed successfully
    """
    print()
    print("╔" + "═" * 68 + "╗")
    print("║" + "  SCOUT MISSION (ONLY)".center(68) + "║")
    print("╚" + "═" * 68 + "╝")
    print()
    
    scout = ScoutMission(config)
    
    if not scout.setup():
        logger.error("Scout mission setup failed")
        return False
    
    targets = scout.run()
    
    print(f"\nMission complete. {len(targets)} targets detected and saved.")
    print(f"Targets file: {config.get_targets_path()}")
    
    return True


def run_delivery_only(config: MissionConfig, targets_file: str = None) -> bool:
    """
    Execute only the Delivery mission.
    
    Args:
        config: Mission configuration
        targets_file: Path to targets.json (uses default if None)
        
    Returns:
        True if mission completed successfully
    """
    print()
    print("╔" + "═" * 68 + "╗")
    print("║" + "  DELIVERY MISSION (ONLY)".center(68) + "║")
    print("╚" + "═" * 68 + "╝")
    print()
    
    delivery = DeliveryMission(config)
    
    if not delivery.setup():
        logger.error("Delivery mission setup failed")
        return False
    
    if not delivery.load_targets(targets_file):
        logger.error("Failed to load targets")
        return False
    
    deliveries = delivery.run()
    
    print(f"\nMission complete. {deliveries} deliveries made.")
    
    return True


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description='Multi-Drone Mission Orchestrator',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    python main.py                              # Full mission
    python main.py --scout-only                 # Scout only
    python main.py --delivery-only              # Delivery only (requires targets.json)
    python main.py --kml custom_area.kml        # Custom survey area
    python main.py --simulate                   # Simulation mode (no real drones)
    python main.py --scout-port /dev/ttyUSB0    # Custom Scout connection
    python main.py --delivery-port /dev/ttyUSB1 # Custom Delivery connection
"""
    )
    
    # Mission mode
    mode_group = parser.add_mutually_exclusive_group()
    mode_group.add_argument('--scout-only', action='store_true',
                           help='Run only the Scout (survey) mission')
    mode_group.add_argument('--delivery-only', action='store_true',
                           help='Run only the Delivery (payload) mission')
    
    # Connections (default=None means use config.py values)
    parser.add_argument('--scout-port',
                       help='Scout drone connection (uses config.py if not specified)')
    parser.add_argument('--delivery-port',
                       help='Delivery drone connection (uses config.py if not specified)')
    parser.add_argument('--baud', type=int,
                       help='Baud rate for serial connections (default: 57600)')
    
    # Mission parameters
    parser.add_argument('--kml', help='Path to KML survey area file')
    parser.add_argument('--targets', help='Path to targets.json (for --delivery-only)')
    parser.add_argument('--rtsp', help='RTSP stream URL for video')
    parser.add_argument('--altitude', type=float,
                       help='Survey altitude in meters (default: 10)')
    parser.add_argument('--capacity', type=int,
                       help='Payload capacity per flight (default: 5)')
    
    # Simulation
    parser.add_argument('--simulate', action='store_true',
                       help='Run in simulation mode (no real drones)')
    
    args = parser.parse_args()
    
    # Build configuration from config.py defaults
    config = MissionConfig()
    
    # Only override config if argument was explicitly provided
    if args.scout_port:
        config.SCOUT_CONNECTION = args.scout_port
    if args.delivery_port:
        config.DELIVERY_CONNECTION = args.delivery_port
    if args.baud:
        config.SCOUT_BAUD = args.baud
        config.DELIVERY_BAUD = args.baud
    if args.altitude:
        config.SCOUT_ALTITUDE = args.altitude
        config.DELIVERY_ALTITUDE = args.altitude
    if args.capacity:
        config.PAYLOAD_CAPACITY = args.capacity
    if args.kml:
        config.KML_FILE = args.kml
    if args.rtsp:
        config.RTSP_URL = args.rtsp
    
    # Print banner and config
    print_banner()
    print_config(config)
    
    # Simulation mode notice
    if args.simulate:
        print("*** SIMULATION MODE - No real drones will be connected ***\n")
        # In simulation mode, we'd use mock connections
        # For now, just note that real connections will fail
    
    # =========================================================================
    # SAFETY CRITICAL: Pre-Flight Checks and Confirmation
    # =========================================================================
    if not args.simulate:
        # Import DroneController for pre-flight status
        from drone_controller import DroneController
        
        # Print failsafe warning
        print_failsafe_warning()
        
        # Determine which drones to check
        drones_to_check = []
        
        if args.scout_only:
            drones_to_check = [
                ("Scout", config.SCOUT_CONNECTION, config.SCOUT_BAUD)
            ]
        elif args.delivery_only:
            drones_to_check = [
                ("Delivery", config.DELIVERY_CONNECTION, config.DELIVERY_BAUD)
            ]
        else:
            drones_to_check = [
                ("Scout", config.SCOUT_CONNECTION, config.SCOUT_BAUD),
                ("Delivery", config.DELIVERY_CONNECTION, config.DELIVERY_BAUD)
            ]
        
        # Pre-flight status check for each drone
        all_drones_ready = True
        
        for drone_name, connection, baud in drones_to_check:
            print(f"\nConnecting to {drone_name} ({connection}) for pre-flight check...")
            
            try:
                drone = DroneController(connection, baud=baud, name=drone_name)
                
                if drone.connect(timeout=config.CONNECTION_TIMEOUT):
                    # Get and display status
                    status = drone.get_preflight_status()
                    is_ready = print_preflight_status(drone_name, status)
                    
                    if not is_ready:
                        all_drones_ready = False
                    
                    drone.disconnect()
                else:
                    print(f"\n  ERROR: Could not connect to {drone_name}!")
                    print(f"  Check connection: {connection}")
                    all_drones_ready = False
                    
            except Exception as e:
                print(f"\n  ERROR: Pre-flight check failed for {drone_name}: {e}")
                all_drones_ready = False
        
        # Final confirmation
        if not all_drones_ready:
            print("\n" + "!" * 60)
            print("  WARNING: Not all pre-flight checks passed!")
            print("  Proceeding is NOT recommended.")
            print("!" * 60)
        
        if not confirm_arm_command():
            print("\nMission aborted. No drones were armed.")
            sys.exit(0)
    
    # Execute mission
    success = False
    
    try:
        if args.scout_only:
            success = run_scout_only(config)
        elif args.delivery_only:
            success = run_delivery_only(config, args.targets)
        else:
            success = run_full_mission(config)
    
    except KeyboardInterrupt:
        print("\n\nMission interrupted by user.")
        success = False
    
    except Exception as e:
        logger.error(f"Mission failed with error: {e}")
        success = False
    
    # Final summary
    print()
    print("=" * 70)
    if success:
        print("  MISSION COMPLETED SUCCESSFULLY")
    else:
        print("  MISSION ENDED WITH ERRORS")
    print(f"  End Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 70)
    print()
    
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
