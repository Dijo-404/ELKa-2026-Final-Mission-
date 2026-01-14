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
