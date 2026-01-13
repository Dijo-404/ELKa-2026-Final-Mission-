#!/usr/bin/env python3
"""
DroneController - MAVLink Wrapper for Drone Control

Provides a simplified interface for common drone operations:
- Connection management
- Arming/Disarming
- Takeoff/Landing
- Waypoint navigation (GUIDED mode)
- Servo control (payload drops)
- GPS position retrieval
"""

import time
import math
import logging
from typing import Optional, Tuple
from pymavlink import mavutil

logger = logging.getLogger(__name__)


def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """
    Calculate the great-circle distance between two GPS points.
    
    Args:
        lat1, lon1: First point coordinates (degrees)
        lat2, lon2: Second point coordinates (degrees)
        
    Returns:
        Distance in meters
    """
    R = 6371000  # Earth's radius in meters
    
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    
    a = (math.sin(delta_phi / 2) ** 2 + 
         math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    return R * c


class DroneController:
    """
    MAVLink wrapper for drone control operations.
    
    Provides high-level methods for common drone operations with
    error handling and safety checks.
    """
    
    def __init__(self, connection_string: str, baud: int = 57600, name: str = "Drone"):
        """
        Initialize DroneController.
        
        Args:
            connection_string: MAVLink connection string (e.g., '/dev/ttyUSB0')
            baud: Baud rate for serial connections
            name: Human-readable name for logging
        """
        self.connection_string = connection_string
        self.baud = baud
        self.name = name
        self.mav: Optional[mavutil.mavlink_connection] = None
        self._is_armed = False
        self.home_position: Optional[Tuple[float, float, float]] = None
        
        # Safety settings
        self.min_battery_voltage = 14.0
        self.min_battery_percent = 20
        
    def connect(self, timeout: float = 10.0) -> bool:
        """
        Establish MAVLink connection to the flight controller.
        
        Args:
            timeout: Connection timeout in seconds
            
        Returns:
            True if connection successful
        """
        try:
            logger.info(f"[{self.name}] Connecting to {self.connection_string}...")
            
            # Create connection based on connection string type
            if self.connection_string.startswith('/dev/') or self.connection_string.startswith('COM'):
                # Serial connection
                self.mav = mavutil.mavlink_connection(
                    self.connection_string, 
                    baud=self.baud
                )
            elif self.connection_string.startswith('udp') or self.connection_string.startswith('tcp'):
                # Network connection
                self.mav = mavutil.mavlink_connection(self.connection_string)
            else:
                # Assume serial
                self.mav = mavutil.mavlink_connection(
                    self.connection_string, 
                    baud=self.baud
                )
            
            # Wait for heartbeat
            logger.info(f"[{self.name}] Waiting for heartbeat...")
            msg = self.mav.wait_heartbeat(timeout=timeout)
            
            if msg:
                logger.info(f"[{self.name}] Connected to system {self.mav.target_system}, "
                           f"component {self.mav.target_component}")
                return True
            else:
                logger.error(f"[{self.name}] No heartbeat received (timeout: {timeout}s)")
                return False
                
        except Exception as e:
            logger.error(f"[{self.name}] Connection failed: {e}")
            return False
    
    def disconnect(self):
        """Close MAVLink connection."""
        if self.mav:
            self.mav.close()
            self.mav = None
            logger.info(f"[{self.name}] Disconnected")
    
    def set_mode(self, mode: str) -> bool:
        """
        Set flight mode.
        
        Args:
            mode: Flight mode string ('GUIDED', 'LOITER', 'RTL', 'LAND', etc.)
            
        Returns:
            True if mode set successfully
        """
        if not self.mav:
            logger.error(f"[{self.name}] Not connected")
            return False
        
        mode_mapping = self.mav.mode_mapping()
        if mode not in mode_mapping:
            logger.error(f"[{self.name}] Unknown mode: {mode}")
            logger.info(f"[{self.name}] Available modes: {list(mode_mapping.keys())}")
            return False
        
        mode_id = mode_mapping[mode]
        self.mav.set_mode(mode_id)
        
        # Verify mode change
        time.sleep(0.5)
        msg = self.mav.recv_match(type='HEARTBEAT', blocking=True, timeout=2.0)
        if msg and msg.custom_mode == mode_id:
            logger.info(f"[{self.name}] Mode set to {mode}")
            return True
        
        logger.warning(f"[{self.name}] Mode change to {mode} may not have succeeded")
        return True  # Optimistic return
    
    def check_battery(self) -> Tuple[bool, float, int]:
        """
        Check battery status.
        
        Returns:
            Tuple of (is_safe, voltage, percentage)
        """
        if not self.mav:
            return False, 0.0, 0
        
        msg = self.mav.recv_match(type='BATTERY_STATUS', blocking=True, timeout=1.0)
        if not msg:
            msg = self.mav.recv_match(type='SYS_STATUS', blocking=True, timeout=1.0)
        
        voltage = 0.0
        percentage = -1
        
        if msg:
            if hasattr(msg, 'voltages') and msg.voltages[0] != 65535:
                voltage = msg.voltages[0] / 1000.0
            elif hasattr(msg, 'voltage_battery'):
                voltage = msg.voltage_battery / 1000.0
            
            if hasattr(msg, 'battery_remaining'):
                percentage = msg.battery_remaining
        
        is_safe = (voltage >= self.min_battery_voltage and 
                   (percentage < 0 or percentage >= self.min_battery_percent))
        
        if not is_safe:
            logger.warning(f"[{self.name}] Battery low! Voltage: {voltage}V, Remaining: {percentage}%")
        
        return is_safe, voltage, percentage
    
    def wait_for_gps(self, timeout: float = 30.0) -> bool:
        """
        Wait for valid GPS fix.
        
        Args:
            timeout: Maximum wait time in seconds
            
        Returns:
            True if GPS is available
        """
        logger.info(f"[{self.name}] Waiting for GPS fix...")
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            msg = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1.0)
            if msg and msg.lat != 0 and msg.lon != 0:
                logger.info(f"[{self.name}] GPS fix acquired")
                return True
            time.sleep(0.5)
        
        logger.error(f"[{self.name}] GPS timeout")
        return False
    
    def arm(self) -> bool:
        """
        Arm the drone with safety checks.
        
        Returns:
            True if armed successfully
        """
        if not self.mav:
            logger.error(f"[{self.name}] Not connected")
            return False
        
        # Pre-arm checks
        battery_ok, voltage, percent = self.check_battery()
        if not battery_ok:
            logger.error(f"[{self.name}] Cannot arm: Battery too low ({voltage}V, {percent}%)")
            return False
        
        # Wait for GPS
        if not self.wait_for_gps():
            logger.error(f"[{self.name}] Cannot arm: No GPS fix")
            return False
        
        # Store home position
        self.home_position = self.get_location()
        
        # Send arm command
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            1,  # arm
            0, 0, 0, 0, 0, 0
        )
        
        # Wait for acknowledgment
        msg = self.mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=3.0)
        if msg and msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            self._is_armed = True
            logger.info(f"[{self.name}] Armed successfully")
            return True
        else:
            logger.error(f"[{self.name}] Arm command failed")
            return False
    
    def disarm(self) -> bool:
        """
        Disarm the drone.
        
        Returns:
            True if disarmed successfully
        """
        if not self.mav:
            return False
        
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            0,  # disarm
            0, 0, 0, 0, 0, 0
        )
        
        msg = self.mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=3.0)
        if msg and msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            self._is_armed = False
            logger.info(f"[{self.name}] Disarmed successfully")
            return True
        return False
    
    def takeoff(self, altitude: float) -> bool:
        """
        Execute takeoff to specified altitude.
        
        Args:
            altitude: Target altitude in meters
            
        Returns:
            True if takeoff initiated successfully
        """
        if not self.mav:
            logger.error(f"[{self.name}] Not connected")
            return False
        
        if not self._is_armed:
            logger.error(f"[{self.name}] Cannot takeoff: Not armed")
            return False
        
        # Ensure GUIDED mode
        if not self.set_mode('GUIDED'):
            logger.error(f"[{self.name}] Cannot takeoff: Failed to set GUIDED mode")
            return False
        
        # Send takeoff command
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,  # confirmation
            0,  # pitch
            0,  # empty
            0,  # empty
            0,  # yaw angle
            0,  # latitude
            0,  # longitude
            altitude
        )
        
        msg = self.mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=3.0)
        if msg and msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            logger.info(f"[{self.name}] Takeoff initiated to {altitude}m")
            return True
        else:
            logger.error(f"[{self.name}] Takeoff command failed")
            return False
    
    def wait_for_altitude(self, target_alt: float, tolerance: float = 1.0, 
                          timeout: float = 30.0) -> bool:
        """
        Wait until drone reaches target altitude.
        
        Args:
            target_alt: Target altitude in meters
            tolerance: Acceptable deviation in meters
            timeout: Maximum wait time in seconds
            
        Returns:
            True if altitude reached
        """
        logger.info(f"[{self.name}] Waiting for altitude {target_alt}m...")
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            loc = self.get_location()
            if loc:
                _, _, current_alt = loc
                if abs(current_alt - target_alt) <= tolerance:
                    logger.info(f"[{self.name}] Reached altitude: {current_alt:.1f}m")
                    return True
            time.sleep(0.5)
        
        logger.warning(f"[{self.name}] Timeout waiting for altitude {target_alt}m")
        return False
    
    def get_location(self) -> Optional[Tuple[float, float, float]]:
        """
        Get current GPS coordinates.
        
        Returns:
            Tuple of (latitude, longitude, altitude) or None
        """
        if not self.mav:
            return None
        
        msg = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1.0)
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.relative_alt / 1000.0  # Convert mm to m
            return lat, lon, alt
        
        return None
    
    def goto(self, lat: float, lon: float, alt: float) -> bool:
        """
        Navigate to specified GPS coordinates.
        
        Args:
            lat: Target latitude
            lon: Target longitude
            alt: Target altitude in meters
            
        Returns:
            True if navigation command sent
        """
        if not self.mav:
            logger.error(f"[{self.name}] Not connected")
            return False
        
        # Check battery before navigation
        battery_ok, _, _ = self.check_battery()
        if not battery_ok:
            logger.warning(f"[{self.name}] Battery low - triggering RTL")
            self.rtl()
            return False
        
        # Send position target
        self.mav.mav.set_position_target_global_int_send(
            0,  # timestamp
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,  # type_mask (only positions)
            int(lat * 1e7),
            int(lon * 1e7),
            alt,
            0, 0, 0,  # velocities
            0, 0, 0,  # accelerations
            0, 0  # yaw, yaw_rate
        )
        
        logger.info(f"[{self.name}] Navigating to ({lat:.6f}, {lon:.6f}, {alt:.1f}m)")
        return True
    
    def set_speed(self, speed: float) -> bool:
        """
        Set flight speed.
        
        Args:
            speed: Speed in m/s
            
        Returns:
            True if speed set successfully
        """
        if not self.mav:
            return False
        
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
            0,  # confirmation
            0,  # speed type (0 = airspeed, 1 = ground speed)
            speed,  # speed
            -1,  # throttle (-1 = no change)
            0, 0, 0, 0
        )
        
        logger.info(f"[{self.name}] Speed set to {speed} m/s")
        return True
    
    def wait_for_arrival(self, lat: float, lon: float, tolerance: float = 2.0,
                         timeout: float = 120.0) -> bool:
        """
        Wait until drone arrives at target coordinates.
        
        Args:
            lat: Target latitude
            lon: Target longitude
            tolerance: Acceptable distance in meters
            timeout: Maximum wait time in seconds
            
        Returns:
            True if arrived within timeout
        """
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            loc = self.get_location()
            if loc:
                current_lat, current_lon, _ = loc
                distance = haversine_distance(current_lat, current_lon, lat, lon)
                
                if distance <= tolerance:
                    logger.info(f"[{self.name}] Arrived at waypoint (distance: {distance:.1f}m)")
                    return True
            
            # Check battery during transit
            battery_ok, _, _ = self.check_battery()
            if not battery_ok:
                logger.warning(f"[{self.name}] Battery low during transit - triggering RTL")
                self.rtl()
                return False
            
            time.sleep(0.5)
        
        logger.warning(f"[{self.name}] Timeout waiting for arrival at ({lat}, {lon})")
        return False
    
    def set_servo(self, channel: int, pwm: int) -> bool:
        """
        Set servo PWM value.
        
        Args:
            channel: Servo channel number (1-16)
            pwm: PWM value (typically 1000-2000)
            
        Returns:
            True if command sent successfully
        """
        if not self.mav:
            return False
        
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,  # confirmation
            channel,  # servo number
            pwm,  # PWM value
            0, 0, 0, 0, 0
        )
        
        logger.info(f"[{self.name}] Servo {channel} set to PWM {pwm}")
        return True
    
    def rtl(self) -> bool:
        """
        Return to Launch.
        
        Returns:
            True if RTL mode set successfully
        """
        logger.warning(f"[{self.name}] RTL TRIGGERED - Returning to launch")
        return self.set_mode('RTL')
    
    def land(self) -> bool:
        """
        Initiate landing at current position.
        
        Returns:
            True if land mode set successfully
        """
        logger.info(f"[{self.name}] Initiating landing")
        return self.set_mode('LAND')
    
    def wait_for_land(self, timeout: float = 60.0) -> bool:
        """
        Wait until drone has landed.
        
        Args:
            timeout: Maximum wait time in seconds
            
        Returns:
            True if landed (altitude near 0 and disarmed)
        """
        logger.info(f"[{self.name}] Waiting for landing...")
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            loc = self.get_location()
            if loc:
                _, _, alt = loc
                if alt < 0.5:  # Less than 0.5m altitude
                    # Check if disarmed
                    msg = self.mav.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
                    if msg and not (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
                        self._is_armed = False
                        logger.info(f"[{self.name}] Landed and disarmed")
                        return True
            time.sleep(0.5)
        
        logger.warning(f"[{self.name}] Landing timeout")
        return False
    
    def is_armed(self) -> bool:
        """Check if drone is armed."""
        return self._is_armed
    
    def is_connected(self) -> bool:
        """Check if MAVLink connection is active."""
        if not self.mav:
            return False
        msg = self.mav.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
        return msg is not None


# =============================================================================
# TEST CODE
# =============================================================================
if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
        datefmt='%H:%M:%S'
    )
    
    # Test with SITL or real drone
    print("DroneController Test")
    print("=" * 50)
    
    # Example connection strings:
    # - SITL: 'udpin:127.0.0.1:14550'
    # - USB Telemetry: '/dev/ttyUSB0'
    
    connection = input("Enter connection string (default: udpin:127.0.0.1:14550): ").strip()
    if not connection:
        connection = 'udpin:127.0.0.1:14550'
    
    drone = DroneController(connection, name="TestDrone")
    
    if drone.connect():
        print(f"\nConnected successfully!")
        
        loc = drone.get_location()
        if loc:
            print(f"Current position: {loc}")
        
        battery_ok, voltage, percent = drone.check_battery()
        print(f"Battery: {voltage}V, {percent}%")
        
        drone.disconnect()
    else:
        print("Connection failed!")
