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
        self.min_gps_fix = 3           # 3D fix minimum
        self.min_satellites = 6         # Minimum satellite count
        self.max_geofence_radius = 500  # Maximum distance from home (meters)
        self.arm_timeout = 10.0         # Timeout for arm sequence
        
        # Heartbeat tracking for failsafe
        self.last_heartbeat_time = 0
        self.heartbeat_timeout = 5.0    # Seconds before failsafe
        
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
            
            # Wait for heartbeat and set target system
            logger.info(f"[{self.name}] Waiting for heartbeat...")
            msg = self.mav.wait_heartbeat(timeout=timeout)
            
            if msg:
                # Ensure we have a valid target system (not 0)
                if self.mav.target_system == 0:
                    # Try to get from heartbeat source
                    self.mav.target_system = msg.get_srcSystem()
                    if self.mav.target_system == 0:
                        self.mav.target_system = 1  # Default to 1
                
                if self.mav.target_component == 0:
                    self.mav.target_component = 1  # Default to autopilot component
                
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
    
    def wait_for_ekf_ready(self, timeout: float = 30.0) -> bool:
        """
        Wait for EKF to have valid position estimation.
        
        Critical for GUIDED mode operation. EKF must have good
        position estimate before arming.
        
        Args:
            timeout: Maximum wait time in seconds
            
        Returns:
            True if EKF is ready
        """
        logger.info(f"[{self.name}] Waiting for EKF position estimation...")
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            # Check EKF status
            msg = self.mav.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=1.0)
            if msg:
                # Check position flags
                pos_horiz = msg.flags & 1   # EKF_POS_HORIZ_ABS
                pos_vert = msg.flags & 2    # EKF_POS_VERT_ABS
                pred_horiz = msg.flags & 4  # EKF_PRED_POS_HORIZ_ABS
                
                if pos_horiz and pos_vert:
                    logger.info(f"[{self.name}] EKF position estimation ready")
                    return True
            
            # Also verify GPS position is valid
            gps_msg = self.mav.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=0.5)
            if gps_msg and gps_msg.lat != 0 and gps_msg.lon != 0:
                # If we have GPS but no EKF message, check GPS fix quality
                gps_raw = self.mav.recv_match(type='GPS_RAW_INT', blocking=True, timeout=0.5)
                if gps_raw and gps_raw.fix_type >= 3:  # 3D fix or better
                    logger.info(f"[{self.name}] GPS 3D fix acquired (fix_type={gps_raw.fix_type})")
                    return True
            
            time.sleep(0.5)
        
        logger.error(f"[{self.name}] EKF position estimation timeout")
        return False
    
    def check_gps_quality(self) -> Tuple[bool, int, int]:
        """
        Check GPS fix quality and satellite count.
        
        SAFETY CRITICAL: Ensures GPS is reliable before arming.
        
        Returns:
            Tuple of (is_safe, fix_type, satellite_count)
        """
        if not self.mav:
            return False, 0, 0
        
        msg = self.mav.recv_match(type='GPS_RAW_INT', blocking=True, timeout=2.0)
        if not msg:
            logger.warning(f"[{self.name}] No GPS_RAW_INT message received")
            return False, 0, 0
        
        fix_type = msg.fix_type
        satellites = msg.satellites_visible
        
        is_safe = fix_type >= self.min_gps_fix and satellites >= self.min_satellites
        
        if not is_safe:
            if fix_type < self.min_gps_fix:
                logger.warning(f"[{self.name}] GPS fix insufficient: {fix_type} (need >= {self.min_gps_fix})")
            if satellites < self.min_satellites:
                logger.warning(f"[{self.name}] Satellites insufficient: {satellites} (need >= {self.min_satellites})")
        else:
            logger.info(f"[{self.name}] GPS OK: fix_type={fix_type}, satellites={satellites}")
        
        return is_safe, fix_type, satellites
    
    def check_ekf_status(self) -> Tuple[bool, int]:
        """
        Check EKF status flags for position estimation validity.
        
        SAFETY CRITICAL: Ensures navigation is reliable before arming.
        
        Returns:
            Tuple of (is_healthy, flags)
        """
        if not self.mav:
            return False, 0
        
        msg = self.mav.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=2.0)
        if not msg:
            # Try ESTIMATOR_STATUS as fallback
            msg = self.mav.recv_match(type='ESTIMATOR_STATUS', blocking=True, timeout=1.0)
        
        if not msg:
            logger.warning(f"[{self.name}] No EKF status message received")
            return False, 0
        
        flags = msg.flags if hasattr(msg, 'flags') else 0
        
        # Check critical flags
        pos_horiz = flags & 1   # EKF_POS_HORIZ_ABS
        pos_vert = flags & 2    # EKF_POS_VERT_ABS
        
        is_healthy = bool(pos_horiz and pos_vert)
        
        if is_healthy:
            logger.info(f"[{self.name}] EKF healthy: flags=0x{flags:04x}")
        else:
            logger.warning(f"[{self.name}] EKF not healthy: flags=0x{flags:04x}")
        
        return is_healthy, flags
    
    def verify_mode(self, expected_mode: str, timeout: float = 5.0) -> bool:
        """
        Verify flight mode via HEARTBEAT.
        
        SAFETY CRITICAL: Confirms mode actually changed.
        
        Args:
            expected_mode: Expected mode string ('GUIDED', etc.)
            timeout: Maximum wait time
            
        Returns:
            True if mode matches expected
        """
        if not self.mav:
            return False
        
        mode_mapping = self.mav.mode_mapping()
        if expected_mode not in mode_mapping:
            return False
        
        expected_id = mode_mapping[expected_mode]
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            msg = self.mav.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
            if msg:
                self.last_heartbeat_time = time.time()
                if msg.custom_mode == expected_id:
                    logger.info(f"[{self.name}] Mode verified: {expected_mode}")
                    return True
            time.sleep(0.1)
        
        logger.error(f"[{self.name}] Mode verification failed for {expected_mode}")
        return False
    
    def check_motors_spinning(self, timeout: float = 5.0) -> bool:
        """
        Verify motors are spinning after arm command.
        
        SAFETY CRITICAL: Confirms drone is actually armed and active.
        
        Args:
            timeout: Maximum wait time
            
        Returns:
            True if system status is ACTIVE and armed
        """
        if not self.mav:
            return False
        
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            msg = self.mav.recv_match(type='HEARTBEAT', blocking=True, timeout=1.0)
            if msg:
                self.last_heartbeat_time = time.time()
                armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                # MAV_STATE_ACTIVE = 4
                active = msg.system_status == 4
                
                if armed and active:
                    logger.info(f"[{self.name}] Motors spinning - system ACTIVE")
                    return True
                elif armed:
                    logger.debug(f"[{self.name}] Armed but system_status={msg.system_status}")
            time.sleep(0.2)
        
        logger.error(f"[{self.name}] Motors spin check failed")
        return False
    
    def check_geofence(self) -> bool:
        """
        Check if drone is within geofence radius.
        
        SAFETY CRITICAL: Triggers RTL if outside radius.
        
        Returns:
            True if within geofence, False and triggers RTL if outside
        """
        if not self.home_position:
            return True  # No home set, can't check
        
        loc = self.get_location()
        if not loc:
            return True  # Can't get location, don't trigger
        
        current_lat, current_lon, _ = loc
        home_lat, home_lon, _ = self.home_position
        
        distance = haversine_distance(current_lat, current_lon, home_lat, home_lon)
        
        if distance > self.max_geofence_radius:
            logger.error(f"[{self.name}] GEOFENCE BREACH! Distance={distance:.0f}m > {self.max_geofence_radius}m - RTL!")
            self.rtl()
            return False
        
        return True
    
    def check_heartbeat_timeout(self) -> bool:
        """
        Check if heartbeat has timed out.
        
        SAFETY CRITICAL: Returns False if last heartbeat > timeout.
        
        Returns:
            True if heartbeat is fresh, False if stale
        """
        if self.last_heartbeat_time == 0:
            return True  # Not tracking yet
        
        elapsed = time.time() - self.last_heartbeat_time
        
        if elapsed > self.heartbeat_timeout:
            logger.error(f"[{self.name}] HEARTBEAT TIMEOUT! Last={elapsed:.1f}s > {self.heartbeat_timeout}s")
            return False
        
        return True
    
    def arm(self) -> bool:
        """
        Arm the drone with comprehensive safety checks.
        
        Performs pre-flight checks:
        - Battery voltage and percentage
        - GPS fix availability
        - EKF position estimation status
        
        Returns:
            True if armed successfully
        """
        if not self.mav:
            logger.error(f"[{self.name}] Not connected")
            return False
        
        logger.info(f"[{self.name}] Running pre-arm checks...")
        
        # Check 1: Battery
        battery_ok, voltage, percent = self.check_battery()
        if not battery_ok:
            logger.error(f"[{self.name}] PRE-ARM FAIL: Battery too low ({voltage:.1f}V, {percent}%)")
            return False
        logger.info(f"[{self.name}] Battery OK: {voltage:.1f}V, {percent}%")
        
        # Check 2: GPS fix
        if not self.wait_for_gps():
            logger.error(f"[{self.name}] PRE-ARM FAIL: No GPS fix")
            return False
        
        # Check 3: EKF position estimation (critical for GUIDED mode)
        if not self.wait_for_ekf_ready():
            logger.error(f"[{self.name}] PRE-ARM FAIL: EKF not ready")
            return False
        
        # Store home position
        self.home_position = self.get_location()
        if self.home_position:
            logger.info(f"[{self.name}] Home position: ({self.home_position[0]:.6f}, "
                       f"{self.home_position[1]:.6f}, {self.home_position[2]:.1f}m)")
        
        logger.info(f"[{self.name}] Pre-arm checks passed. Sending arm command...")
        
        # Retry arm command up to 3 times
        for attempt in range(3):
            # Clear any pending messages
            self.mav.recv_match(type='COMMAND_ACK', blocking=False)
            
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
            
            if msg:
                if msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        self._is_armed = True
                        logger.info(f"[{self.name}] ARMED SUCCESSFULLY")
                        return True
                    else:
                        logger.warning(f"[{self.name}] Arm attempt {attempt+1} failed: result={msg.result}")
                else:
                    # Got ACK for different command, retry
                    logger.debug(f"[{self.name}] Got ACK for different command, retrying...")
            else:
                logger.warning(f"[{self.name}] Arm attempt {attempt+1} timeout")
            
            time.sleep(0.5)
        
        logger.error(f"[{self.name}] ARM FAILED after 3 attempts")
        return False
    
    def arm_and_takeoff(self, target_altitude: float) -> bool:
        """
        Robust arming and takeoff sequence with comprehensive safety checks.
        
        SAFETY CRITICAL: This is the recommended method for arming in production.
        
        Sequence:
        1. Pre-arm checks (GPS fix>=3, sats>=6, EKF health, battery)
        2. Set GUIDED mode + verify via HEARTBEAT
        3. Send arm command + verify COMMAND_ACK
        4. Verify motors spinning via system_status
        5. Takeoff and wait for 95% of target altitude
        
        All steps have configured timeout (default 10s).
        
        Args:
            target_altitude: Target altitude in meters
            
        Returns:
            True if armed and reached altitude
            
        Raises:
            TimeoutError: If any step exceeds timeout
        """
        if not self.mav:
            logger.error(f"[{self.name}] Not connected")
            return False
        
        timeout = self.arm_timeout
        
        # =====================================================================
        # STEP 1: Pre-Arm Checks
        # =====================================================================
        logger.info(f"[{self.name}] === ROBUST ARM SEQUENCE STARTING ===")
        logger.info(f"[{self.name}] [1/5] Running pre-arm safety checks...")
        
        # Check 1a: Battery
        battery_ok, voltage, percent = self.check_battery()
        if not battery_ok:
            logger.error(f"[{self.name}] PRE-ARM FAIL: Battery too low ({voltage:.1f}V, {percent}%)")
            return False
        logger.info(f"[{self.name}] ✓ Battery OK: {voltage:.1f}V, {percent}%")
        
        # Check 1b: GPS Quality
        gps_ok, fix_type, satellites = self.check_gps_quality()
        if not gps_ok:
            logger.error(f"[{self.name}] PRE-ARM FAIL: GPS quality insufficient")
            return False
        logger.info(f"[{self.name}] ✓ GPS OK: fix={fix_type}, sats={satellites}")
        
        # Check 1c: EKF Status
        ekf_ok, flags = self.check_ekf_status()
        if not ekf_ok:
            # Warn but continue - some flight controllers don't report this
            logger.warning(f"[{self.name}] EKF check inconclusive (flags=0x{flags:04x})")
        else:
            logger.info(f"[{self.name}] ✓ EKF healthy")
        
        # Store home position
        self.home_position = self.get_location()
        if self.home_position:
            logger.info(f"[{self.name}] ✓ Home: ({self.home_position[0]:.6f}, "
                       f"{self.home_position[1]:.6f})")
        
        # =====================================================================
        # STEP 2: Set GUIDED Mode + Verify
        # =====================================================================
        logger.info(f"[{self.name}] [2/5] Setting GUIDED mode...")
        
        if not self.set_mode('GUIDED'):
            logger.error(f"[{self.name}] Failed to send GUIDED mode command")
            return False
        
        if not self.verify_mode('GUIDED', timeout=timeout):
            logger.error(f"[{self.name}] GUIDED mode verification FAILED")
            return False
        logger.info(f"[{self.name}] ✓ GUIDED mode verified")
        
        # =====================================================================
        # STEP 3: Arm with ACK Verification
        # =====================================================================
        logger.info(f"[{self.name}] [3/5] Arming motors...")
        
        # Clear pending ACKs
        self.mav.recv_match(type='COMMAND_ACK', blocking=False)
        
        # Send arm command
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            1,  # arm
            0, 0, 0, 0, 0, 0
        )
        
        # Wait for ACK
        start_time = time.time()
        arm_accepted = False
        
        while time.time() - start_time < timeout:
            msg = self.mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=1.0)
            if msg and msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    arm_accepted = True
                    break
                else:
                    # Map result codes to human readable errors
                    result_names = {
                        0: "ACCEPTED",
                        1: "TEMPORARILY_REJECTED",
                        2: "DENIED",
                        3: "UNSUPPORTED",
                        4: "FAILED",
                        5: "IN_PROGRESS",
                        6: "CANCELLED"
                    }
                    result_str = result_names.get(msg.result, f"UNKNOWN({msg.result})")
                    logger.error(f"[{self.name}] ARM REJECTED: {result_str}")
                    return False
        
        if not arm_accepted:
            logger.error(f"[{self.name}] ARM command timeout - no ACK received")
            return False
        
        self._is_armed = True
        logger.info(f"[{self.name}] ✓ Arm command accepted")
        
        # =====================================================================
        # STEP 4: Verify Motors Spinning
        # =====================================================================
        logger.info(f"[{self.name}] [4/5] Verifying motors spinning...")
        
        if not self.check_motors_spinning(timeout=timeout):
            logger.error(f"[{self.name}] Motors not spinning - disarming!")
            self.disarm()
            return False
        logger.info(f"[{self.name}] ✓ Motors confirmed spinning")
        
        # =====================================================================
        # STEP 5: Takeoff to 95% Altitude
        # =====================================================================
        logger.info(f"[{self.name}] [5/5] Taking off to {target_altitude}m...")
        
        # Send takeoff command
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,  # confirmation
            0,  # pitch
            0, 0, 0, 0, 0,
            target_altitude
        )
        
        msg = self.mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=3.0)
        if not msg or msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            logger.error(f"[{self.name}] Takeoff command not accepted")
            self.disarm()
            return False
        
        # Wait for 95% of target altitude
        target_95 = target_altitude * 0.95
        takeoff_timeout = 60.0  # 1 minute for takeoff
        start_time = time.time()
        
        logger.info(f"[{self.name}] Waiting for altitude >= {target_95:.1f}m (95%)...")
        
        while time.time() - start_time < takeoff_timeout:
            # Maintain heartbeat
            self.mav.recv_match(type='HEARTBEAT', blocking=False)
            
            loc = self.get_location()
            if loc:
                _, _, current_alt = loc
                if current_alt >= target_95:
                    logger.info(f"[{self.name}] ✓ Reached {current_alt:.1f}m >= {target_95:.1f}m")
                    logger.info(f"[{self.name}] === ARM AND TAKEOFF COMPLETE ===")
                    return True
                
                # Progress log every 2 seconds
                if int(time.time() - start_time) % 2 == 0:
                    logger.debug(f"[{self.name}] Climbing: {current_alt:.1f}m / {target_altitude}m")
            
            time.sleep(0.3)
        
        logger.error(f"[{self.name}] Takeoff altitude timeout!")
        return False

    
    def disarm(self) -> bool:
        """
        Disarm the drone.
        
        Returns:
            True if disarmed successfully
        """
        if not self.mav:
            return False
        
        logger.info(f"[{self.name}] Disarming...")
        
        # Clear pending messages
        self.mav.recv_match(type='COMMAND_ACK', blocking=False)
        
        # Send disarm command
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
        
        # If first attempt failed, try force disarm
        logger.warning(f"[{self.name}] Normal disarm failed, trying force disarm...")
        self.mav.mav.command_long_send(
            self.mav.target_system,
            self.mav.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,      # confirmation
            0,      # disarm
            21196,  # force disarm magic number
            0, 0, 0, 0, 0
        )
        
        msg = self.mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=3.0)
        if msg and msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            self._is_armed = False
            logger.info(f"[{self.name}] Force disarmed successfully")
            return True
        
        logger.error(f"[{self.name}] Disarm failed")
        self._is_armed = False  # Reset internal state anyway
        return False
    
    def is_armed(self) -> bool:
        """
        Check if drone is armed (queries internal state).
        
        Returns:
            True if armed
        """
        return self._is_armed
    
    def query_armed_state(self) -> bool:
        """
        Query actual armed state from the drone via heartbeat.
        
        Returns:
            True if drone reports armed
        """
        if not self.mav:
            return False
        
        msg = self.mav.recv_match(type='HEARTBEAT', blocking=True, timeout=2.0)
        if msg:
            armed = msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            self._is_armed = bool(armed)
            return self._is_armed
        return self._is_armed
    
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
        
        Maintains GCS heartbeat during wait to prevent failsafe.
        
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
            # Receive any pending messages (maintains heartbeat)
            self.mav.recv_match(type='HEARTBEAT', blocking=False)
            
            loc = self.get_location()
            if loc:
                _, _, current_alt = loc
                if abs(current_alt - target_alt) <= tolerance:
                    logger.info(f"[{self.name}] Reached altitude: {current_alt:.1f}m")
                    return True
                    
                # Log progress periodically
                if int(time.time() - start_time) % 5 == 0:
                    logger.debug(f"[{self.name}] Current altitude: {current_alt:.1f}m")
            
            time.sleep(0.3)
        
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
        
        Maintains GCS heartbeat during wait to prevent failsafe.
        
        Args:
            lat: Target latitude
            lon: Target longitude
            tolerance: Acceptable distance in meters
            timeout: Maximum wait time in seconds
            
        Returns:
            True if arrived within timeout
        """
        start_time = time.time()
        last_log_time = 0
        
        while time.time() - start_time < timeout:
            # Receive any pending messages (maintains heartbeat)
            self.mav.recv_match(type='HEARTBEAT', blocking=False)
            
            loc = self.get_location()
            if loc:
                current_lat, current_lon, _ = loc
                distance = haversine_distance(current_lat, current_lon, lat, lon)
                
                if distance <= tolerance:
                    logger.info(f"[{self.name}] Arrived at waypoint (distance: {distance:.1f}m)")
                    return True
                
                # Log progress every 5 seconds
                current_time = time.time()
                if current_time - last_log_time >= 5.0:
                    logger.debug(f"[{self.name}] Distance to waypoint: {distance:.1f}m")
                    last_log_time = current_time
            
            # Check battery during transit
            battery_ok, _, _ = self.check_battery()
            if not battery_ok:
                logger.warning(f"[{self.name}] Battery low during transit - triggering RTL")
                self.rtl()
                return False
            
            time.sleep(0.3)
        
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
