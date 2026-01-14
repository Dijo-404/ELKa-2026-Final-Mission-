#!/usr/bin/env python3
"""
Mission Configuration - Central configuration for multi-drone mission

All tunable parameters for Scout and Delivery drone missions.
Modify these values to adjust mission behavior.

Cross-platform compatible: Works on Linux (Arch) and Windows 10/11
"""

from dataclasses import dataclass, field
from typing import List
from pathlib import Path
import os
import platform


@dataclass
class MissionConfig:
    """
    Central configuration class for multi-drone mission.
    
    Modify these values to tune mission parameters.
    """
    
    # =========================================================================
    # CONNECTION SETTINGS (Auto-detected based on OS)
    # =========================================================================
    # Windows: Use COM ports (e.g., "COM3", "COM4")
    # Linux: Use /dev/ttyACM* or /dev/ttyUSB* (e.g., "/dev/ttyACM0")
    # macOS: Use /dev/tty.usbserial* or /dev/tty.usbmodem*
    
    # Scout Drone (Survey & Detection) - 4S battery (~14-17V)
    # Default will be set in __post_init__ based on platform
    SCOUT_CONNECTION: str = ""
    SCOUT_BAUD: int = 57600
    SCOUT_NAME: str = "Scout"
    
    # Delivery Drone (Payload Drop) - 6S battery (~21-25V)
    # Default will be set in __post_init__ based on platform
    DELIVERY_CONNECTION: str = ""
    DELIVERY_BAUD: int = 57600
    DELIVERY_NAME: str = "Delivery"
    
    # Connection timeout (seconds)
    CONNECTION_TIMEOUT: float = 10.0
    
    def __post_init__(self):
        """Set platform-specific defaults after initialization."""
        # Only set defaults if not already specified
        if not self.SCOUT_CONNECTION:
            self.SCOUT_CONNECTION = self._get_default_port(1)
        if not self.DELIVERY_CONNECTION:
            self.DELIVERY_CONNECTION = self._get_default_port(0)
    
    def _get_default_port(self, index: int) -> str:
        """
        Get platform-appropriate default port.
        
        Args:
            index: Port index (0 = first port, 1 = second port)
            
        Returns:
            Default port string for current platform
        """
        system = platform.system()
        
        if system == 'Windows':
            # Windows uses COM ports (COM3, COM4, etc.)
            return f"COM{3 + index}"
        elif system == 'Darwin':
            # macOS
            return f"/dev/tty.usbserial-{index}"
        else:
            # Linux (default)
            return f"/dev/ttyACM{index}"
    
    # =========================================================================
    # FLIGHT ALTITUDES
    # =========================================================================
    
    # Survey altitude for Scout drone (meters)
    SCOUT_ALTITUDE: float = 10.0
    
    # Delivery cruise altitude (meters)
    DELIVERY_ALTITUDE: float = 10.0
    
    # Altitude to descend to for payload drop (meters)
    DROP_ALTITUDE: float = 5.0
    
    # RTL altitude (meters) - same as cruise altitude
    RTL_ALTITUDE: float = 10.0
    
    # =========================================================================
    # SURVEY SETTINGS (Scout Drone)
    # =========================================================================
    
    # Survey speed (m/s)
    SURVEY_SPEED: float = 5.0
    
    # Lawnmower pattern: spacing between sweep lines (meters)
    SWEEP_SPACING: float = 15.0
    
    # Distance between waypoints along sweep lines (meters)
    WAYPOINT_INTERVAL: float = 20.0
    
    # Waypoint arrival tolerance (meters)
    WAYPOINT_TOLERANCE: float = 2.0
    
    # Maximum time to wait at each waypoint (seconds)
    WAYPOINT_TIMEOUT: float = 120.0
    
    # =========================================================================
    # DETECTION SETTINGS
    # =========================================================================
    
    # Detection confidence threshold (70%)
    DETECTION_CONFIDENCE: float = 0.7
    
    # IOU threshold for NMS (lower = stricter, reduces duplicate detections)
    IOU_THRESHOLD: float = 0.45
    
    # Detection frame interval (process every Nth frame)
    DETECTION_INTERVAL: int = 3
    
    # Minimum time between detections at same location (seconds)
    DETECTION_COOLDOWN: float = 2.0
    
    # Tracker settings for improved accuracy
    TRACKER_BUFFER: int = 60      # Track memory in frames
    TRACKER_MATCH_THRESH: float = 0.8  # Matching threshold
    
    # =========================================================================
    # PAYLOAD SETTINGS (Delivery Drone - Cube Orange)
    # =========================================================================
    # Cube Orange AUX outputs map to SERVO channels as follows:
    #   AUX1 = SERVO9,  AUX2 = SERVO10, AUX3 = SERVO11
    #   AUX4 = SERVO12, AUX5 = SERVO13, AUX6 = SERVO14
    
    # Number of payload slots (6 servos = 6 payloads)
    PAYLOAD_COUNT: int = 6
    
    # Maximum payloads per flight (same as PAYLOAD_COUNT for single flight)
    PAYLOAD_CAPACITY: int = 6
    
    # Servo channels for each payload (AUX1-6 = SERVO9-14)
    # Index 0 = first payload, Index 5 = sixth payload
    PAYLOAD_SERVO_CHANNELS: list = field(default_factory=lambda: [9, 10, 11, 12, 13, 14])
    
    # Servo PWM for drop/release position
    DROP_SERVO_PWM: int = 1900
    
    # Servo PWM for closed/loaded position
    LOAD_SERVO_PWM: int = 1100
    
    # Hover duration during drop (seconds)
    DROP_DURATION: float = 3.0
    
    # Legacy single-servo setting (kept for backwards compatibility)
    DROP_SERVO_CHANNEL: int = 9
    
    # =========================================================================
    # VIDEO / RTSP SETTINGS
    # =========================================================================
    
    # RTSP stream URL for live feed (SIYI camera)
    RTSP_URL: str = "rtsp://192.168.144.25:8554/main.264"
    
    # Video resolution (640p optimized for YOLO model)
    VIDEO_WIDTH: int = 640
    VIDEO_HEIGHT: int = 360
    
    # Display window settings
    DISPLAY_WINDOW_NAME: str = "Multi-Drone Mission - Live Feed"
    DISPLAY_FONT_SCALE: float = 0.6
    
    # =========================================================================
    # FILE PATHS
    # =========================================================================
    
    # Base directory (will be set to project root)
    BASE_DIR: str = field(default_factory=lambda: os.path.dirname(os.path.abspath(__file__)))
    
    # KML survey area file
    KML_FILE: str = "config/survey_area.kml"
    
    # YOLO model path
    YOLO_MODEL: str = "models/best.pt"
    
    # Output directory
    OUTPUT_DIR: str = "output"
    
    # Detected targets JSON file
    TARGETS_FILE: str = "output/targets.json"
    
    # Mission log file
    LOG_FILE: str = "output/mission_log.txt"
    
    # =========================================================================
    # SAFETY SETTINGS
    # =========================================================================
    
    # Scout drone: 4S LiPo (14.8V nominal, 3.5V/cell min = 14.0V)
    SCOUT_MIN_BATTERY_VOLTAGE: float = 14.0
    SCOUT_MIN_BATTERY_PERCENT: int = 20
    
    # Delivery drone: 6S LiPo (22.2V nominal, 3.5V/cell min = 21.0V)
    DELIVERY_MIN_BATTERY_VOLTAGE: float = 21.0
    DELIVERY_MIN_BATTERY_PERCENT: int = 20
    
    # Maximum distance from home (meters) - triggers RTL if exceeded
    MAX_DISTANCE_FROM_HOME: float = 500.0
    
    # =========================================================================
    # HELPER METHODS (pathlib for cross-platform compatibility)
    # =========================================================================
    
    def get_absolute_path(self, relative_path: str) -> Path:
        """Get absolute path for a relative path (cross-platform)."""
        path = Path(relative_path)
        if path.is_absolute():
            return path
        return Path(self.BASE_DIR) / path
    
    def get_kml_path(self) -> Path:
        """Get absolute path to KML file."""
        return self.get_absolute_path(self.KML_FILE)
    
    def get_model_path(self) -> Path:
        """Get absolute path to YOLO model."""
        return self.get_absolute_path(self.YOLO_MODEL)
    
    def get_targets_path(self) -> Path:
        """Get absolute path to targets JSON file."""
        return self.get_absolute_path(self.TARGETS_FILE)
    
    def get_log_path(self) -> Path:
        """Get absolute path to log file."""
        return self.get_absolute_path(self.LOG_FILE)
    
    def get_output_dir(self) -> Path:
        """Get absolute path to output directory."""
        return self.get_absolute_path(self.OUTPUT_DIR)
    
    def ensure_directories(self):
        """Create required directories if they don't exist."""
        self.get_output_dir().mkdir(parents=True, exist_ok=True)
        self.get_kml_path().parent.mkdir(parents=True, exist_ok=True)
        self.get_model_path().parent.mkdir(parents=True, exist_ok=True)


# Default configuration instance
DEFAULT_CONFIG = MissionConfig()


# =========================================================================
# QUICK CONFIGURATION OVERRIDE
# =========================================================================
# Modify these for quick changes without editing the class above

def get_config(**overrides) -> MissionConfig:
    """
    Get configuration with optional overrides.
    
    Example:
        config = get_config(SCOUT_ALTITUDE=30.0, PAYLOAD_CAPACITY=3)
    """
    config = MissionConfig()
    for key, value in overrides.items():
        if hasattr(config, key):
            setattr(config, key, value)
        else:
            raise ValueError(f"Unknown config key: {key}")
    return config


def get_available_ports() -> list:
    """
    Get list of available serial ports (cross-platform).
    
    Returns:
        List of available port names (e.g., ['/dev/ttyACM0', 'COM3'])
    """
    import platform
    ports = []
    
    system = platform.system()
    
    if system == 'Windows':
        try:
            import serial.tools.list_ports
            for port in serial.tools.list_ports.comports():
                ports.append(port.device)
        except ImportError:
            # Fallback: check common COM ports
            for i in range(1, 20):
                ports.append(f"COM{i}")
    else:
        # Linux/macOS
        import glob
        ports.extend(glob.glob('/dev/ttyACM*'))
        ports.extend(glob.glob('/dev/ttyUSB*'))
        ports.extend(glob.glob('/dev/tty.usbserial*'))
        ports.extend(glob.glob('/dev/tty.usbmodem*'))
    
    return sorted(ports)


if __name__ == "__main__":
    # Print current configuration
    config = MissionConfig()
    print("=" * 60)
    print("MISSION CONFIGURATION")
    print("=" * 60)
    print(f"\n[Scout Drone]")
    print(f"  Connection: {config.SCOUT_CONNECTION} @ {config.SCOUT_BAUD} baud")
    print(f"  Altitude: {config.SCOUT_ALTITUDE}m")
    print(f"  Survey Speed: {config.SURVEY_SPEED} m/s")
    print(f"\n[Delivery Drone]")
    print(f"  Connection: {config.DELIVERY_CONNECTION} @ {config.DELIVERY_BAUD} baud")
    print(f"  Altitude: {config.DELIVERY_ALTITUDE}m")
    print(f"  Payload Capacity: {config.PAYLOAD_CAPACITY}")
    print(f"\n[Paths]")
    print(f"  KML File: {config.get_kml_path()}")
    print(f"  YOLO Model: {config.get_model_path()}")
    print(f"  Targets: {config.get_targets_path()}")
    print(f"\n[Video]")
    print(f"  RTSP URL: {config.RTSP_URL}")
    print("=" * 60)
