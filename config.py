#!/usr/bin/env python3
"""
Mission Configuration - Central configuration for multi-drone mission

All tunable parameters for Scout and Delivery drone missions.
Modify these values to adjust mission behavior.
"""

from dataclasses import dataclass, field
from typing import List
import os


@dataclass
class MissionConfig:
    """
    Central configuration class for multi-drone mission.
    
    Modify these values to tune mission parameters.
    """
    
    # =========================================================================
    # CONNECTION SETTINGS
    # =========================================================================
    
    # Scout Drone (Survey & Detection)
    SCOUT_CONNECTION: str = "/dev/ttyUSB0"
    SCOUT_BAUD: int = 57600
    SCOUT_NAME: str = "Scout"
    
    # Delivery Drone (Payload Drop)
    DELIVERY_CONNECTION: str = "/dev/ttyUSB1"
    DELIVERY_BAUD: int = 57600
    DELIVERY_NAME: str = "Delivery"
    
    # Connection timeout (seconds)
    CONNECTION_TIMEOUT: float = 10.0
    
    # =========================================================================
    # FLIGHT ALTITUDES
    # =========================================================================
    
    # Survey altitude for Scout drone (meters)
    SCOUT_ALTITUDE: float = 10.0
    
    # Delivery cruise altitude (meters)
    DELIVERY_ALTITUDE: float = 10.0
    
    # Altitude to descend to for payload drop (meters)
    DROP_ALTITUDE: float = 5.0
    
    # RTL altitude (meters)
    RTL_ALTITUDE: float = 15.0
    
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
    
    # Detection confidence threshold
    DETECTION_CONFIDENCE: float = 0.4
    
    # IOU threshold for NMS
    IOU_THRESHOLD: float = 0.5
    
    # Detection frame interval (process every Nth frame)
    DETECTION_INTERVAL: int = 5
    
    # Minimum time between detections at same location (seconds)
    DETECTION_COOLDOWN: float = 3.0
    
    # =========================================================================
    # PAYLOAD SETTINGS (Delivery Drone)
    # =========================================================================
    
    # Maximum payloads per flight
    PAYLOAD_CAPACITY: int = 5
    
    # Servo channel for payload release
    DROP_SERVO_CHANNEL: int = 9
    
    # Servo PWM for drop position
    DROP_SERVO_PWM: int = 1900
    
    # Servo PWM for closed/loaded position
    LOAD_SERVO_PWM: int = 1100
    
    # Hover duration during drop (seconds)
    DROP_DURATION: float = 5.0
    
    # =========================================================================
    # VIDEO / RTSP SETTINGS
    # =========================================================================
    
    # RTSP stream URL for live feed (SIYI camera)
    RTSP_URL: str = "rtsp://192.168.144.25:8554/main.264"
    
    # Video resolution
    VIDEO_WIDTH: int = 1280
    VIDEO_HEIGHT: int = 720
    
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
    
    # Minimum battery voltage to continue mission
    MIN_BATTERY_VOLTAGE: float = 14.0
    
    # Minimum battery percentage to continue mission
    MIN_BATTERY_PERCENT: int = 20
    
    # Maximum distance from home (meters) - triggers RTL if exceeded
    MAX_DISTANCE_FROM_HOME: float = 500.0
    
    # =========================================================================
    # HELPER METHODS
    # =========================================================================
    
    def get_absolute_path(self, relative_path: str) -> str:
        """Get absolute path for a relative path."""
        if os.path.isabs(relative_path):
            return relative_path
        return os.path.join(self.BASE_DIR, relative_path)
    
    def get_kml_path(self) -> str:
        """Get absolute path to KML file."""
        return self.get_absolute_path(self.KML_FILE)
    
    def get_model_path(self) -> str:
        """Get absolute path to YOLO model."""
        return self.get_absolute_path(self.YOLO_MODEL)
    
    def get_targets_path(self) -> str:
        """Get absolute path to targets JSON file."""
        return self.get_absolute_path(self.TARGETS_FILE)
    
    def get_log_path(self) -> str:
        """Get absolute path to log file."""
        return self.get_absolute_path(self.LOG_FILE)
    
    def get_output_dir(self) -> str:
        """Get absolute path to output directory."""
        return self.get_absolute_path(self.OUTPUT_DIR)
    
    def ensure_directories(self):
        """Create required directories if they don't exist."""
        os.makedirs(self.get_output_dir(), exist_ok=True)
        os.makedirs(os.path.dirname(self.get_kml_path()), exist_ok=True)
        os.makedirs(os.path.dirname(self.get_model_path()), exist_ok=True)


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
