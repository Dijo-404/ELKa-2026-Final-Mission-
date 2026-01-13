#!/usr/bin/env python3
"""
Scout Mission - Survey and Detection Drone Logic

Drone 1 (Scout) workflow:
1. Load KML polygon survey area
2. Generate lawnmower coverage path
3. Connect, arm, and takeoff
4. Fly the survey path while detecting humans
5. Log detected locations to targets.json
6. RTL when complete
"""

import json
import logging
import time
import threading
from typing import List, Dict, Optional, Tuple
from datetime import datetime
from dataclasses import dataclass, asdict

from config import MissionConfig
from drone_controller import DroneController
from kml_processor import KMLProcessor, Waypoint
from human_detector import HumanDetector, DetectionResult
from video_display import VideoDisplay

logger = logging.getLogger(__name__)


@dataclass
class DetectedTarget:
    """A detected human target with GPS coordinates."""
    id: int
    lat: float
    lon: float
    alt: float
    confidence: float
    timestamp: str
    track_id: Optional[int] = None
    
    def to_dict(self) -> dict:
        return asdict(self)


class ScoutMission:
    """
    Scout drone mission controller.
    
    Executes survey pattern while detecting humans and logging
    their GPS coordinates.
    """
    
    def __init__(self, config: MissionConfig = None):
        """
        Initialize Scout mission.
        
        Args:
            config: Mission configuration
        """
        self.config = config or MissionConfig()
        
        # Components
        self.drone: Optional[DroneController] = None
        self.kml_processor: Optional[KMLProcessor] = None
        self.detector: Optional[HumanDetector] = None
        self.display: Optional[VideoDisplay] = None
        
        # Mission state
        self.waypoints: List[Waypoint] = []
        self.current_waypoint_index = 0
        self.detected_targets: List[DetectedTarget] = []
        self.target_counter = 0
        self.mission_id = f"scout-{datetime.now().strftime('%Y%m%d-%H%M%S')}"
        
        # Control flags
        self.running = False
        self.paused = False
        self.abort_requested = False
        
        # Video thread
        self.video_thread: Optional[threading.Thread] = None
        self.current_frame = None
        self.frame_lock = threading.Lock()
    
    def setup(self, kml_path: str = None) -> bool:
        """
        Set up all mission components.
        
        Args:
            kml_path: Path to KML survey area file
            
        Returns:
            True if setup successful
        """
        logger.info("=" * 60)
        logger.info("SCOUT MISSION - SETUP")
        logger.info("=" * 60)
        
        # Ensure output directory exists
        self.config.ensure_directories()
        
        # 1. Load KML and generate waypoints
        kml_file = kml_path or self.config.get_kml_path()
        logger.info(f"Loading KML: {kml_file}")
        
        self.kml_processor = KMLProcessor({
            'sweep_spacing': self.config.SWEEP_SPACING,
            'waypoint_interval': self.config.WAYPOINT_INTERVAL,
            'altitude': self.config.SCOUT_ALTITUDE
        })
        
        if not self.kml_processor.load(kml_file):
            logger.error("Failed to load KML file")
            return False
        
        self.waypoints = self.kml_processor.generate_waypoints()
        logger.info(f"Generated {len(self.waypoints)} survey waypoints")
        
        if len(self.waypoints) == 0:
            logger.error("No waypoints generated")
            return False
        
        # 2. Initialize detector
        logger.info("Initializing human detector...")
        detector_config = {
            'model': self.config.get_model_path(),
            'confidence': self.config.DETECTION_CONFIDENCE,
            'iou_threshold': self.config.IOU_THRESHOLD,
            'use_tracker': True
        }
        
        self.detector = HumanDetector(detector_config)
        
        if self.detector.is_available():
            logger.info("Human detector ready")
        else:
            logger.warning("Detector not available - using simulation mode")
        
        # 3. Initialize video display
        logger.info("Initializing video display...")
        display_config = {
            'rtsp_url': self.config.RTSP_URL,
            'window_name': f"Scout Mission - {self.mission_id}",
            'window_width': self.config.VIDEO_WIDTH,
            'window_height': self.config.VIDEO_HEIGHT
        }
        
        self.display = VideoDisplay(display_config)
        
        # 4. Initialize drone controller
        logger.info(f"Preparing drone controller for {self.config.SCOUT_CONNECTION}...")
        self.drone = DroneController(
            self.config.SCOUT_CONNECTION,
            baud=self.config.SCOUT_BAUD,
            name=self.config.SCOUT_NAME
        )
        
        logger.info("Setup complete!")
        logger.info("=" * 60)
        
        return True
    
    def connect_drone(self) -> bool:
        """
        Connect to the Scout drone.
        
        Returns:
            True if connected successfully
        """
        if not self.drone:
            logger.error("Drone controller not initialized")
            return False
        
        return self.drone.connect(timeout=self.config.CONNECTION_TIMEOUT)
    
    def start_video(self) -> bool:
        """
        Start video capture in background thread.
        
        Returns:
            True if started successfully
        """
        if not self.display:
            return False
        
        if not self.display.open():
            logger.warning("Could not open video source - continuing without video")
            return False
        
        # Start async reading
        self.display.start_async()
        return True
    
    def process_frame(self) -> Optional[DetectionResult]:
        """
        Process current video frame for detections.
        
        Returns:
            DetectionResult if frame available
        """
        if not self.display or not self.detector:
            return None
        
        frame = self.display.get_frame_async()
        if frame is None:
            return None
        
        # Run detection
        result = self.detector.detect(frame, draw=True)
        
        # Store current frame for display
        with self.frame_lock:
            self.current_frame = result.frame
        
        return result
    
    def log_detection(self, detection_result: DetectionResult):
        """
        Log detected humans to target list.
        
        Args:
            detection_result: Detection result with new tracks
        """
        if not detection_result.has_new_detections:
            return
        
        # Get current drone position
        pos = self.drone.get_location() if self.drone else None
        
        if pos is None:
            logger.warning("Cannot geotag detection - no GPS position")
            return
        
        lat, lon, alt = pos
        
        for track_id in detection_result.new_track_ids:
            self.target_counter += 1
            
            # Calculate average confidence for this detection
            avg_conf = 0.0
            for det in detection_result.detections:
                if det.track_id == track_id:
                    avg_conf = det.confidence
                    break
            
            target = DetectedTarget(
                id=self.target_counter,
                lat=lat,
                lon=lon,
                alt=alt,
                confidence=avg_conf,
                timestamp=datetime.now().isoformat(),
                track_id=track_id
            )
            
            self.detected_targets.append(target)
            logger.info(f"TARGET #{target.id}: Human detected at ({lat:.6f}, {lon:.6f})")
        
        # Save to file
        self._save_targets()
    
    def _save_targets(self):
        """Save detected targets to JSON file."""
        output_data = {
            'mission_id': self.mission_id,
            'timestamp': datetime.now().isoformat(),
            'total_targets': len(self.detected_targets),
            'targets': [t.to_dict() for t in self.detected_targets]
        }
        
        targets_path = self.config.get_targets_path()
        
        with open(targets_path, 'w') as f:
            json.dump(output_data, f, indent=2)
        
        logger.debug(f"Saved {len(self.detected_targets)} targets to {targets_path}")
    
    def run(self) -> List[DetectedTarget]:
        """
        Execute the Scout mission.
        
        Returns:
            List of detected targets
        """
        logger.info("=" * 60)
        logger.info("SCOUT MISSION - EXECUTING")
        logger.info(f"Mission ID: {self.mission_id}")
        logger.info(f"Waypoints: {len(self.waypoints)}")
        logger.info("=" * 60)
        
        self.running = True
        
        try:
            # 1. Connect to drone
            logger.info("[1/6] Connecting to drone...")
            if not self.connect_drone():
                logger.error("Failed to connect to drone")
                return self.detected_targets
            
            # 2. Start video capture
            logger.info("[2/6] Starting video capture...")
            video_available = self.start_video()
            if video_available:
                logger.info("Video capture started")
            else:
                logger.info("Video not available - continuing with detection only")
            
            # 3. Set GUIDED mode and arm
            logger.info("[3/6] Setting GUIDED mode...")
            if not self.drone.set_mode('GUIDED'):
                logger.error("Failed to set GUIDED mode")
                return self.detected_targets
            
            logger.info("[4/6] Arming drone...")
            if not self.drone.arm():
                logger.error("Failed to arm drone")
                return self.detected_targets
            
            # 4. Takeoff
            logger.info(f"[5/6] Taking off to {self.config.SCOUT_ALTITUDE}m...")
            if not self.drone.takeoff(self.config.SCOUT_ALTITUDE):
                logger.error("Takeoff failed")
                self.drone.disarm()
                return self.detected_targets
            
            # Wait for altitude
            if not self.drone.wait_for_altitude(self.config.SCOUT_ALTITUDE, tolerance=2.0):
                logger.warning("Altitude not reached, continuing anyway")
            
            # Set survey speed
            self.drone.set_speed(self.config.SURVEY_SPEED)
            
            # 5. Execute survey
            logger.info("[6/6] Executing survey pattern...")
            self._execute_survey()
            
            # 6. RTL
            logger.info("Survey complete - Returning to launch...")
            self.drone.rtl()
            
            # Wait for landing
            logger.info("Waiting for landing...")
            self.drone.wait_for_land(timeout=120.0)
            
        except KeyboardInterrupt:
            logger.warning("Mission interrupted by user")
            if self.drone and self.drone.is_armed():
                logger.info("Triggering RTL...")
                self.drone.rtl()
        
        except Exception as e:
            logger.error(f"Mission error: {e}")
            if self.drone and self.drone.is_armed():
                self.drone.rtl()
        
        finally:
            self.running = False
            self._cleanup()
        
        # Final save
        self._save_targets()
        
        logger.info("=" * 60)
        logger.info("SCOUT MISSION - COMPLETE")
        logger.info(f"Waypoints visited: {self.current_waypoint_index}/{len(self.waypoints)}")
        logger.info(f"Targets detected: {len(self.detected_targets)}")
        logger.info("=" * 60)
        
        return self.detected_targets
    
    def _execute_survey(self):
        """Execute the survey pattern, processing video at each waypoint."""
        total_waypoints = len(self.waypoints)
        frame_counter = 0
        
        for i, wp in enumerate(self.waypoints):
            if self.abort_requested:
                logger.warning("Mission abort requested")
                break
            
            self.current_waypoint_index = i + 1
            
            logger.info(f"Waypoint {self.current_waypoint_index}/{total_waypoints}: "
                       f"({wp.lat:.6f}, {wp.lon:.6f})")
            
            # Navigate to waypoint
            if not self.drone.goto(wp.lat, wp.lon, wp.alt):
                logger.warning("Navigation failed - skipping waypoint")
                continue
            
            # Wait for arrival while processing video
            start_time = time.time()
            arrived = False
            
            while time.time() - start_time < self.config.WAYPOINT_TIMEOUT:
                if self.abort_requested:
                    break
                
                # Process video frame (every Nth frame)
                frame_counter += 1
                if frame_counter % self.config.DETECTION_INTERVAL == 0:
                    result = self.process_frame()
                    if result and result.has_new_detections:
                        self.log_detection(result)
                
                # Update display
                self._update_display()
                
                # Check arrival
                loc = self.drone.get_location()
                if loc:
                    from drone_controller import haversine_distance
                    distance = haversine_distance(loc[0], loc[1], wp.lat, wp.lon)
                    if distance <= self.config.WAYPOINT_TOLERANCE:
                        arrived = True
                        break
                
                # Handle keyboard input
                if self._handle_input():
                    break
                
                time.sleep(0.05)  # 20Hz loop
            
            if arrived:
                logger.debug(f"Arrived at waypoint {self.current_waypoint_index}")
            else:
                logger.warning(f"Timeout at waypoint {self.current_waypoint_index}")
    
    def _update_display(self):
        """Update video display with current state."""
        if not self.display or not self.display.window_created:
            return
        
        with self.frame_lock:
            frame = self.current_frame
        
        if frame is None:
            return
        
        # Get GPS position
        gps = self.drone.get_location() if self.drone else None
        
        # Show frame with overlays
        self.display.show(
            frame,
            gps=gps,
            status="Surveying..." if not self.paused else "PAUSED",
            drone_name="Scout",
            waypoint_current=self.current_waypoint_index,
            waypoint_total=len(self.waypoints),
            detection_current=self.detector.frames_processed if self.detector else 0,
            detection_unique=self.detector.get_unique_count() if self.detector else 0,
            detection_total=len(self.detected_targets)
        )
    
    def _handle_input(self) -> bool:
        """
        Handle keyboard input.
        
        Returns:
            True if mission should stop
        """
        import cv2
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            logger.info("Quit requested")
            self.abort_requested = True
            return True
        
        elif key == ord(' '):
            self.paused = not self.paused
            logger.info(f"Mission {'PAUSED' if self.paused else 'RESUMED'}")
        
        elif key == ord('s'):
            if self.display and self.current_frame is not None:
                self.display.save_screenshot(
                    self.current_frame, 
                    self.config.get_output_dir()
                )
        
        return False
    
    def _cleanup(self):
        """Clean up resources."""
        if self.display:
            self.display.close()
        
        if self.drone:
            self.drone.disconnect()
    
    def get_status(self) -> dict:
        """Get current mission status."""
        return {
            'mission_id': self.mission_id,
            'running': self.running,
            'paused': self.paused,
            'current_waypoint': self.current_waypoint_index,
            'total_waypoints': len(self.waypoints),
            'targets_detected': len(self.detected_targets),
            'unique_tracks': self.detector.get_unique_count() if self.detector else 0
        }


# =============================================================================
# STANDALONE EXECUTION
# =============================================================================
if __name__ == "__main__":
    import argparse
    
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
        datefmt='%H:%M:%S'
    )
    
    parser = argparse.ArgumentParser(description='Scout Drone Survey Mission')
    parser.add_argument('--kml', help='Path to KML survey area file')
    parser.add_argument('--connection', default='/dev/ttyUSB0',
                       help='Drone connection string')
    parser.add_argument('--rtsp', help='RTSP stream URL')
    parser.add_argument('--simulate', action='store_true',
                       help='Use simulated detection')
    
    args = parser.parse_args()
    
    # Create configuration
    config = MissionConfig()
    config.SCOUT_CONNECTION = args.connection
    
    if args.rtsp:
        config.RTSP_URL = args.rtsp
    
    if args.kml:
        config.KML_FILE = args.kml
    
    # Run mission
    mission = ScoutMission(config)
    
    if mission.setup(kml_path=args.kml):
        targets = mission.run()
        print(f"\nMission complete. {len(targets)} targets detected.")
        
        for t in targets:
            print(f"  Target {t.id}: ({t.lat:.6f}, {t.lon:.6f})")
    else:
        print("Mission setup failed!")
