#!/usr/bin/env python3
"""
Video Display - Live RTSP/camera feed with mission overlays

Displays:
- Live video feed from RTSP or camera
- Detection bounding boxes and track IDs
- GPS coordinates
- Mission status and waypoint progress
- Detection statistics
"""

import logging
import time
import threading
from typing import Optional, Dict, Tuple
from datetime import datetime
from queue import Queue, Empty
import cv2
import numpy as np

logger = logging.getLogger(__name__)


class VideoDisplay:
    """
    Real-time video display with mission information overlays.
    
    Supports RTSP streams and local cameras. Can run in a separate
    thread for non-blocking operation during missions.
    """
    
    def __init__(self, config: dict = None):
        """
        Initialize video display.
        
        Args:
            config: Display configuration:
                - rtsp_url: RTSP stream URL
                - camera_index: Camera index (0, 1, etc.)
                - window_width: Display window width
                - window_height: Display window height
                - font_scale: Font scale for overlays
        """
        self.config = config or {}
        
        # Video source
        self.rtsp_url = self.config.get('rtsp_url', None)
        self.camera_index = self.config.get('camera_index', 0)
        
        # Display settings
        self.window_name = self.config.get('window_name', "Mission Feed")
        self.window_width = self.config.get('window_width', 1280)
        self.window_height = self.config.get('window_height', 720)
        self.font_scale = self.config.get('font_scale', 0.6)
        
        # State
        self.capture: Optional[cv2.VideoCapture] = None
        self.window_created = False
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.frame_queue: Queue = Queue(maxsize=2)
        
        # Display data
        self.last_frame_time = time.time()
        self.fps = 0.0
        self.current_frame: Optional[np.ndarray] = None
        
        # Colors
        self.colors = {
            'text': (255, 255, 255),
            'background': (40, 40, 40),
            'detection': (0, 255, 0),
            'warning': (0, 165, 255),
            'error': (0, 0, 255),
            'gps': (255, 200, 0),
            'waypoint': (200, 200, 200)
        }
        
        self.font = cv2.FONT_HERSHEY_SIMPLEX
    
    def open(self) -> bool:
        """
        Open video source with RTSP optimization.
        
        Returns:
            True if opened successfully
        """
        try:
            if self.rtsp_url:
                logger.info(f"Opening RTSP stream: {self.rtsp_url}")
                # Use FFMPEG backend for better RTSP handling
                self.capture = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
                
                if self.capture.isOpened():
                    # Set minimum buffer size for low latency
                    self.capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            else:
                logger.info(f"Opening camera: {self.camera_index}")
                self.capture = cv2.VideoCapture(self.camera_index)
            
            if not self.capture.isOpened():
                logger.error("Failed to open video source")
                return False
            
            logger.info("Video source opened successfully")
            return True
            
        except Exception as e:
            logger.error(f"Failed to open video source: {e}")
            return False
    
    def read_frame(self) -> Optional[np.ndarray]:
        """
        Read a frame from the video source.
        Automatically resizes to 640p for optimized model performance.
        
        Returns:
            Frame as numpy array, or None if read failed
        """
        if self.capture is None or not self.capture.isOpened():
            return None
        
        ret, frame = self.capture.read()
        if ret and frame is not None:
            # Resize to 640 width for optimized detection
            h, w = frame.shape[:2]
            target_width = 640
            if w != target_width:
                scale = target_width / w
                new_h = int(h * scale)
                frame = cv2.resize(frame, (target_width, new_h))
            
            self.current_frame = frame
            return frame
        return None
    
    def create_window(self):
        """Create display window."""
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, self.window_width, self.window_height)
        self.window_created = True
    
    def _draw_text_with_bg(self, frame: np.ndarray, text: str, 
                           pos: Tuple[int, int], color: Tuple[int, int, int],
                           bg_color: Tuple[int, int, int] = None):
        """Draw text with background rectangle."""
        bg_color = bg_color or self.colors['background']
        
        (w, h), _ = cv2.getTextSize(text, self.font, self.font_scale, 1)
        x, y = pos
        
        cv2.rectangle(frame, (x - 2, y - h - 4), (x + w + 2, y + 4), bg_color, -1)
        cv2.putText(frame, text, pos, self.font, self.font_scale, color, 1, cv2.LINE_AA)
    
    def draw_gps_overlay(self, frame: np.ndarray, lat: float, lon: float, alt: float):
        """Draw GPS coordinates overlay."""
        text = f"GPS: {lat:.6f}, {lon:.6f} | Alt: {alt:.1f}m"
        self._draw_text_with_bg(frame, text, (10, 30), self.colors['gps'])
    
    def draw_mission_status(self, frame: np.ndarray, status: str, 
                            drone_name: str = "Drone"):
        """Draw mission status overlay."""
        text = f"[{drone_name}] {status}"
        self._draw_text_with_bg(frame, text, (10, 60), self.colors['text'])
    
    def draw_waypoint_progress(self, frame: np.ndarray, current: int, total: int):
        """Draw waypoint progress bar."""
        if total == 0:
            return
        
        w = frame.shape[1]
        text = f"Waypoint: {current}/{total}"
        self._draw_text_with_bg(frame, text, (w - 180, 30), self.colors['waypoint'])
        
        # Progress bar
        bar_width = 150
        bar_height = 10
        progress = current / total
        
        x = w - 170
        y = 45
        
        cv2.rectangle(frame, (x, y), (x + bar_width, y + bar_height), 
                     self.colors['background'], -1)
        cv2.rectangle(frame, (x, y), (x + int(bar_width * progress), y + bar_height), 
                     self.colors['detection'], -1)
        cv2.rectangle(frame, (x, y), (x + bar_width, y + bar_height), 
                     self.colors['waypoint'], 1)
    
    def draw_detection_stats(self, frame: np.ndarray, current_count: int,
                             unique_count: int, total_count: int):
        """Draw detection statistics overlay."""
        h = frame.shape[0]
        
        if current_count > 0:
            text = f"HUMANS DETECTED: {current_count}"
            color = self.colors['detection']
        else:
            text = "No detections"
            color = self.colors['text']
        
        self._draw_text_with_bg(frame, text, (10, h - 60), color)
        
        if unique_count > 0:
            text = f"Unique Tracked: {unique_count}"
            self._draw_text_with_bg(frame, text, (10, h - 35), self.colors['gps'])
        
        text = f"Total Logged: {total_count}"
        self._draw_text_with_bg(frame, text, (10, h - 10), self.colors['text'])
    
    def draw_fps(self, frame: np.ndarray):
        """Draw FPS counter."""
        current_time = time.time()
        dt = current_time - self.last_frame_time
        self.last_frame_time = current_time
        
        if dt > 0:
            self.fps = 0.9 * self.fps + 0.1 * (1.0 / dt)
        
        w, h = frame.shape[1], frame.shape[0]
        text = f"FPS: {self.fps:.1f}"
        self._draw_text_with_bg(frame, text, (w - 100, h - 15), self.colors['text'])
    
    def draw_timestamp(self, frame: np.ndarray):
        """Draw current timestamp."""
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        w = frame.shape[1]
        self._draw_text_with_bg(frame, timestamp, (w - 200, 60), self.colors['text'])
    
    def show(self, frame: np.ndarray, 
             gps: Tuple[float, float, float] = None,
             status: str = None,
             drone_name: str = "Drone",
             waypoint_current: int = 0,
             waypoint_total: int = 0,
             detection_current: int = 0,
             detection_unique: int = 0,
             detection_total: int = 0) -> int:
        """
        Display frame with all overlays.
        
        Args:
            frame: Video frame to display
            gps: (lat, lon, alt) tuple
            status: Mission status string
            drone_name: Name of active drone
            waypoint_current: Current waypoint index
            waypoint_total: Total waypoint count
            detection_current: Current frame detection count
            detection_unique: Unique tracked count
            detection_total: Total logged detections
            
        Returns:
            Key code pressed (-1 if none)
        """
        if not self.window_created:
            self.create_window()
        
        if frame is None:
            return -1
        
        # Draw overlays
        if gps:
            self.draw_gps_overlay(frame, gps[0], gps[1], gps[2])
        
        if status:
            self.draw_mission_status(frame, status, drone_name)
        
        self.draw_waypoint_progress(frame, waypoint_current, waypoint_total)
        self.draw_detection_stats(frame, detection_current, detection_unique, detection_total)
        self.draw_fps(frame)
        self.draw_timestamp(frame)
        
        # Instructions
        h = frame.shape[0]
        instructions = "Q: Quit | S: Screenshot | Space: Pause"
        self._draw_text_with_bg(frame, instructions, (10, h - 85), self.colors['waypoint'])
        
        cv2.imshow(self.window_name, frame)
        return cv2.waitKey(1) & 0xFF
    
    def save_screenshot(self, frame: np.ndarray, output_dir: str = "output") -> str:
        """
        Save current frame as screenshot.
        
        Args:
            frame: Frame to save
            output_dir: Output directory
            
        Returns:
            Path to saved file
        """
        import os
        os.makedirs(output_dir, exist_ok=True)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"screenshot_{timestamp}.jpg"
        filepath = os.path.join(output_dir, filename)
        
        cv2.imwrite(filepath, frame)
        logger.info(f"Screenshot saved: {filepath}")
        
        return filepath
    
    def start_async(self):
        """Start video reading in a separate thread."""
        if self.thread is not None and self.thread.is_alive():
            return
        
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()
        logger.info("Video thread started")
    
    def _read_loop(self):
        """Background frame reading loop."""
        while self.running:
            frame = self.read_frame()
            if frame is not None:
                try:
                    # Non-blocking put, discard if queue full
                    if self.frame_queue.full():
                        try:
                            self.frame_queue.get_nowait()
                        except Empty:
                            pass
                    self.frame_queue.put(frame, block=False)
                except:
                    pass
            time.sleep(0.001)  # Small delay to prevent CPU spin
    
    def get_frame_async(self) -> Optional[np.ndarray]:
        """
        Get latest frame from async reader.
        
        Returns:
            Latest frame or None
        """
        try:
            return self.frame_queue.get_nowait()
        except Empty:
            return None
    
    def stop_async(self):
        """Stop async video reading."""
        self.running = False
        if self.thread is not None:
            self.thread.join(timeout=2)
            self.thread = None
    
    def close(self):
        """Close display and release resources."""
        self.stop_async()
        
        if self.capture is not None:
            self.capture.release()
            self.capture = None
        
        if self.window_created:
            cv2.destroyWindow(self.window_name)
            self.window_created = False
        
        logger.info("Video display closed")


# =============================================================================
# TEST CODE
# =============================================================================
if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s [%(levelname)s] %(name)s: %(message)s',
        datefmt='%H:%M:%S'
    )
    
    print("Video Display Test")
    print("=" * 50)
    
    config = {
        'camera_index': 0,
        'window_name': "Video Display Test"
    }
    
    display = VideoDisplay(config)
    
    if display.open():
        print("Press 'q' to quit, 's' for screenshot")
        
        # Simulated mission data
        waypoint = 0
        total_waypoints = 20
        detections = 0
        
        while True:
            frame = display.read_frame()
            if frame is None:
                print("Failed to read frame")
                time.sleep(0.1)
                continue
            
            # Simulated GPS
            gps = (12.9710 + waypoint * 0.0001, 77.5910 + waypoint * 0.0001, 25.0)
            
            key = display.show(
                frame,
                gps=gps,
                status="Surveying...",
                drone_name="Scout",
                waypoint_current=waypoint,
                waypoint_total=total_waypoints,
                detection_current=2,
                detection_unique=detections,
                detection_total=detections
            )
            
            if key == ord('q'):
                break
            elif key == ord('s'):
                display.save_screenshot(frame)
            elif key == ord('n'):
                waypoint = min(waypoint + 1, total_waypoints)
                detections += 1
        
        display.close()
    else:
        print("Failed to open video source")
