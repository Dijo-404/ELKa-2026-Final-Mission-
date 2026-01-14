#!/usr/bin/env python3
"""
Test 2: RTSP Stream Test with Detection

Cross-platform compatible (Windows/Linux/macOS)
Optimized for low-latency RTSP streaming with buffering.

Tests:
- Connect to RTSP video stream with buffer management
- Run YOLO human detection with tracking
- Display live feed at 640p resolution
- Show FPS and detection count
"""

import sys
import time
import cv2
import threading
from queue import Queue

sys.path.insert(0, '..')
from config import MissionConfig
from human_detector import HumanDetector


class RTSPReader:
    """Threaded RTSP reader with buffer management for low latency."""
    
    def __init__(self, rtsp_url: str, target_width: int = 640):
        self.rtsp_url = rtsp_url
        self.target_width = target_width
        self.cap = None
        self.frame_queue = Queue(maxsize=2)  # Small buffer for low latency
        self.running = False
        self.thread = None
        
    def start(self) -> bool:
        """Start the RTSP reader thread."""
        # Use FFMPEG backend with low-latency settings
        self.cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
        
        if not self.cap.isOpened():
            return False
        
        # Set buffer size to minimum
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()
        return True
    
    def _read_loop(self):
        """Continuously read frames in background thread."""
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.01)
                continue
            
            # Resize to target resolution
            if frame is not None:
                h, w = frame.shape[:2]
                if w != self.target_width:
                    scale = self.target_width / w
                    new_h = int(h * scale)
                    frame = cv2.resize(frame, (self.target_width, new_h))
            
            # Drop old frame if queue is full (keeps latest frame)
            if self.frame_queue.full():
                try:
                    self.frame_queue.get_nowait()
                except:
                    pass
            
            self.frame_queue.put(frame)
    
    def read(self):
        """Get the latest frame."""
        if self.frame_queue.empty():
            return None
        
        # Get latest frame, discard older ones
        frame = None
        while not self.frame_queue.empty():
            frame = self.frame_queue.get()
        return frame
    
    def stop(self):
        """Stop the reader."""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.cap:
            self.cap.release()


def main():
    print("="*60)
    print("RTSP STREAM TEST + DETECTION (640p Optimized)")
    print("="*60)
    
    config = MissionConfig()
    rtsp_url = config.RTSP_URL
    target_width = 640  # Optimized for model
    
    print(f"\nRTSP URL: {rtsp_url}")
    print(f"Resolution: {target_width}p (optimized)")
    print(f"Model: {config.get_model_path()}")
    print("\nInitializing detector...")
    
    # Initialize detector
    detector_config = {
        'model': config.get_model_path(),
        'confidence': config.DETECTION_CONFIDENCE,
        'iou_threshold': config.IOU_THRESHOLD,
        'use_tracker': True
    }
    
    detector = HumanDetector(detector_config)
    
    if detector.is_available():
        print("Detector: READY")
    else:
        print("Detector: NOT AVAILABLE (simulation mode)")
    
    print("\nConnecting to RTSP stream (with buffering)...")
    
    # Open RTSP stream with threaded reader
    reader = RTSPReader(rtsp_url, target_width=target_width)
    
    if not reader.start():
        print("\nFAILED: Could not open RTSP stream")
        print("\nTroubleshooting:")
        print("  1. Check if the camera is powered on")
        print("  2. Verify network connection to camera")
        print(f"  3. Try pinging the camera IP: ping 192.168.144.25")
        print("  4. Check if RTSP URL is correct")
        return 1
    
    print("Stream: CONNECTED")
    print("\nDisplaying stream with detection...")
    print("Press 'Q' to quit, 'S' to save screenshot")
    
    # FPS calculation
    fps = 0.0
    frame_count = 0
    start_time = time.time()
    
    window_name = "RTSP + Detection Test (640p)"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, 960, 540)  # Display larger
    
    unique_count = 0
    last_frame = None
    
    while True:
        frame = reader.read()
        
        if frame is None:
            # Show last frame if no new frame available
            if last_frame is not None:
                cv2.imshow(window_name, last_frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            continue
        
        frame_count += 1
        elapsed = time.time() - start_time
        
        if elapsed > 1.0:
            fps = frame_count / elapsed
            frame_count = 0
            start_time = time.time()
        
        # Run detection
        result = detector.detect(frame, draw=True)
        display_frame = result.frame
        
        # Get detection info
        detection_count = len(result.detections)
        unique_count = detector.get_unique_count()
        
        # Draw info overlay
        h, w = display_frame.shape[:2]
        
        # Background for text
        cv2.rectangle(display_frame, (5, 5), (250, 110), (0, 0, 0), -1)
        
        # FPS
        color = (0, 255, 0) if fps > 15 else (0, 165, 255) if fps > 10 else (0, 0, 255)
        cv2.putText(display_frame, f"FPS: {fps:.1f}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        # Resolution
        cv2.putText(display_frame, f"Resolution: {w}x{h}", (10, 55),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        # Detection count
        cv2.putText(display_frame, f"Detections: {detection_count}", (10, 80),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
        
        # Unique tracks
        cv2.putText(display_frame, f"Unique Humans: {unique_count}", (10, 105),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 1)
        
        # Instructions
        cv2.putText(display_frame, "Q=Quit  S=Screenshot", (w - 180, 25),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        last_frame = display_frame.copy()
        cv2.imshow(window_name, display_frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            filename = f"screenshot_{int(time.time())}.jpg"
            cv2.imwrite(filename, display_frame)
            print(f"Screenshot saved: {filename}")
    
    reader.stop()
    cv2.destroyAllWindows()
    
    print(f"\nTest completed")
    print(f"Total unique humans detected: {unique_count}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
