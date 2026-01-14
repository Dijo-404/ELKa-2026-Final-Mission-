#!/usr/bin/env python3
"""
Test 2: RTSP Stream Test with Detection

Cross-platform compatible (Windows/Linux/macOS)
Tests:
- Connect to RTSP video stream
- Run YOLO human detection with tracking
- Display live feed with detection overlays
- Show FPS and detection count
"""

import sys
import time
import cv2

sys.path.insert(0, '..')
from config import MissionConfig
from human_detector import HumanDetector


def main():
    print("="*60)
    print("RTSP STREAM TEST + DETECTION")
    print("="*60)
    
    config = MissionConfig()
    rtsp_url = config.RTSP_URL
    
    print(f"\nRTSP URL: {rtsp_url}")
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
    
    print("\nConnecting to RTSP stream...")
    
    # Open RTSP stream
    cap = cv2.VideoCapture(rtsp_url)
    
    if not cap.isOpened():
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
    
    window_name = "RTSP + Detection Test"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, config.VIDEO_WIDTH, config.VIDEO_HEIGHT)
    
    unique_count = 0
    
    while True:
        ret, frame = cap.read()
        
        if not ret:
            print("\nWARNING: Frame read failed")
            time.sleep(0.1)
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
        cv2.rectangle(display_frame, (5, 5), (280, 110), (0, 0, 0), -1)
        
        # FPS
        cv2.putText(display_frame, f"FPS: {fps:.1f}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
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
        cv2.putText(display_frame, "Q=Quit  S=Screenshot", (w - 200, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        cv2.imshow(window_name, display_frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            filename = f"screenshot_{int(time.time())}.jpg"
            cv2.imwrite(filename, display_frame)
            print(f"Screenshot saved: {filename}")
    
    cap.release()
    cv2.destroyAllWindows()
    
    print(f"\nTest completed")
    print(f"Total unique humans detected: {unique_count}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
