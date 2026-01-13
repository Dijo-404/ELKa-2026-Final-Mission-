#!/usr/bin/env python3
"""
Test 2: RTSP Stream Test

Tests:
- Connect to RTSP video stream
- Display live feed in window
- Show FPS counter
"""

import sys
import time
import cv2

sys.path.insert(0, '..')
from config import MissionConfig


def main():
    print("="*60)
    print("RTSP STREAM TEST")
    print("="*60)
    
    config = MissionConfig()
    rtsp_url = config.RTSP_URL
    
    print(f"\nRTSP URL: {rtsp_url}")
    print("\nAttempting to connect...")
    
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
    
    print("PASSED: Stream opened successfully")
    print("\nDisplaying stream... Press 'Q' to quit")
    
    # FPS calculation
    fps = 0.0
    frame_count = 0
    start_time = time.time()
    
    window_name = "RTSP Stream Test"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, config.VIDEO_WIDTH, config.VIDEO_HEIGHT)
    
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
        
        # Draw info overlay
        h, w = frame.shape[:2]
        
        # Background for text
        cv2.rectangle(frame, (5, 5), (250, 80), (0, 0, 0), -1)
        
        # FPS
        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Resolution
        cv2.putText(frame, f"Resolution: {w}x{h}", (10, 55),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        # URL
        cv2.putText(frame, f"Source: RTSP", (10, 75),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        # Instructions
        cv2.putText(frame, "Press Q to quit", (w - 150, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        cv2.imshow(window_name, frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()
    
    print("\nStream test completed successfully")
    return 0


if __name__ == "__main__":
    sys.exit(main())
