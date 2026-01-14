#!/usr/bin/env python3
"""
Test: Video Detection and Tracking

Tests YOLO detection + tracker on a video file.
Outputs:
- Processed video with detection overlays
- Detection count summary
- Optional: CSV with detection timestamps

Usage:
    python test_video_detection.py --input video.mp4
    python test_video_detection.py --input video.mp4 --output output.mp4
    python test_video_detection.py --input video.mp4 --show
"""

import sys
import time
import argparse
import cv2

sys.path.insert(0, '..')
from config import MissionConfig
from human_detector import HumanDetector


def process_video(input_path: str, output_path: str = None, show: bool = False):
    """
    Process video with detection and tracking.
    
    Args:
        input_path: Path to input video
        output_path: Path to output video (optional)
        show: Show video while processing
    """
    print("="*60)
    print("VIDEO DETECTION TEST")
    print("="*60)
    
    config = MissionConfig()
    
    print(f"\nInput: {input_path}")
    print(f"Output: {output_path or 'None (display only)'}")
    print(f"Model: {config.get_model_path()}")
    
    # Open video
    cap = cv2.VideoCapture(input_path)
    
    if not cap.isOpened():
        print(f"\nERROR: Could not open video: {input_path}")
        return 1
    
    # Get video properties
    fps = int(cap.get(cv2.CAP_PROP_FPS))
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    
    print(f"\nVideo info:")
    print(f"  Resolution: {width}x{height}")
    print(f"  FPS: {fps}")
    print(f"  Total frames: {total_frames}")
    print(f"  Duration: {total_frames/fps:.1f}s")
    
    # Resize to 640 for model
    target_width = 640
    scale = target_width / width
    target_height = int(height * scale)
    
    print(f"  Processing at: {target_width}x{target_height}")
    
    # Initialize detector
    print("\nInitializing detector...")
    detector_config = {
        'model': config.get_model_path(),
        'confidence': config.DETECTION_CONFIDENCE,
        'iou_threshold': config.IOU_THRESHOLD,
        'use_tracker': True
    }
    
    detector = HumanDetector(detector_config)
    
    if not detector.is_available():
        print("WARNING: Detector not available, running in simulation mode")
    
    # Setup output video writer
    writer = None
    if output_path:
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        writer = cv2.VideoWriter(output_path, fourcc, fps, (target_width, target_height))
        print(f"\nWriting output to: {output_path}")
    
    if show:
        cv2.namedWindow("Video Detection Test", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Video Detection Test", 960, 540)
    
    print("\nProcessing...")
    print("Press 'Q' to quit, 'P' to pause")
    
    # Processing loop
    frame_count = 0
    start_time = time.time()
    paused = False
    
    while True:
        if not paused:
            ret, frame = cap.read()
            
            if not ret:
                break
            
            frame_count += 1
            
            # Resize
            frame = cv2.resize(frame, (target_width, target_height))
            
            # Run detection
            result = detector.detect(frame, draw=True)
            display_frame = result.frame
            
            # Draw info overlay
            current_detections = len(result.detections)
            unique_count = detector.get_unique_count()
            
            # Progress bar
            progress = frame_count / total_frames
            bar_width = 200
            cv2.rectangle(display_frame, (10, 10), (10 + bar_width, 25), (50, 50, 50), -1)
            cv2.rectangle(display_frame, (10, 10), (10 + int(bar_width * progress), 25), (0, 255, 0), -1)
            
            # Text overlay
            cv2.putText(display_frame, f"Frame: {frame_count}/{total_frames} ({progress*100:.1f}%)", 
                       (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            cv2.putText(display_frame, f"Detections: {current_detections}", 
                       (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
            cv2.putText(display_frame, f"Unique Humans: {unique_count}", 
                       (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 1)
            
            # Write output
            if writer:
                writer.write(display_frame)
            
            # Show progress
            if frame_count % 30 == 0:
                elapsed = time.time() - start_time
                fps_actual = frame_count / elapsed if elapsed > 0 else 0
                remaining = (total_frames - frame_count) / fps_actual if fps_actual > 0 else 0
                print(f"  {frame_count}/{total_frames} ({progress*100:.0f}%) - {fps_actual:.1f} FPS - ETA: {remaining:.0f}s")
        
        # Display
        if show:
            cv2.imshow("Video Detection Test", display_frame if not paused else display_frame)
            
            key = cv2.waitKey(1 if not paused else 100) & 0xFF
            if key == ord('q'):
                print("\nQuitting...")
                break
            elif key == ord('p'):
                paused = not paused
                print("PAUSED" if paused else "RESUMED")
        else:
            # Check for quit in headless mode
            pass
    
    # Cleanup
    cap.release()
    if writer:
        writer.release()
    if show:
        cv2.destroyAllWindows()
    
    # Summary
    elapsed = time.time() - start_time
    unique_count = detector.get_unique_count()
    
    print("\n" + "="*60)
    print("PROCESSING COMPLETE")
    print("="*60)
    print(f"Frames processed: {frame_count}")
    print(f"Time elapsed: {elapsed:.1f}s")
    print(f"Average FPS: {frame_count/elapsed:.1f}")
    print(f"Unique humans detected: {unique_count}")
    
    if output_path:
        print(f"\nOutput saved to: {output_path}")
    
    print("="*60)
    
    return 0


def main():
    parser = argparse.ArgumentParser(
        description='Test YOLO detection + tracking on video file',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
    # Process video and save output
    python test_video_detection.py --input drone_footage.mp4 --output detected.mp4
    
    # Process and display live
    python test_video_detection.py --input drone_footage.mp4 --show
    
    # Process, save, and display
    python test_video_detection.py -i video.mp4 -o output.mp4 --show
'''
    )
    
    parser.add_argument('--input', '-i', required=True,
                       help='Input video file path')
    parser.add_argument('--output', '-o',
                       help='Output video file path (optional)')
    parser.add_argument('--show', '-s', action='store_true',
                       help='Show video while processing')
    
    args = parser.parse_args()
    
    return process_video(args.input, args.output, args.show)


if __name__ == "__main__":
    sys.exit(main())
