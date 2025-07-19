#!/usr/bin/env python3
"""
Object Detection Example for FernAssist

This example demonstrates how to use the YOLOv8-based object detection system
with Isaac Sim camera feeds and shows various detection capabilities.
"""

import time
import cv2
import numpy as np
from scripts.detect_objects import ObjectDetector, DetectionConfig, DetectionMode

def main():
    """Main example function."""
    print("🔍 FernAssist Object Detection Example")
    print("=" * 50)
    
    # Create object detector
    detector = ObjectDetector()
    
    # Wait for initialization
    time.sleep(2)
    
    print("✅ Object detector initialized")
    print(f"   Model: {detector.config.model_path}")
    print(f"   Detection mode: {detector.config.detection_mode.value}")
    print(f"   Confidence threshold: {detector.config.confidence_threshold}")
    
    # Example 1: Basic detection on test image
    print("\n📋 Example 1: Basic Object Detection")
    print("Creating test image...")
    
    test_image = create_test_scene()
    cv2.imwrite("/tmp/test_scene.jpg", test_image)
    print("✅ Test scene saved: /tmp/test_scene.jpg")
    
    # Perform detection
    print("Running object detection...")
    start_time = time.time()
    detections = detector._detect_objects(test_image)
    processing_time = time.time() - start_time
    
    print(f"✅ Detection completed in {processing_time:.3f}s")
    print(f"   Objects detected: {len(detections)}")
    
    # Display results
    for i, detection in enumerate(detections):
        print(f"   {i+1}. {detection.class_name} (confidence: {detection.confidence:.2f})")
        print(f"      Bounding box: {detection.bbox_2d}")
    
    # Example 2: Create annotated image
    print("\n📋 Example 2: Visualization")
    print("Creating annotated image...")
    
    annotated_image = test_image.copy()
    for detection in detections:
        # Draw bounding box
        x1, y1, x2, y2 = map(int, detection.bbox_2d)
        cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        
        # Draw label
        label = f"{detection.class_name} {detection.confidence:.2f}"
        cv2.putText(annotated_image, label, (x1, y1 - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    cv2.imwrite("/tmp/annotated_scene.jpg", annotated_image)
    print("✅ Annotated image saved: /tmp/annotated_scene.jpg")
    
    # Example 3: Object tracking
    print("\n📋 Example 3: Object Tracking")
    print("Testing object tracking...")
    
    detector.config.enable_tracking = True
    
    # Create sequence of images with moving objects
    for frame in range(5):
        # Create image with movement
        moving_image = create_moving_scene(frame)
        
        # Detect objects
        frame_detections = detector._detect_objects(moving_image)
        
        print(f"   Frame {frame + 1}: {len(frame_detections)} detections")
        for detection in frame_detections:
            if detection.track_id is not None:
                print(f"      Track {detection.track_id}: {detection.class_name}")
    
    # Example 4: 3D localization (simulated)
    print("\n📋 Example 4: 3D Localization")
    print("Testing 3D object localization...")
    
    # Enable depth estimation
    detector.config.enable_depth_estimation = True
    
    # Simulate depth data
    depth_image = create_depth_map()
    detector.current_depth_image = depth_image
    
    # Simulate camera calibration
    detector.camera_matrix = np.array([
        [525.0, 0.0, 319.5],
        [0.0, 525.0, 239.5],
        [0.0, 0.0, 1.0]
    ])
    
    # Detect with 3D information
    detections_3d = detector._detect_objects(test_image)
    
    print(f"✅ 3D detection completed")
    for detection in detections_3d:
        if detection.position_3d:
            print(f"   {detection.class_name}: 3D position {detection.position_3d}, depth {detection.depth:.2f}m")
        else:
            print(f"   {detection.class_name}: No 3D information")
    
    # Example 5: Performance monitoring
    print("\n📋 Example 5: Performance Monitoring")
    print("Testing detection performance...")
    
    # Performance test
    num_runs = 10
    processing_times = []
    
    for i in range(num_runs):
        start_time = time.time()
        detector._detect_objects(test_image)
        processing_time = time.time() - start_time
        processing_times.append(processing_time)
    
    avg_time = np.mean(processing_times)
    fps = 1.0 / avg_time
    
    print(f"✅ Performance test completed")
    print(f"   Average processing time: {avg_time:.3f}s")
    print(f"   FPS: {fps:.1f}")
    
    # Example 6: Detection statistics
    print("\n📋 Example 6: Detection Statistics")
    stats = detector.get_detection_stats()
    
    print("📊 Detection Statistics:")
    print(f"   Frames processed: {stats['frame_count']}")
    print(f"   Processing time: {stats['processing_time']:.3f}s")
    print(f"   Detection history: {stats['detection_history_size']} entries")
    print(f"   Active tracks: {stats['active_tracks']}")
    print(f"   Model loaded: {stats['model_loaded']}")
    print(f"   Detection mode: {stats['detection_mode']}")
    
    # Example 7: Recent detections
    print("\n📋 Example 7: Recent Detections")
    recent_detections = detector.get_recent_detections(5)
    
    print(f"📝 Recent detections ({len(recent_detections)}):")
    for detection in recent_detections:
        print(f"   {detection.class_name} (confidence: {detection.confidence:.2f})")
    
    # Example 8: ROS 2 integration
    print("\n📋 Example 8: ROS 2 Integration")
    print("Publishing detections to ROS topics...")
    
    # Simulate ROS publishing
    try:
        from std_msgs.msg import Header
        header = Header()
        header.frame_id = "camera_frame"
        
        detector._publish_detections(detections, header)
        print("✅ Detections published to ROS topics")
        print("   Topic: /fernassist/object_detections")
        print("   Topic: /fernassist/detection_visualization")
        print("   Topic: /fernassist/detection_image")
        
    except Exception as e:
        print(f"⚠️  ROS publishing failed: {e}")
    
    print("\n✅ Object detection example completed successfully!")
    
    # Cleanup
    detector.shutdown()

def create_test_scene():
    """Create a test scene with various objects."""
    # Create blank image
    image = np.ones((480, 640, 3), dtype=np.uint8) * 255
    
    # Draw various objects that YOLOv8 can detect
    
    # Person
    cv2.rectangle(image, (100, 200), (150, 400), (0, 0, 0), -1)  # Body
    cv2.circle(image, (125, 180), 30, (0, 0, 0), -1)  # Head
    
    # Car
    cv2.rectangle(image, (300, 300), (500, 350), (0, 0, 0), -1)  # Car body
    cv2.circle(image, (320, 360), 15, (0, 0, 0), -1)  # Front wheel
    cv2.circle(image, (480, 360), 15, (0, 0, 0), -1)  # Back wheel
    
    # Cup
    cv2.rectangle(image, (50, 50), (100, 120), (0, 0, 0), -1)  # Cup body
    cv2.rectangle(image, (45, 45), (105, 55), (0, 0, 0), -1)   # Cup handle
    
    # Chair
    cv2.rectangle(image, (200, 250), (280, 350), (0, 0, 0), -1)  # Chair seat
    cv2.rectangle(image, (200, 200), (220, 250), (0, 0, 0), -1)  # Chair back
    
    # Book
    cv2.rectangle(image, (400, 100), (500, 150), (0, 0, 0), -1)  # Book
    
    return image

def create_moving_scene(frame):
    """Create a scene with moving objects."""
    image = create_test_scene()
    
    # Simulate movement by shifting objects
    shift = frame * 30
    
    # Move the person
    if shift < 200:
        # Clear original person
        cv2.rectangle(image, (100, 200), (150, 400), (255, 255, 255), -1)
        cv2.circle(image, (125, 180), 30, (255, 255, 255), -1)
        
        # Draw person in new position
        cv2.rectangle(image, (100 + shift, 200), (150 + shift, 400), (0, 0, 0), -1)
        cv2.circle(image, (125 + shift, 180), 30, (0, 0, 0), -1)
    
    return image

def create_depth_map():
    """Create a simulated depth map."""
    # Create depth image (in meters)
    depth_image = np.ones((480, 640), dtype=np.float32) * 2.0  # 2m default
    
    # Add depth variation for different objects
    depth_image[200:400, 100:150] = 1.5  # Person at 1.5m
    depth_image[300:350, 300:500] = 3.0  # Car at 3m
    depth_image[50:120, 50:100] = 0.8    # Cup at 0.8m
    depth_image[250:350, 200:280] = 2.5  # Chair at 2.5m
    depth_image[100:150, 400:500] = 1.2  # Book at 1.2m
    
    return depth_image

if __name__ == "__main__":
    main() 