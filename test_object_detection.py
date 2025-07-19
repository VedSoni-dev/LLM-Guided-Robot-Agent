#!/usr/bin/env python3
"""
Test script for FernAssist Object Detection

This script tests the YOLOv8-based object detection system with various inputs
and demonstrates integration with Isaac Sim camera feeds.
"""

import os
import sys
import time
import json
import cv2
import numpy as np
from pathlib import Path

def test_yolov8_availability():
    """Test if YOLOv8 is available."""
    print("\n🤖 Testing YOLOv8 Availability")
    print("=" * 50)
    
    try:
        from ultralytics import YOLO
        
        # Test model loading
        model = YOLO('yolov8n.pt')
        
        print("✅ YOLOv8 is available")
        print(f"   Model loaded: yolov8n.pt")
        print(f"   Available classes: {len(model.names)}")
        
        # Show some class names
        class_names = list(model.names.values())[:10]
        print(f"   Sample classes: {class_names}")
        
        return True
        
    except ImportError as e:
        print("❌ YOLOv8 not available")
        print(f"   Error: {e}")
        print("💡 Install with: pip install ultralytics")
        return False
    except Exception as e:
        print(f"❌ Error loading YOLOv8 model: {e}")
        return False

def test_object_detector_creation():
    """Test object detector creation and initialization."""
    print("\n🔍 Testing Object Detector Creation")
    print("=" * 50)
    
    try:
        from scripts.detect_objects import ObjectDetector, DetectionConfig, DetectionMode
        
        # Create detector
        detector = ObjectDetector()
        
        print("✅ Object Detector created successfully")
        print(f"   Detection mode: {detector.config.detection_mode.value}")
        print(f"   Confidence threshold: {detector.config.confidence_threshold}")
        print(f"   Max detections: {detector.config.max_detections}")
        print(f"   Tracking enabled: {detector.config.enable_tracking}")
        print(f"   Depth estimation: {detector.config.enable_depth_estimation}")
        
        return detector
        
    except Exception as e:
        print(f"❌ Failed to create object detector: {e}")
        return None

def test_image_detection(detector):
    """Test object detection on sample images."""
    print("\n📸 Testing Image Detection")
    print("=" * 50)
    
    if not detector:
        print("❌ No detector available")
        return False
    
    # Create a test image with simple shapes
    test_image = create_test_image()
    
    # Save test image
    cv2.imwrite('/tmp/test_image.jpg', test_image)
    print("✅ Created test image: /tmp/test_image.jpg")
    
    # Perform detection
    print("🧪 Running object detection...")
    start_time = time.time()
    detections = detector._detect_objects(test_image)
    processing_time = time.time() - start_time
    
    print(f"✅ Detection completed in {processing_time:.3f}s")
    print(f"   Detections found: {len(detections)}")
    
    # Display results
    for i, detection in enumerate(detections):
        print(f"   {i+1}. {detection.class_name} (confidence: {detection.confidence:.2f})")
        print(f"      Bbox: {detection.bbox_2d}")
        if detection.position_3d:
            print(f"      3D Position: {detection.position_3d}")
    
    # Create annotated image
    annotated_image = test_image.copy()
    for detection in detections:
        x1, y1, x2, y2 = map(int, detection.bbox_2d)
        cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        label = f"{detection.class_name} {detection.confidence:.2f}"
        cv2.putText(annotated_image, label, (x1, y1 - 10),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    cv2.imwrite('/tmp/detection_result.jpg', annotated_image)
    print("✅ Saved detection result: /tmp/detection_result.jpg")
    
    return True

def test_depth_integration(detector):
    """Test depth integration for 3D localization."""
    print("\n📏 Testing Depth Integration")
    print("=" * 50)
    
    if not detector:
        print("❌ No detector available")
        return False
    
    # Create test RGB image
    rgb_image = create_test_image()
    
    # Create test depth image
    depth_image = create_test_depth_image()
    
    # Simulate depth callback
    detector.current_depth_image = depth_image
    detector.depth_header = None
    
    # Simulate camera calibration
    detector.camera_matrix = np.array([
        [525.0, 0.0, 319.5],
        [0.0, 525.0, 239.5],
        [0.0, 0.0, 1.0]
    ])
    
    print("✅ Set up depth integration")
    print("   Camera matrix configured")
    print("   Depth image created")
    
    # Test detection with depth
    detector.config.enable_depth_estimation = True
    detections = detector._detect_objects(rgb_image)
    
    print(f"✅ Detection with depth completed")
    print(f"   Detections: {len(detections)}")
    
    # Check 3D information
    for detection in detections:
        if detection.position_3d:
            print(f"   {detection.class_name}: 3D pos {detection.position_3d}, depth {detection.depth:.2f}m")
        else:
            print(f"   {detection.class_name}: No 3D info")
    
    return True

def test_object_tracking(detector):
    """Test object tracking functionality."""
    print("\n🎯 Testing Object Tracking")
    print("=" * 50)
    
    if not detector:
        print("❌ No detector available")
        return False
    
    detector.config.enable_tracking = True
    
    # Create sequence of test images (simulating movement)
    test_images = []
    for i in range(5):
        img = create_test_image_with_movement(i)
        test_images.append(img)
    
    print("✅ Created test image sequence")
    
    # Process sequence
    track_ids = set()
    for i, image in enumerate(test_images):
        detections = detector._detect_objects(image)
        
        for detection in detections:
            if detection.track_id is not None:
                track_ids.add(detection.track_id)
        
        print(f"   Frame {i+1}: {len(detections)} detections, {len(track_ids)} unique tracks")
    
    print(f"✅ Tracking test completed")
    print(f"   Total unique tracks: {len(track_ids)}")
    
    return True

def test_ros_integration(detector):
    """Test ROS 2 integration."""
    print("\n📡 Testing ROS 2 Integration")
    print("=" * 50)
    
    if not detector:
        print("❌ No detector available")
        return False
    
    try:
        import rospy
        from sensor_msgs.msg import Image
        from std_msgs.msg import Header
        
        # Create test ROS image message
        test_image = create_test_image()
        ros_image = detector._cv2_to_ros_image(test_image, Header())
        
        print("✅ ROS image conversion successful")
        print(f"   Image size: {ros_image.width}x{ros_image.height}")
        print(f"   Encoding: {ros_image.encoding}")
        
        # Test detection publishing
        detections = detector._detect_objects(test_image)
        detector._publish_detections(detections, Header())
        
        print("✅ Detection publishing successful")
        print(f"   Published {len(detections)} detections")
        
        return True
        
    except Exception as e:
        print(f"❌ ROS integration test failed: {e}")
        return False

def test_performance(detector):
    """Test detection performance."""
    print("\n⚡ Testing Performance")
    print("=" * 50)
    
    if not detector:
        print("❌ No detector available")
        return False
    
    # Create test image
    test_image = create_test_image()
    
    # Performance test
    num_runs = 10
    processing_times = []
    
    print(f"🧪 Running {num_runs} detection iterations...")
    
    for i in range(num_runs):
        start_time = time.time()
        detections = detector._detect_objects(test_image)
        processing_time = time.time() - start_time
        processing_times.append(processing_time)
        
        print(f"   Run {i+1}: {processing_time:.3f}s, {len(detections)} detections")
    
    # Calculate statistics
    avg_time = np.mean(processing_times)
    min_time = np.min(processing_times)
    max_time = np.max(processing_times)
    fps = 1.0 / avg_time
    
    print(f"✅ Performance test completed")
    print(f"   Average time: {avg_time:.3f}s")
    print(f"   Min time: {min_time:.3f}s")
    print(f"   Max time: {max_time:.3f}s")
    print(f"   FPS: {fps:.1f}")
    
    # Performance assessment
    if fps >= 10:
        print("   🟢 Performance: Good (>10 FPS)")
    elif fps >= 5:
        print("   🟡 Performance: Acceptable (5-10 FPS)")
    else:
        print("   🔴 Performance: Poor (<5 FPS)")
    
    return True

def create_test_image():
    """Create a test image with simple objects."""
    # Create blank image
    image = np.ones((480, 640, 3), dtype=np.uint8) * 255
    
    # Draw some simple shapes that might be detected
    # Person-like shape
    cv2.rectangle(image, (100, 200), (150, 400), (0, 0, 0), -1)  # Body
    cv2.circle(image, (125, 180), 30, (0, 0, 0), -1)  # Head
    
    # Car-like shape
    cv2.rectangle(image, (300, 300), (500, 350), (0, 0, 0), -1)  # Car body
    cv2.circle(image, (320, 360), 15, (0, 0, 0), -1)  # Wheel
    cv2.circle(image, (480, 360), 15, (0, 0, 0), -1)  # Wheel
    
    # Cup-like shape
    cv2.rectangle(image, (50, 50), (100, 120), (0, 0, 0), -1)  # Cup
    
    return image

def create_test_image_with_movement(frame_num):
    """Create test image with simulated movement."""
    image = create_test_image()
    
    # Simulate movement by shifting objects
    shift = frame_num * 20
    if shift < 200:
        # Shift the person
        cv2.rectangle(image, (100 + shift, 200), (150 + shift, 400), (0, 0, 0), -1)
        cv2.circle(image, (125 + shift, 180), 30, (0, 0, 0), -1)
    
    return image

def create_test_depth_image():
    """Create a test depth image."""
    # Create depth image (16-bit)
    depth_image = np.ones((480, 640), dtype=np.uint16) * 1000  # 1 meter default
    
    # Add some depth variation
    depth_image[200:400, 100:150] = 2000  # Person at 2m
    depth_image[300:350, 300:500] = 3000  # Car at 3m
    depth_image[50:120, 50:100] = 1500    # Cup at 1.5m
    
    return depth_image.astype(np.float32) / 1000.0  # Convert to meters

def show_usage_instructions():
    """Show usage instructions for object detection."""
    print("\n📖 Usage Instructions")
    print("=" * 50)
    print("1. Install YOLOv8:")
    print("   pip install ultralytics")
    print()
    print("2. Launch object detection:")
    print("   ros2 launch fernassist object_detection.launch.py")
    print()
    print("3. Test with different models:")
    print("   ros2 launch fernassist object_detection.launch.py model_path:=yolov8s.pt")
    print()
    print("4. Enable depth estimation:")
    print("   ros2 launch fernassist object_detection.launch.py enable_depth_estimation:=true")
    print()
    print("5. Monitor detections:")
    print("   ros2 topic echo /fernassist/object_detections")
    print("   ros2 topic echo /fernassist/detection_image")
    print()
    print("6. Use in Python:")
    print("   from scripts.detect_objects import ObjectDetector")
    print("   detector = ObjectDetector()")
    print("   detections = detector._detect_objects(image)")

def main():
    """Main test function."""
    print("🔍 FernAssist Object Detection Test")
    print("=" * 60)
    
    # Test YOLOv8 availability
    yolov8_available = test_yolov8_availability()
    
    # Test object detector creation
    detector = test_object_detector_creation()
    
    if detector:
        # Test various functionalities
        test_image_detection(detector)
        test_depth_integration(detector)
        test_object_tracking(detector)
        test_ros_integration(detector)
        test_performance(detector)
        
        # Cleanup
        detector.shutdown()
    
    # Show usage instructions
    show_usage_instructions()
    
    print("\n🎉 Object Detection test completed!")
    print(f"\nResults:")
    print(f"  YOLOv8 Available: {'✅' if yolov8_available else '❌'}")
    print(f"  Object Detector Created: {'✅' if detector else '❌'}")

if __name__ == "__main__":
    main() 