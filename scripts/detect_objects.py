#!/usr/bin/env python3
"""
Object Detection Module for FernAssist

This module uses YOLOv8 for real-time object detection from Isaac Sim camera feeds.
It processes RGB images and can integrate with depth data for 3D object localization.
"""

import rospy
import cv2
import numpy as np
import json
import time
import threading
from typing import List, Dict, Optional, Tuple
from dataclasses import dataclass
from enum import Enum
from pathlib import Path

# ROS 2 message imports
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from fernassist.msg import DetectedObject, ObjectDetectionArray

# YOLOv8 imports
try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    rospy.logwarn("YOLOv8 not available - install ultralytics package")

class DetectionMode(Enum):
    """Enumeration of detection modes."""
    RGB_ONLY = "rgb_only"
    RGB_DEPTH = "rgb_depth"
    RGB_LIDAR = "rgb_lidar"

@dataclass
class DetectionConfig:
    """Configuration for object detection."""
    model_path: str = "yolov8n.pt"
    confidence_threshold: float = 0.5
    nms_threshold: float = 0.4
    max_detections: int = 50
    detection_mode: DetectionMode = DetectionMode.RGB_ONLY
    enable_tracking: bool = False
    enable_depth_estimation: bool = False
    camera_fov: float = 60.0
    camera_resolution: Tuple[int, int] = (640, 480)
    publish_visualization: bool = True
    save_detections: bool = False
    detection_history_size: int = 100

@dataclass
class DetectedObject3D:
    """3D detected object with position and metadata."""
    class_id: int
    class_name: str
    confidence: float
    bbox_2d: List[float]  # [x1, y1, x2, y2]
    position_3d: Optional[List[float]] = None  # [x, y, z]
    depth: Optional[float] = None
    size_3d: Optional[List[float]] = None  # [width, height, depth]
    orientation: Optional[List[float]] = None  # [roll, pitch, yaw]
    timestamp: float = 0.0
    track_id: Optional[int] = None

class ObjectDetector:
    """YOLOv8-based object detector for Isaac Sim camera feeds."""
    
    def __init__(self):
        rospy.init_node('object_detector', anonymous=True)
        
        # Configuration
        self.config = self._load_config()
        
        # Initialize YOLOv8 model
        self.model = None
        self._initialize_model()
        
        # State tracking
        self.detection_history: List[DetectedObject3D] = []
        self.frame_count = 0
        self.processing_time = 0.0
        self.detection_lock = threading.Lock()
        
        # Camera calibration (placeholder for future depth integration)
        self.camera_matrix = None
        self.distortion_coeffs = None
        self.camera_info = None
        
        # Initialize ROS 2 interface
        self._initialize_ros_interface()
        
        # Object tracking (if enabled)
        self.trackers = {}
        self.next_track_id = 0
        
        rospy.loginfo("Object Detector initialized")
    
    def _load_config(self) -> DetectionConfig:
        """Load detection configuration from ROS parameters."""
        config = DetectionConfig()
        
        # Load from ROS parameters
        config.model_path = rospy.get_param('~model_path', config.model_path)
        config.confidence_threshold = rospy.get_param('~confidence_threshold', config.confidence_threshold)
        config.nms_threshold = rospy.get_param('~nms_threshold', config.nms_threshold)
        config.max_detections = rospy.get_param('~max_detections', config.max_detections)
        config.detection_mode = DetectionMode(rospy.get_param('~detection_mode', config.detection_mode.value))
        config.enable_tracking = rospy.get_param('~enable_tracking', config.enable_tracking)
        config.enable_depth_estimation = rospy.get_param('~enable_depth_estimation', config.enable_depth_estimation)
        config.camera_fov = rospy.get_param('~camera_fov', config.camera_fov)
        config.camera_resolution = tuple(rospy.get_param('~camera_resolution', config.camera_resolution))
        config.publish_visualization = rospy.get_param('~publish_visualization', config.publish_visualization)
        config.save_detections = rospy.get_param('~save_detections', config.save_detections)
        config.detection_history_size = rospy.get_param('~detection_history_size', config.detection_history_size)
        
        return config
    
    def _initialize_model(self):
        """Initialize YOLOv8 model."""
        if not YOLO_AVAILABLE:
            rospy.logerr("YOLOv8 not available - install ultralytics package")
            return
        
        try:
            # Load model
            model_path = Path(self.config.model_path)
            if not model_path.exists():
                rospy.logwarn(f"Model not found at {model_path}, downloading...")
                self.model = YOLO('yolov8n.pt')  # Download default model
            else:
                self.model = YOLO(str(model_path))
            
            # Set model parameters
            self.model.conf = self.config.confidence_threshold
            self.model.iou = self.config.nms_threshold
            
            rospy.loginfo(f"YOLOv8 model loaded: {self.config.model_path}")
            
        except Exception as e:
            rospy.logerr(f"Failed to initialize YOLOv8 model: {e}")
            self.model = None
    
    def _initialize_ros_interface(self):
        """Initialize ROS 2 publishers and subscribers."""
        # Publishers
        self.detection_publisher = rospy.Publisher(
            '/fernassist/object_detections',
            ObjectDetectionArray,
            queue_size=10
        )
        
        self.visualization_publisher = rospy.Publisher(
            '/fernassist/detection_visualization',
            MarkerArray,
            queue_size=10
        )
        
        self.detection_image_publisher = rospy.Publisher(
            '/fernassist/detection_image',
            Image,
            queue_size=10
        )
        
        # Subscribers
        self.camera_subscriber = rospy.Subscriber(
            '/carter/camera/image_raw',  # Default camera topic
            Image,
            self._camera_callback
        )
        
        self.depth_subscriber = rospy.Subscriber(
            '/carter/camera/depth',  # Depth camera topic
            Image,
            self._depth_callback
        )
        
        self.camera_info_subscriber = rospy.Subscriber(
            '/carter/camera/camera_info',
            CameraInfo,
            self._camera_info_callback
        )
        
        # Service for dynamic configuration
        self.config_service = rospy.Service(
            '/fernassist/update_detection_config',
            String,  # Placeholder - would need proper service definition
            self._update_config_callback
        )
    
    def _camera_callback(self, msg: Image):
        """Process camera image for object detection."""
        try:
            # Convert ROS image to OpenCV format
            cv_image = self._ros_image_to_cv2(msg)
            
            if cv_image is None:
                return
            
            # Perform detection
            start_time = time.time()
            detections = self._detect_objects(cv_image)
            self.processing_time = time.time() - start_time
            
            # Update frame count
            self.frame_count += 1
            
            # Publish detections
            self._publish_detections(detections, msg.header)
            
            # Publish visualization
            if self.config.publish_visualization:
                self._publish_visualization(detections, cv_image, msg.header)
            
            # Save detections if enabled
            if self.config.save_detections:
                self._save_detections(detections)
            
            rospy.logdebug(f"Processed frame {self.frame_count} in {self.processing_time:.3f}s")
            
        except Exception as e:
            rospy.logerr(f"Error processing camera image: {e}")
    
    def _depth_callback(self, msg: Image):
        """Process depth image for 3D localization."""
        if not self.config.enable_depth_estimation:
            return
        
        try:
            # Convert depth image to numpy array
            depth_image = self._ros_image_to_depth(msg)
            
            if depth_image is not None:
                # Store depth data for 3D localization
                self.current_depth_image = depth_image
                self.depth_header = msg.header
                
        except Exception as e:
            rospy.logerr(f"Error processing depth image: {e}")
    
    def _camera_info_callback(self, msg: CameraInfo):
        """Process camera calibration information."""
        self.camera_info = msg
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.distortion_coeffs = np.array(msg.d)
        
        rospy.loginfo("Camera calibration loaded")
    
    def _ros_image_to_cv2(self, msg: Image) -> Optional[np.ndarray]:
        """Convert ROS Image message to OpenCV format."""
        try:
            # Handle different image encodings
            if msg.encoding == 'rgb8':
                cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            elif msg.encoding == 'bgr8':
                cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            elif msg.encoding == 'mono8':
                cv_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
            else:
                rospy.logwarn(f"Unsupported image encoding: {msg.encoding}")
                return None
            
            return cv_image
            
        except Exception as e:
            rospy.logerr(f"Error converting ROS image to CV2: {e}")
            return None
    
    def _ros_image_to_depth(self, msg: Image) -> Optional[np.ndarray]:
        """Convert ROS depth image to numpy array."""
        try:
            if msg.encoding == '16UC1':
                depth_image = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width)
                depth_image = depth_image.astype(np.float32) / 1000.0  # Convert to meters
            elif msg.encoding == '32FC1':
                depth_image = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
            else:
                rospy.logwarn(f"Unsupported depth encoding: {msg.encoding}")
                return None
            
            return depth_image
            
        except Exception as e:
            rospy.logerr(f"Error converting depth image: {e}")
            return None
    
    def _detect_objects(self, image: np.ndarray) -> List[DetectedObject3D]:
        """Perform object detection using YOLOv8."""
        if self.model is None:
            return []
        
        try:
            # Run YOLOv8 inference
            results = self.model(image, verbose=False)
            
            detections = []
            
            for result in results:
                boxes = result.boxes
                if boxes is None:
                    continue
                
                for box in boxes:
                    # Extract detection information
                    bbox = box.xyxy[0].cpu().numpy()  # [x1, y1, x2, y2]
                    confidence = float(box.conf[0].cpu().numpy())
                    class_id = int(box.cls[0].cpu().numpy())
                    class_name = self.model.names[class_id]
                    
                    # Create detection object
                    detection = DetectedObject3D(
                        class_id=class_id,
                        class_name=class_name,
                        confidence=confidence,
                        bbox_2d=bbox.tolist(),
                        timestamp=time.time()
                    )
                    
                    # Add 3D information if available
                    if self.config.enable_depth_estimation and hasattr(self, 'current_depth_image'):
                        self._add_3d_information(detection, image)
                    
                    # Add tracking ID if enabled
                    if self.config.enable_tracking:
                        detection.track_id = self._assign_track_id(detection)
                    
                    detections.append(detection)
            
            # Limit number of detections
            if len(detections) > self.config.max_detections:
                detections = sorted(detections, key=lambda x: x.confidence, reverse=True)[:self.config.max_detections]
            
            return detections
            
        except Exception as e:
            rospy.logerr(f"Error in object detection: {e}")
            return []
    
    def _add_3d_information(self, detection: DetectedObject3D, image: np.ndarray):
        """Add 3D position and size information using depth data."""
        try:
            if not hasattr(self, 'current_depth_image') or self.current_depth_image is None:
                return
            
            # Get bounding box coordinates
            x1, y1, x2, y2 = map(int, detection.bbox_2d)
            
            # Extract depth values in the bounding box
            depth_roi = self.current_depth_image[y1:y2, x1:x2]
            valid_depths = depth_roi[depth_roi > 0]
            
            if len(valid_depths) == 0:
                return
            
            # Calculate median depth (more robust than mean)
            median_depth = np.median(valid_depths)
            
            # Calculate 3D position using camera parameters
            if self.camera_matrix is not None:
                # Center of bounding box
                center_x = (x1 + x2) / 2
                center_y = (y1 + y2) / 2
                
                # Convert to 3D coordinates
                fx = self.camera_matrix[0, 0]
                fy = self.camera_matrix[1, 1]
                cx = self.camera_matrix[0, 2]
                cy = self.camera_matrix[1, 2]
                
                # 3D position in camera frame
                x_3d = (center_x - cx) * median_depth / fx
                y_3d = (center_y - cy) * median_depth / fy
                z_3d = median_depth
                
                detection.position_3d = [x_3d, y_3d, z_3d]
                detection.depth = median_depth
                
                # Estimate object size (simplified)
                bbox_width = x2 - x1
                bbox_height = y2 - y1
                width_3d = bbox_width * median_depth / fx
                height_3d = bbox_height * median_depth / fy
                depth_3d = min(width_3d, height_3d) * 0.5  # Rough estimate
                
                detection.size_3d = [width_3d, height_3d, depth_3d]
            
        except Exception as e:
            rospy.logerr(f"Error adding 3D information: {e}")
    
    def _assign_track_id(self, detection: DetectedObject3D) -> int:
        """Assign tracking ID to detection using simple IoU tracking."""
        # Simple tracking implementation - could be enhanced with more sophisticated trackers
        best_iou = 0.0
        best_track_id = None
        
        for track_id, track_info in self.trackers.items():
            if track_info['class_id'] == detection.class_id:
                iou = self._calculate_iou(detection.bbox_2d, track_info['bbox'])
                if iou > best_iou and iou > 0.3:  # IoU threshold
                    best_iou = iou
                    best_track_id = track_id
        
        if best_track_id is not None:
            # Update existing track
            self.trackers[best_track_id]['bbox'] = detection.bbox_2d
            self.trackers[best_track_id]['timestamp'] = time.time()
            return best_track_id
        else:
            # Create new track
            track_id = self.next_track_id
            self.next_track_id += 1
            self.trackers[track_id] = {
                'bbox': detection.bbox_2d,
                'class_id': detection.class_id,
                'timestamp': time.time()
            }
            return track_id
    
    def _calculate_iou(self, bbox1: List[float], bbox2: List[float]) -> float:
        """Calculate Intersection over Union between two bounding boxes."""
        x1_1, y1_1, x2_1, y2_1 = bbox1
        x1_2, y1_2, x2_2, y2_2 = bbox2
        
        # Calculate intersection
        x1_i = max(x1_1, x1_2)
        y1_i = max(y1_1, y1_2)
        x2_i = min(x2_1, x2_2)
        y2_i = min(y2_1, y2_2)
        
        if x2_i <= x1_i or y2_i <= y1_i:
            return 0.0
        
        intersection = (x2_i - x1_i) * (y2_i - y1_i)
        
        # Calculate union
        area1 = (x2_1 - x1_1) * (y2_1 - y1_1)
        area2 = (x2_2 - x1_2) * (y2_2 - y1_2)
        union = area1 + area2 - intersection
        
        return intersection / union if union > 0 else 0.0
    
    def _publish_detections(self, detections: List[DetectedObject3D], header: Header):
        """Publish detection results to ROS topics."""
        try:
            # Create detection array message
            detection_array = ObjectDetectionArray()
            detection_array.header = header
            detection_array.header.frame_id = "camera_frame"
            
            for detection in detections:
                # Create ROS detection message
                ros_detection = DetectedObject()
                ros_detection.class_id = detection.class_id
                ros_detection.class_name = detection.class_name
                ros_detection.confidence = detection.confidence
                ros_detection.bbox = detection.bbox_2d
                
                if detection.position_3d:
                    ros_detection.position.x = detection.position_3d[0]
                    ros_detection.position.y = detection.position_3d[1]
                    ros_detection.position.z = detection.position_3d[2]
                
                if detection.size_3d:
                    ros_detection.size.x = detection.size_3d[0]
                    ros_detection.size.y = detection.size_3d[1]
                    ros_detection.size.z = detection.size_3d[2]
                
                if detection.track_id is not None:
                    ros_detection.track_id = detection.track_id
                
                detection_array.detections.append(ros_detection)
            
            # Publish detection array
            self.detection_publisher.publish(detection_array)
            
            # Update detection history
            with self.detection_lock:
                self.detection_history.extend(detections)
                if len(self.detection_history) > self.config.detection_history_size:
                    self.detection_history = self.detection_history[-self.config.detection_history_size:]
            
        except Exception as e:
            rospy.logerr(f"Error publishing detections: {e}")
    
    def _publish_visualization(self, detections: List[DetectedObject3D], image: np.ndarray, header: Header):
        """Publish visualization markers and annotated image."""
        try:
            # Create annotated image
            annotated_image = image.copy()
            
            for detection in detections:
                # Draw bounding box
                x1, y1, x2, y2 = map(int, detection.bbox_2d)
                cv2.rectangle(annotated_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                # Draw label
                label = f"{detection.class_name} {detection.confidence:.2f}"
                if detection.track_id is not None:
                    label += f" ID:{detection.track_id}"
                
                cv2.putText(annotated_image, label, (x1, y1 - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Draw 3D position if available
                if detection.position_3d:
                    pos_text = f"({detection.position_3d[0]:.2f}, {detection.position_3d[1]:.2f}, {detection.position_3d[2]:.2f})"
                    cv2.putText(annotated_image, pos_text, (x1, y2 + 20),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)
            
            # Publish annotated image
            ros_image = self._cv2_to_ros_image(annotated_image, header)
            self.detection_image_publisher.publish(ros_image)
            
            # Create visualization markers
            marker_array = MarkerArray()
            
            for i, detection in enumerate(detections):
                if detection.position_3d:
                    marker = Marker()
                    marker.header = header
                    marker.header.frame_id = "camera_frame"
                    marker.id = i
                    marker.type = Marker.CUBE
                    marker.action = Marker.ADD
                    
                    marker.pose.position.x = detection.position_3d[0]
                    marker.pose.position.y = detection.position_3d[1]
                    marker.pose.position.z = detection.position_3d[2]
                    
                    if detection.size_3d:
                        marker.scale.x = detection.size_3d[0]
                        marker.scale.y = detection.size_3d[1]
                        marker.scale.z = detection.size_3d[2]
                    else:
                        marker.scale.x = 0.1
                        marker.scale.y = 0.1
                        marker.scale.z = 0.1
                    
                    # Color based on confidence
                    confidence = detection.confidence
                    marker.color.r = 1.0 - confidence
                    marker.color.g = confidence
                    marker.color.b = 0.0
                    marker.color.a = 0.7
                    
                    marker_array.markers.append(marker)
            
            # Publish markers
            if marker_array.markers:
                self.visualization_publisher.publish(marker_array)
            
        except Exception as e:
            rospy.logerr(f"Error publishing visualization: {e}")
    
    def _cv2_to_ros_image(self, cv_image: np.ndarray, header: Header) -> Image:
        """Convert OpenCV image to ROS Image message."""
        try:
            # Convert BGR to RGB
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            
            # Create ROS image message
            ros_image = Image()
            ros_image.header = header
            ros_image.height, ros_image.width = rgb_image.shape[:2]
            ros_image.encoding = 'rgb8'
            ros_image.step = ros_image.width * 3
            ros_image.data = rgb_image.tobytes()
            
            return ros_image
            
        except Exception as e:
            rospy.logerr(f"Error converting CV2 image to ROS: {e}")
            return Image()
    
    def _save_detections(self, detections: List[DetectedObject3D]):
        """Save detection results to file."""
        try:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"/tmp/detections_{timestamp}.json"
            
            detection_data = []
            for detection in detections:
                data = {
                    'class_id': detection.class_id,
                    'class_name': detection.class_name,
                    'confidence': detection.confidence,
                    'bbox_2d': detection.bbox_2d,
                    'position_3d': detection.position_3d,
                    'size_3d': detection.size_3d,
                    'track_id': detection.track_id,
                    'timestamp': detection.timestamp
                }
                detection_data.append(data)
            
            with open(filename, 'w') as f:
                json.dump(detection_data, f, indent=2)
            
            rospy.logdebug(f"Detections saved to {filename}")
            
        except Exception as e:
            rospy.logerr(f"Error saving detections: {e}")
    
    def _update_config_callback(self, request):
        """Update detection configuration dynamically."""
        try:
            # Parse configuration update
            config_update = json.loads(request.data)
            
            # Update configuration parameters
            for key, value in config_update.items():
                if hasattr(self.config, key):
                    setattr(self.config, key, value)
            
            # Update model parameters
            if self.model is not None:
                self.model.conf = self.config.confidence_threshold
                self.model.iou = self.config.nms_threshold
            
            rospy.loginfo("Detection configuration updated")
            return String(data="Configuration updated successfully")
            
        except Exception as e:
            rospy.logerr(f"Error updating configuration: {e}")
            return String(data=f"Error: {str(e)}")
    
    def get_detection_stats(self) -> Dict:
        """Get detection statistics."""
        with self.detection_lock:
            return {
                'frame_count': self.frame_count,
                'processing_time': self.processing_time,
                'detection_history_size': len(self.detection_history),
                'active_tracks': len(self.trackers),
                'model_loaded': self.model is not None,
                'detection_mode': self.config.detection_mode.value
            }
    
    def get_recent_detections(self, num_detections: int = 10) -> List[DetectedObject3D]:
        """Get recent detections from history."""
        with self.detection_lock:
            return self.detection_history[-num_detections:]
    
    def clear_detection_history(self):
        """Clear detection history."""
        with self.detection_lock:
            self.detection_history.clear()
            self.trackers.clear()
            self.next_track_id = 0
    
    def run(self):
        """Main run loop for the object detector."""
        rospy.loginfo("Object Detector started - processing camera feeds...")
        rospy.spin()
    
    def shutdown(self):
        """Shutdown the object detector."""
        rospy.loginfo("Object Detector shutting down")

# Convenience functions
def create_detector() -> ObjectDetector:
    """Create and return an object detector instance."""
    return ObjectDetector()

def detect_objects_in_image(detector: ObjectDetector, image: np.ndarray) -> List[DetectedObject3D]:
    """Detect objects in a single image."""
    return detector._detect_objects(image)

if __name__ == '__main__':
    try:
        detector = ObjectDetector()
        detector.run()
    except rospy.ROSInterruptException:
        detector.shutdown() 