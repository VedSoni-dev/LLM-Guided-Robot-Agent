#!/usr/bin/env python3
"""
Object Detection Launch File for FernAssist

Launches the YOLOv8-based object detection system for Isaac Sim camera feeds.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    """Generate launch description for object detection."""
    
    # Launch arguments
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='yolov8n.pt',
        description='YOLOv8 model path (yolov8n.pt, yolov8s.pt, etc.)'
    )
    
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='Detection confidence threshold (0.0-1.0)'
    )
    
    detection_mode_arg = DeclareLaunchArgument(
        'detection_mode',
        default_value='rgb_only',
        description='Detection mode (rgb_only, rgb_depth, rgb_lidar)'
    )
    
    enable_tracking_arg = DeclareLaunchArgument(
        'enable_tracking',
        default_value='true',
        description='Enable object tracking'
    )
    
    enable_depth_arg = DeclareLaunchArgument(
        'enable_depth_estimation',
        default_value='false',
        description='Enable depth estimation for 3D localization'
    )
    
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/carter/camera/image_raw',
        description='Camera image topic'
    )
    
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/carter/camera/depth',
        description='Depth image topic'
    )
    
    # Get package share directory
    pkg_share = FindPackageShare('fernassist')
    
    # Object Detector Node
    object_detector_node = Node(
        package='fernassist',
        executable='detect_objects.py',
        name='object_detector',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_share, 'config', 'object_detection_config.yaml']),
            {
                'model_path': LaunchConfiguration('model_path'),
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                'detection_mode': LaunchConfiguration('detection_mode'),
                'enable_tracking': LaunchConfiguration('enable_tracking'),
                'enable_depth_estimation': LaunchConfiguration('enable_depth_estimation'),
                'camera_topic': LaunchConfiguration('camera_topic'),
                'depth_topic': LaunchConfiguration('depth_topic')
            }
        ]
    )
    
    # RViz for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_share, 'config', 'object_detection.rviz'])],
        condition=IfCondition(LaunchConfiguration('enable_rviz'))
    )
    
    # Image viewer for detection results
    image_view_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='detection_image_viewer',
        output='screen',
        arguments=['/fernassist/detection_image'],
        condition=IfCondition(LaunchConfiguration('enable_image_view'))
    )
    
    # YOLOv8 model download (if needed)
    model_download = ExecuteProcess(
        cmd=['python3', '-c', '''
import sys
try:
    from ultralytics import YOLO
    model = YOLO("yolov8n.pt")
    print("YOLOv8 model ready")
except Exception as e:
    print(f"Error loading YOLOv8: {e}")
    sys.exit(1)
'''],
        output='screen',
        condition=IfCondition(LaunchConfiguration('download_model'))
    )
    
    # Create launch description
    return LaunchDescription([
        # Launch arguments
        model_path_arg,
        confidence_threshold_arg,
        detection_mode_arg,
        enable_tracking_arg,
        enable_depth_arg,
        camera_topic_arg,
        depth_topic_arg,
        DeclareLaunchArgument(
            'enable_rviz',
            default_value='true',
            description='Enable RViz visualization'
        ),
        DeclareLaunchArgument(
            'enable_image_view',
            default_value='true',
            description='Enable image viewer for detections'
        ),
        DeclareLaunchArgument(
            'download_model',
            default_value='false',
            description='Download YOLOv8 model if not present'
        ),
        
        # Nodes
        object_detector_node,
        rviz_node,
        image_view_node,
        
        # Model download (if requested)
        model_download
    ]) 