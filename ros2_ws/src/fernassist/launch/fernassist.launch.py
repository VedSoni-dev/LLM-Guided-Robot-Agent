#!/usr/bin/env python3
"""
FernAssist Main Launch File

Launches all components of the FernAssist system:
- AAC Listener
- LLM Interpreter (using Google Gemini)
- Robot Controller
- System Monitor
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for FernAssist system."""
    
    # Launch arguments
    llm_model_arg = DeclareLaunchArgument(
        'llm_model',
        default_value='gemini-pro',
        description='LLM model to use for interpretation (Gemini model)'
    )
    
    confidence_threshold_arg = DeclareLaunchArgument(
        'confidence_threshold',
        default_value='0.5',
        description='Minimum confidence threshold for processing inputs'
    )
    
    # Get package share directory
    pkg_share = FindPackageShare('fernassist')
    
    # AAC Listener Node
    aac_listener_node = Node(
        package='fernassist',
        executable='aac_listener.py',
        name='aac_listener',
        output='screen',
        parameters=[{
            'confidence_threshold': LaunchConfiguration('confidence_threshold')
        }]
    )
    
    # LLM Interpreter Node (Gemini)
    llm_interpreter_node = Node(
        package='fernassist',
        executable='llm_interpreter.py',
        name='llm_interpreter',
        output='screen',
        parameters=[{
            'llm_model': LaunchConfiguration('llm_model'),
            'max_tokens': 150,
            'temperature': 0.7,
            'api_timeout': 30.0,
            'retry_attempts': 3,
            'top_p': 0.8,
            'top_k': 40
        }]
    )
    
    # Robot Controller Node
    robot_controller_node = Node(
        package='fernassist',
        executable='robot_controller.py',
        name='robot_controller',
        output='screen',
        parameters=[{
            'max_velocity': 0.5,
            'safety_timeout': 30.0
        }]
    )
    
    # System Monitor Node
    system_monitor_node = Node(
        package='fernassist',
        executable='system_monitor.py',
        name='system_monitor',
        output='screen'
    )
    
    # Create launch description
    return LaunchDescription([
        llm_model_arg,
        confidence_threshold_arg,
        aac_listener_node,
        llm_interpreter_node,
        robot_controller_node,
        system_monitor_node
    ]) 