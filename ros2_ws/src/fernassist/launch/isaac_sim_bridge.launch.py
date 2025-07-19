#!/usr/bin/env python3
"""
Isaac Sim Bridge Launch File for FernAssist

Launches the Isaac Sim bridge and simulation environment for robot testing.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    """Generate launch description for Isaac Sim bridge."""
    
    # Launch arguments
    enable_isaac_sim_arg = DeclareLaunchArgument(
        'enable_isaac_sim',
        default_value='true',
        description='Enable Isaac Sim integration'
    )
    
    simulation_rate_arg = DeclareLaunchArgument(
        'simulation_rate',
        default_value='60.0',
        description='Simulation rate in Hz'
    )
    
    environment_arg = DeclareLaunchArgument(
        'environment',
        default_value='warehouse',
        description='Isaac Sim environment (warehouse, office, home, custom)'
    )
    
    # Get package share directory
    pkg_share = FindPackageShare('fernassist')
    
    # Isaac Sim Bridge Node
    isaac_sim_bridge_node = Node(
        package='fernassist',
        executable='ros_bridge_utils.py',
        name='isaac_sim_bridge',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_share, 'config', 'isaac_sim_bridge_config.yaml']),
            {
                'enable_isaac_sim': LaunchConfiguration('enable_isaac_sim'),
                'simulation_rate': LaunchConfiguration('simulation_rate'),
                'environment': LaunchConfiguration('environment')
            }
        ]
    )
    
    # Robot State Publisher for TF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_tf_static': True,
            'publish_frequency': 30.0
        }]
    )
    
    # TF2 Static Broadcaster
    tf2_static_broadcaster_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf2_static_broadcaster',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    # RViz for visualization (optional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_share, 'config', 'isaac_sim.rviz'])],
        condition=IfCondition(LaunchConfiguration('enable_rviz'))
    )
    
    # Isaac Sim startup script (if Isaac Sim is available)
    isaac_sim_startup = ExecuteProcess(
        cmd=['python3', '-c', '''
import sys
try:
    import omni
    print("Isaac Sim detected - starting simulation...")
    # Add Isaac Sim startup commands here
except ImportError:
    print("Isaac Sim not available - running in ROS-only mode")
    sys.exit(0)
'''],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_isaac_sim'))
    )
    
    # Delayed startup for Isaac Sim
    delayed_isaac_sim = TimerAction(
        period=5.0,
        actions=[isaac_sim_startup]
    )
    
    # Create launch description
    return LaunchDescription([
        # Launch arguments
        enable_isaac_sim_arg,
        simulation_rate_arg,
        environment_arg,
        DeclareLaunchArgument(
            'enable_rviz',
            default_value='true',
            description='Enable RViz visualization'
        ),
        
        # Nodes
        isaac_sim_bridge_node,
        robot_state_publisher_node,
        tf2_static_broadcaster_node,
        rviz_node,
        
        # Isaac Sim startup (delayed)
        delayed_isaac_sim
    ]) 