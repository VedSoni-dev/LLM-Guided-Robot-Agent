#!/usr/bin/env python3
"""
Simulation Runner Launch File for FernAssist

Launches the complete FernAssist pipeline:
Isaac Sim → ROS Bridge → LLM Interpreter → Object Detection → Feedback Module
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    """Generate launch description for simulation runner."""
    
    # Launch arguments
    scene_mode_arg = DeclareLaunchArgument(
        'scene_mode',
        default_value='kitchen',
        description='Simulation scene (kitchen, bedroom, living_room, office)'
    )
    
    robots_arg = DeclareLaunchArgument(
        'robots',
        default_value='turtlebot,franka',
        description='Comma-separated list of robots (turtlebot,franka,carter,humanoid)'
    )
    
    demo_mode_arg = DeclareLaunchArgument(
        'demo_mode',
        default_value='false',
        description='Run in demo mode with predefined scenarios'
    )
    
    interactive_mode_arg = DeclareLaunchArgument(
        'interactive_mode',
        default_value='false',
        description='Run in interactive mode for user input'
    )
    
    enable_screenshots_arg = DeclareLaunchArgument(
        'enable_screenshots',
        default_value='true',
        description='Enable automatic screenshot capture'
    )
    
    enable_video_arg = DeclareLaunchArgument(
        'enable_video_recording',
        default_value='false',
        description='Enable video recording'
    )
    
    log_directory_arg = DeclareLaunchArgument(
        'log_directory',
        default_value='demo/logs',
        description='Directory for logs and screenshots'
    )
    
    # Get package share directory
    pkg_share = FindPackageShare('fernassist')
    
    # Simulation Runner Node
    simulation_runner_node = Node(
        package='fernassist',
        executable='sim_runner.py',
        name='simulation_runner',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_share, 'config', 'simulation_config.yaml']),
            {
                'scene_mode': LaunchConfiguration('scene_mode'),
                'robots': LaunchConfiguration('robots'),
                'demo_mode': LaunchConfiguration('demo_mode'),
                'interactive_mode': LaunchConfiguration('interactive_mode'),
                'enable_screenshots': LaunchConfiguration('enable_screenshots'),
                'enable_video_recording': LaunchConfiguration('enable_video_recording'),
                'log_directory': LaunchConfiguration('log_directory')
            }
        ]
    )
    
    # LLM Interpreter Node
    llm_interpreter_node = Node(
        package='fernassist',
        executable='llm_interpreter.py',
        name='llm_interpreter',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_share, 'config', 'fernassist_config.yaml'])
        ],
        condition=IfCondition(LaunchConfiguration('enable_llm'))
    )
    
    # ROS Bridge Node
    ros_bridge_node = Node(
        package='fernassist',
        executable='ros_bridge_utils.py',
        name='ros_bridge',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_share, 'config', 'isaac_sim_bridge_config.yaml'])
        ],
        condition=IfCondition(LaunchConfiguration('enable_ros_bridge'))
    )
    
    # Object Detection Node
    object_detection_node = Node(
        package='fernassist',
        executable='detect_objects.py',
        name='object_detector',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_share, 'config', 'object_detection_config.yaml'])
        ],
        condition=IfCondition(LaunchConfiguration('enable_object_detection'))
    )
    
    # Feedback Module Node
    feedback_module_node = Node(
        package='fernassist',
        executable='feedback_module.py',
        name='feedback_module',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_share, 'config', 'feedback_config.yaml'])
        ],
        condition=IfCondition(LaunchConfiguration('enable_feedback'))
    )
    
    # RViz for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_share, 'config', 'simulation.rviz'])],
        condition=IfCondition(LaunchConfiguration('enable_rviz'))
    )
    
    # Isaac Sim dependency check
    isaac_sim_check = ExecuteProcess(
        cmd=['python3', '-c', '''
import sys
try:
    import omni
    from omni.isaac.kit import SimulationApp
    print("✅ Isaac Sim available")
except ImportError:
    print("❌ Isaac Sim not available - install Isaac Sim")
    sys.exit(1)
'''],
        output='screen',
        condition=IfCondition(LaunchConfiguration('check_dependencies'))
    )
    
    # Create log directory
    create_log_dir = ExecuteProcess(
        cmd=['mkdir', '-p', LaunchConfiguration('log_directory')],
        output='screen'
    )
    
    # Demo scenario publisher (for demo mode)
    demo_scenario_publisher = Node(
        package='demo_scenarios',
        executable='scenario_publisher',
        name='demo_scenario_publisher',
        output='screen',
        condition=IfCondition(LaunchConfiguration('demo_mode'))
    )
    
    # Performance monitor
    performance_monitor = Node(
        package='performance_monitor',
        executable='monitor',
        name='performance_monitor',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_performance_monitoring'))
    )
    
    # Delayed start for dependent nodes
    delayed_llm = TimerAction(
        period=2.0,
        actions=[llm_interpreter_node]
    )
    
    delayed_bridge = TimerAction(
        period=3.0,
        actions=[ros_bridge_node]
    )
    
    delayed_detection = TimerAction(
        period=4.0,
        actions=[object_detection_node]
    )
    
    delayed_feedback = TimerAction(
        period=5.0,
        actions=[feedback_module_node]
    )
    
    # Create launch description
    return LaunchDescription([
        # Launch arguments
        scene_mode_arg,
        robots_arg,
        demo_mode_arg,
        interactive_mode_arg,
        enable_screenshots_arg,
        enable_video_arg,
        log_directory_arg,
        DeclareLaunchArgument(
            'enable_llm',
            default_value='true',
            description='Enable LLM interpreter'
        ),
        DeclareLaunchArgument(
            'enable_ros_bridge',
            default_value='true',
            description='Enable ROS bridge'
        ),
        DeclareLaunchArgument(
            'enable_object_detection',
            default_value='true',
            description='Enable object detection'
        ),
        DeclareLaunchArgument(
            'enable_feedback',
            default_value='true',
            description='Enable feedback module'
        ),
        DeclareLaunchArgument(
            'enable_rviz',
            default_value='true',
            description='Enable RViz visualization'
        ),
        DeclareLaunchArgument(
            'enable_performance_monitoring',
            default_value='true',
            description='Enable performance monitoring'
        ),
        DeclareLaunchArgument(
            'check_dependencies',
            default_value='true',
            description='Check system dependencies'
        ),
        
        # Setup actions
        create_log_dir,
        isaac_sim_check,
        
        # Main nodes
        simulation_runner_node,
        rviz_node,
        performance_monitor,
        
        # Delayed dependent nodes
        delayed_llm,
        delayed_bridge,
        delayed_detection,
        delayed_feedback,
        
        # Demo-specific nodes
        demo_scenario_publisher
    ]) 