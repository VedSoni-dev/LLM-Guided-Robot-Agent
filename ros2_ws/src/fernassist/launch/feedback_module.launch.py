#!/usr/bin/env python3
"""
Feedback Module Launch File for FernAssist

Launches the multi-modal feedback system with TTS, visual feedback, and error handling.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    """Generate launch description for feedback module."""
    
    # Launch arguments
    enable_tts_arg = DeclareLaunchArgument(
        'enable_tts',
        default_value='true',
        description='Enable text-to-speech feedback'
    )
    
    tts_provider_arg = DeclareLaunchArgument(
        'tts_provider',
        default_value='pyttsx3',
        description='TTS provider (pyttsx3, gtts, espeak, festival)'
    )
    
    enable_visual_arg = DeclareLaunchArgument(
        'enable_visual_feedback',
        default_value='true',
        description='Enable visual feedback'
    )
    
    enable_aac_arg = DeclareLaunchArgument(
        'enable_aac_feedback',
        default_value='true',
        description='Enable AAC app feedback'
    )
    
    enable_error_recovery_arg = DeclareLaunchArgument(
        'enable_error_recovery',
        default_value='true',
        description='Enable automatic error recovery'
    )
    
    # Get package share directory
    pkg_share = FindPackageShare('fernassist')
    
    # Feedback Module Node
    feedback_module_node = Node(
        package='fernassist',
        executable='feedback_module.py',
        name='feedback_module',
        output='screen',
        parameters=[
            PathJoinSubstitution([pkg_share, 'config', 'feedback_config.yaml']),
            {
                'enable_tts': LaunchConfiguration('enable_tts'),
                'tts_provider': LaunchConfiguration('tts_provider'),
                'enable_visual_feedback': LaunchConfiguration('enable_visual_feedback'),
                'enable_aac_feedback': LaunchConfiguration('enable_aac_feedback'),
                'enable_error_recovery': LaunchConfiguration('enable_error_recovery')
            }
        ]
    )
    
    # TTS Test Node (optional)
    tts_test_node = Node(
        package='fernassist',
        executable='tts_test.py',
        name='tts_test',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_tts_test'))
    )
    
    # Audio player for gTTS (if using Google TTS)
    audio_player_node = Node(
        package='audio_play',
        executable='audio_play',
        name='audio_player',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_audio_player'))
    )
    
    # RViz for visual feedback
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([pkg_share, 'config', 'feedback.rviz'])],
        condition=IfCondition(LaunchConfiguration('enable_rviz'))
    )
    
    # TTS dependency check
    tts_dependency_check = ExecuteProcess(
        cmd=['python3', '-c', '''
import sys
try:
    import pyttsx3
    print("✅ pyttsx3 available")
except ImportError:
    print("❌ pyttsx3 not available - install with: pip install pyttsx3")

try:
    from gtts import gTTS
    import pygame
    print("✅ gTTS available")
except ImportError:
    print("❌ gTTS not available - install with: pip install gtts pygame")
'''],
        output='screen',
        condition=IfCondition(LaunchConfiguration('check_dependencies'))
    )
    
    # Create launch description
    return LaunchDescription([
        # Launch arguments
        enable_tts_arg,
        tts_provider_arg,
        enable_visual_arg,
        enable_aac_arg,
        enable_error_recovery_arg,
        DeclareLaunchArgument(
            'enable_tts_test',
            default_value='false',
            description='Enable TTS test node'
        ),
        DeclareLaunchArgument(
            'enable_audio_player',
            default_value='false',
            description='Enable audio player for gTTS'
        ),
        DeclareLaunchArgument(
            'enable_rviz',
            default_value='true',
            description='Enable RViz visualization'
        ),
        DeclareLaunchArgument(
            'check_dependencies',
            default_value='true',
            description='Check TTS dependencies'
        ),
        
        # Nodes
        feedback_module_node,
        tts_test_node,
        audio_player_node,
        rviz_node,
        
        # Dependency check
        tts_dependency_check
    ]) 