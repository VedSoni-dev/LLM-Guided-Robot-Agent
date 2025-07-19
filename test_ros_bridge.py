#!/usr/bin/env python3
"""
Test script for FernAssist ROS Bridge

This script demonstrates the ROS bridge functionality with Isaac Sim integration.
"""

import os
import sys
import time
import json
import threading
from pathlib import Path

def test_isaac_sim_availability():
    """Test if Isaac Sim is available."""
    print("\n🤖 Testing Isaac Sim Availability")
    print("=" * 50)
    
    try:
        import omni
        import omni.kit.commands
        from omni.isaac.core.utils import stage
        from omni.isaac.core.robots import Robot
        
        print("✅ Isaac Sim packages are available")
        print(f"   Omni version: {omni.__version__}")
        return True
        
    except ImportError as e:
        print("❌ Isaac Sim packages not available")
        print(f"   Error: {e}")
        print("💡 Install Isaac Sim to enable full simulation features")
        return False

def test_ros_bridge_creation():
    """Test ROS bridge creation and initialization."""
    print("\n🌉 Testing ROS Bridge Creation")
    print("=" * 50)
    
    try:
        from scripts.ros_bridge_utils import IsaacSimBridge, RobotConfig, RobotType
        
        # Create bridge
        bridge = IsaacSimBridge()
        
        print("✅ ROS Bridge created successfully")
        print(f"   Available robots: {bridge.get_available_robots()}")
        
        # Test robot configurations
        for robot_name in bridge.get_available_robots():
            config = bridge.get_robot_config(robot_name)
            print(f"   {robot_name}: {config.robot_type.value}")
        
        return bridge
        
    except Exception as e:
        print(f"❌ Failed to create ROS bridge: {e}")
        return None

def test_robot_commands(bridge):
    """Test sending commands to robots."""
    print("\n🎮 Testing Robot Commands")
    print("=" * 50)
    
    if not bridge:
        print("❌ No bridge available")
        return False
    
    available_robots = bridge.get_available_robots()
    if not available_robots:
        print("❌ No robots available")
        return False
    
    # Test with first available robot
    robot_name = available_robots[0]
    print(f"🧪 Testing with robot: {robot_name}")
    
    # Test velocity commands
    print("\n1. Testing velocity commands:")
    success = bridge.send_velocity_command(robot_name, [0.5, 0.0, 0.0], [0.0, 0.0, 0.2])
    print(f"   Forward + turn: {'✅' if success else '❌'}")
    
    time.sleep(1)
    
    success = bridge.send_velocity_command(robot_name, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    print(f"   Stop: {'✅' if success else '❌'}")
    
    # Test joint commands (if supported)
    config = bridge.get_robot_config(robot_name)
    if config.enable_manipulation:
        print("\n2. Testing joint commands:")
        joint_positions = {"joint1": 0.5, "joint2": -0.3}
        success = bridge.send_joint_command(robot_name, joint_positions)
        print(f"   Joint positions: {'✅' if success else '❌'}")
        
        print("\n3. Testing gripper commands:")
        success = bridge.send_gripper_command(robot_name, 0.8)
        print(f"   Gripper open: {'✅' if success else '❌'}")
    
    # Test emergency stop
    print("\n4. Testing emergency stop:")
    success = bridge.emergency_stop(robot_name)
    print(f"   Emergency stop: {'✅' if success else '❌'}")
    
    return True

def test_robot_feedback(bridge):
    """Test receiving feedback from robots."""
    print("\n📡 Testing Robot Feedback")
    print("=" * 50)
    
    if not bridge:
        print("❌ No bridge available")
        return False
    
    available_robots = bridge.get_available_robots()
    if not available_robots:
        print("❌ No robots available")
        return False
    
    # Test with first available robot
    robot_name = available_robots[0]
    print(f"🧪 Testing feedback from robot: {robot_name}")
    
    # Wait for some feedback
    print("⏳ Waiting for robot feedback...")
    time.sleep(3)
    
    # Check robot state
    state = bridge.get_robot_state(robot_name)
    if state:
        print("✅ Robot state received:")
        print(f"   Position: {state['position']}")
        print(f"   Orientation: {state['orientation']}")
        print(f"   Linear velocity: {state['linear_velocity']}")
        print(f"   Angular velocity: {state['angular_velocity']}")
        print(f"   Joint positions: {len(state['joint_positions'])} joints")
        print(f"   Last update: {time.time() - state['last_update']:.2f}s ago")
    else:
        print("❌ No robot state received")
    
    # Check specific feedback
    position = bridge.get_robot_position(robot_name)
    if position:
        print(f"   Current position: {position}")
    
    velocity = bridge.get_robot_velocity(robot_name)
    if velocity:
        print(f"   Current velocity: linear={velocity['linear']}, angular={velocity['angular']}")
    
    # Check sensor data
    laser_scan = bridge.get_laser_scan(robot_name)
    if laser_scan:
        print(f"   Laser scan: {len(laser_scan.ranges)} points")
    
    camera_image = bridge.get_camera_image(robot_name)
    if camera_image:
        print(f"   Camera image: {camera_image.width}x{camera_image.height}")
    
    # Check connection status
    connected = bridge.is_robot_connected(robot_name)
    print(f"   Robot connected: {'✅' if connected else '❌'}")
    
    return True

def test_multi_robot_support(bridge):
    """Test multi-robot support."""
    print("\n🤖 Testing Multi-Robot Support")
    print("=" * 50)
    
    if not bridge:
        print("❌ No bridge available")
        return False
    
    available_robots = bridge.get_available_robots()
    print(f"📋 Available robots: {available_robots}")
    
    if len(available_robots) < 2:
        print("⚠️  Need at least 2 robots for multi-robot test")
        return True
    
    # Test commands to multiple robots
    print("\n🧪 Testing commands to multiple robots:")
    
    for robot_name in available_robots[:2]:  # Test first 2 robots
        print(f"   Sending command to {robot_name}...")
        success = bridge.send_velocity_command(robot_name, [0.1, 0.0, 0.0], [0.0, 0.0, 0.0])
        print(f"   {robot_name}: {'✅' if success else '❌'}")
    
    # Test emergency stop all robots
    print("\n🛑 Testing emergency stop all robots:")
    success = bridge.emergency_stop()  # Stop all robots
    print(f"   Emergency stop all: {'✅' if success else '❌'}")
    
    return True

def test_custom_robot_creation(bridge):
    """Test creating custom robot configurations."""
    print("\n🔧 Testing Custom Robot Creation")
    print("=" * 50)
    
    if not bridge:
        print("❌ No bridge available")
        return False
    
    try:
        from scripts.ros_bridge_utils import RobotConfig, RobotType
        
        # Create custom robot configuration
        custom_robot = RobotConfig(
            name="custom_robot",
            robot_type=RobotType.CUSTOM,
            namespace="custom",
            base_frame="custom_base_link",
            odom_frame="custom_odom",
            enable_lidar=True,
            enable_camera=True,
            enable_manipulation=True,
            max_linear_velocity=1.5,
            max_angular_velocity=0.8
        )
        
        # Add robot to bridge
        bridge.add_robot(custom_robot)
        
        print("✅ Custom robot added successfully")
        print(f"   Name: {custom_robot.name}")
        print(f"   Type: {custom_robot.robot_type.value}")
        print(f"   Namespace: {custom_robot.namespace}")
        
        # Test with custom robot
        success = bridge.send_velocity_command("custom_robot", [0.5, 0.0, 0.0], [0.0, 0.0, 0.0])
        print(f"   Command test: {'✅' if success else '❌'}")
        
        return True
        
    except Exception as e:
        print(f"❌ Failed to create custom robot: {e}")
        return False

def show_usage_instructions():
    """Show usage instructions for the ROS bridge."""
    print("\n📖 Usage Instructions")
    print("=" * 50)
    print("1. Start Isaac Sim (if available):")
    print("   isaac-sim")
    print()
    print("2. Launch the ROS bridge:")
    print("   ros2 launch fernassist isaac_sim_bridge.launch.py")
    print()
    print("3. Test robot commands:")
    print("   ros2 topic pub /carter/cmd_vel geometry_msgs/msg/Twist")
    print()
    print("4. Monitor robot feedback:")
    print("   ros2 topic echo /carter/odom")
    print("   ros2 topic echo /carter/scan")
    print()
    print("5. Use the bridge in Python:")
    print("   from scripts.ros_bridge_utils import IsaacSimBridge")
    print("   bridge = IsaacSimBridge()")
    print("   bridge.send_velocity_command('carter', [0.5, 0, 0], [0, 0, 0])")

def main():
    """Main test function."""
    print("🌉 FernAssist ROS Bridge Test")
    print("=" * 60)
    
    # Test Isaac Sim availability
    isaac_available = test_isaac_sim_availability()
    
    # Test ROS bridge creation
    bridge = test_ros_bridge_creation()
    
    if bridge:
        # Test various functionalities
        test_robot_commands(bridge)
        test_robot_feedback(bridge)
        test_multi_robot_support(bridge)
        test_custom_robot_creation(bridge)
        
        # Cleanup
        bridge.shutdown()
    
    # Show usage instructions
    show_usage_instructions()
    
    print("\n🎉 ROS Bridge test completed!")
    print(f"\nResults:")
    print(f"  Isaac Sim Available: {'✅' if isaac_available else '❌'}")
    print(f"  ROS Bridge Created: {'✅' if bridge else '❌'}")

if __name__ == "__main__":
    main() 