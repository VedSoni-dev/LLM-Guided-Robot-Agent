#!/usr/bin/env python3
"""
Robot Control Example using FernAssist ROS Bridge

This example demonstrates how to use the ROS bridge to control robots in Isaac Sim.
"""

import time
import math
from scripts.ros_bridge_utils import IsaacSimBridge, RobotConfig, RobotType

def main():
    """Main example function."""
    print("🤖 FernAssist Robot Control Example")
    print("=" * 50)
    
    # Create ROS bridge
    bridge = IsaacSimBridge()
    
    # Wait for bridge to initialize
    time.sleep(2)
    
    # Get available robots
    available_robots = bridge.get_available_robots()
    print(f"Available robots: {available_robots}")
    
    if not available_robots:
        print("No robots available. Exiting.")
        return
    
    # Use the first available robot
    robot_name = available_robots[0]
    print(f"Using robot: {robot_name}")
    
    # Example 1: Basic movement
    print("\n📋 Example 1: Basic Movement")
    print("Moving forward...")
    bridge.send_velocity_command(robot_name, [0.5, 0.0, 0.0], [0.0, 0.0, 0.0])
    time.sleep(3)
    
    print("Stopping...")
    bridge.send_velocity_command(robot_name, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    time.sleep(1)
    
    # Example 2: Turning
    print("\n📋 Example 2: Turning")
    print("Turning left...")
    bridge.send_velocity_command(robot_name, [0.0, 0.0, 0.0], [0.0, 0.0, 0.5])
    time.sleep(2)
    
    print("Stopping...")
    bridge.send_velocity_command(robot_name, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    time.sleep(1)
    
    # Example 3: Complex movement
    print("\n📋 Example 3: Complex Movement")
    print("Moving in a circle...")
    for i in range(10):
        # Forward + turn for circular motion
        bridge.send_velocity_command(robot_name, [0.3, 0.0, 0.0], [0.0, 0.0, 0.3])
        time.sleep(0.5)
    
    print("Stopping...")
    bridge.send_velocity_command(robot_name, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    time.sleep(1)
    
    # Example 4: Manipulation (if supported)
    config = bridge.get_robot_config(robot_name)
    if config.enable_manipulation:
        print("\n📋 Example 4: Manipulation")
        
        # Move joints
        print("Moving joints...")
        joint_positions = {
            "shoulder_joint": 0.5,
            "elbow_joint": -0.3,
            "wrist_joint": 0.2
        }
        bridge.send_joint_command(robot_name, joint_positions)
        time.sleep(2)
        
        # Control gripper
        print("Opening gripper...")
        bridge.send_gripper_command(robot_name, 1.0)
        time.sleep(1)
        
        print("Closing gripper...")
        bridge.send_gripper_command(robot_name, 0.0)
        time.sleep(1)
    
    # Example 5: Reading sensor data
    print("\n📋 Example 5: Reading Sensor Data")
    
    # Get robot state
    state = bridge.get_robot_state(robot_name)
    if state:
        print(f"Robot position: {state['position']}")
        print(f"Robot orientation: {state['orientation']}")
        print(f"Linear velocity: {state['linear_velocity']}")
        print(f"Angular velocity: {state['angular_velocity']}")
    
    # Get laser scan data
    laser_scan = bridge.get_laser_scan(robot_name)
    if laser_scan:
        print(f"Laser scan: {len(laser_scan.ranges)} points")
        if len(laser_scan.ranges) > 0:
            min_distance = min(laser_scan.ranges)
            print(f"Minimum distance: {min_distance:.2f} meters")
    
    # Get camera image
    camera_image = bridge.get_camera_image(robot_name)
    if camera_image:
        print(f"Camera image: {camera_image.width}x{camera_image.height}")
    
    # Example 6: Safety features
    print("\n📋 Example 6: Safety Features")
    
    # Check connection status
    connected = bridge.is_robot_connected(robot_name)
    print(f"Robot connected: {connected}")
    
    # Emergency stop
    print("Testing emergency stop...")
    bridge.emergency_stop(robot_name)
    time.sleep(1)
    
    # Example 7: Multi-robot control
    if len(available_robots) > 1:
        print("\n📋 Example 7: Multi-Robot Control")
        
        for robot in available_robots[:2]:  # Control first 2 robots
            print(f"Moving {robot}...")
            bridge.send_velocity_command(robot, [0.2, 0.0, 0.0], [0.0, 0.0, 0.0])
        
        time.sleep(2)
        
        # Stop all robots
        print("Stopping all robots...")
        bridge.emergency_stop()
    
    # Example 8: Custom robot creation
    print("\n📋 Example 8: Custom Robot Creation")
    
    # Create custom robot configuration
    custom_robot = RobotConfig(
        name="my_custom_robot",
        robot_type=RobotType.CUSTOM,
        namespace="custom",
        base_frame="custom_base_link",
        odom_frame="custom_odom",
        enable_lidar=True,
        enable_camera=True,
        enable_manipulation=False,
        max_linear_velocity=1.0,
        max_angular_velocity=0.5
    )
    
    # Add to bridge
    bridge.add_robot(custom_robot)
    print(f"Added custom robot: {custom_robot.name}")
    
    # Test custom robot
    bridge.send_velocity_command("my_custom_robot", [0.3, 0.0, 0.0], [0.0, 0.0, 0.0])
    time.sleep(1)
    bridge.send_velocity_command("my_custom_robot", [0.0, 0.0, 0.0], [0.0, 0.0, 0.0])
    
    print("\n✅ Example completed successfully!")
    
    # Cleanup
    bridge.shutdown()

if __name__ == "__main__":
    main() 