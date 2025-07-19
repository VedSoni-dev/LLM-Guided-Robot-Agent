#!/usr/bin/env python3
"""
ROS Bridge Utilities for FernAssist

This module provides utilities to connect ROS 2 nodes to Isaac Sim for robot simulation.
It handles publishing commands and subscribing to feedback topics for multiple robots.
"""

import rospy
import json
import time
import threading
from typing import Dict, List, Optional, Callable
from dataclasses import dataclass
from enum import Enum

# ROS 2 message imports
from std_msgs.msg import String, Bool, Float32, Float64
from geometry_msgs.msg import Twist, Pose, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan, Image, PointCloud2, JointState
from tf2_msgs.msg import TFMessage

# Isaac Sim specific imports (if available)
try:
    import omni
    import omni.kit.commands
    from omni.isaac.core.utils import stage
    from omni.isaac.core.robots import Robot
    from omni.isaac.core.utils.nucleus import get_nucleus_server
    ISAAC_SIM_AVAILABLE = True
except ImportError:
    ISAAC_SIM_AVAILABLE = False
    rospy.logwarn("Isaac Sim packages not available - running in ROS-only mode")

class RobotType(Enum):
    """Enumeration of supported robot types."""
    MOBILE_BASE = "mobile_base"
    MANIPULATOR = "manipulator"
    HUMANOID = "humanoid"
    CUSTOM = "custom"

@dataclass
class RobotConfig:
    """Configuration for a robot in Isaac Sim."""
    name: str
    robot_type: RobotType
    namespace: str
    base_frame: str
    odom_frame: str
    map_frame: str = "map"
    cmd_vel_topic: str = "cmd_vel"
    odom_topic: str = "odom"
    joint_states_topic: str = "joint_states"
    laser_scan_topic: str = "scan"
    camera_topic: str = "camera/image_raw"
    enable_lidar: bool = True
    enable_camera: bool = True
    enable_manipulation: bool = False
    max_linear_velocity: float = 2.0
    max_angular_velocity: float = 1.0
    safety_timeout: float = 5.0

class IsaacSimBridge:
    """Bridge class to connect ROS 2 nodes to Isaac Sim robots."""
    
    def __init__(self):
        rospy.init_node('isaac_sim_bridge', anonymous=True)
        
        # Configuration
        self.simulation_rate = rospy.get_param('~simulation_rate', 60.0)
        self.enable_isaac_sim = rospy.get_param('~enable_isaac_sim', True)
        self.isaac_sim_host = rospy.get_param('~isaac_sim_host', 'localhost')
        self.isaac_sim_port = rospy.get_param('~isaac_sim_port', 11311)
        
        # Robot configurations
        self.robots: Dict[str, RobotConfig] = {}
        self.robot_publishers: Dict[str, Dict] = {}
        self.robot_subscribers: Dict[str, Dict] = {}
        self.robot_states: Dict[str, Dict] = {}
        
        # Threading
        self.bridge_lock = threading.Lock()
        self.is_running = False
        
        # Initialize Isaac Sim connection
        self.isaac_sim_connected = False
        if self.enable_isaac_sim and ISAAC_SIM_AVAILABLE:
            self._initialize_isaac_sim()
        
        # Load robot configurations
        self._load_robot_configurations()
        
        # Initialize ROS 2 publishers and subscribers
        self._initialize_ros_interface()
        
        rospy.loginfo("Isaac Sim Bridge initialized")
    
    def _initialize_isaac_sim(self):
        """Initialize connection to Isaac Sim."""
        try:
            if ISAAC_SIM_AVAILABLE:
                # Initialize Isaac Sim connection
                omni.kit.commands.execute('IsaacSimStartup')
                
                # Set up stage
                stage.add_reference_to_stage(
                    usd_path="/Isaac/Robots/Carter/carter_v1.usd",
                    prim_path="/World/Carter"
                )
                
                self.isaac_sim_connected = True
                rospy.loginfo("Isaac Sim connection established")
            else:
                rospy.logwarn("Isaac Sim not available - running in simulation mode")
                
        except Exception as e:
            rospy.logerr(f"Failed to initialize Isaac Sim: {e}")
            self.isaac_sim_connected = False
    
    def _load_robot_configurations(self):
        """Load robot configurations from parameters or defaults."""
        # Default robot configurations
        default_robots = {
            "carter": RobotConfig(
                name="carter",
                robot_type=RobotType.MOBILE_BASE,
                namespace="carter",
                base_frame="carter_base_link",
                odom_frame="carter_odom",
                enable_lidar=True,
                enable_camera=True
            ),
            "franka": RobotConfig(
                name="franka",
                robot_type=RobotType.MANIPULATOR,
                namespace="franka",
                base_frame="franka_base_link",
                odom_frame="franka_base_link",
                enable_manipulation=True,
                enable_lidar=False,
                enable_camera=True
            ),
            "humanoid": RobotConfig(
                name="humanoid",
                robot_type=RobotType.HUMANOID,
                namespace="humanoid",
                base_frame="humanoid_base_link",
                odom_frame="humanoid_base_link",
                enable_manipulation=True,
                enable_lidar=False,
                enable_camera=True
            )
        }
        
        # Load from ROS parameters if available
        for robot_name, default_config in default_robots.items():
            param_prefix = f"~robots/{robot_name}"
            
            if rospy.has_param(param_prefix):
                # Load configuration from parameters
                config = RobotConfig(
                    name=rospy.get_param(f"{param_prefix}/name", default_config.name),
                    robot_type=RobotType(rospy.get_param(f"{param_prefix}/type", default_config.robot_type.value)),
                    namespace=rospy.get_param(f"{param_prefix}/namespace", default_config.namespace),
                    base_frame=rospy.get_param(f"{param_prefix}/base_frame", default_config.base_frame),
                    odom_frame=rospy.get_param(f"{param_prefix}/odom_frame", default_config.odom_frame),
                    map_frame=rospy.get_param(f"{param_prefix}/map_frame", default_config.map_frame),
                    cmd_vel_topic=rospy.get_param(f"{param_prefix}/cmd_vel_topic", default_config.cmd_vel_topic),
                    odom_topic=rospy.get_param(f"{param_prefix}/odom_topic", default_config.odom_topic),
                    joint_states_topic=rospy.get_param(f"{param_prefix}/joint_states_topic", default_config.joint_states_topic),
                    laser_scan_topic=rospy.get_param(f"{param_prefix}/laser_scan_topic", default_config.laser_scan_topic),
                    camera_topic=rospy.get_param(f"{param_prefix}/camera_topic", default_config.camera_topic),
                    enable_lidar=rospy.get_param(f"{param_prefix}/enable_lidar", default_config.enable_lidar),
                    enable_camera=rospy.get_param(f"{param_prefix}/enable_camera", default_config.enable_camera),
                    enable_manipulation=rospy.get_param(f"{param_prefix}/enable_manipulation", default_config.enable_manipulation),
                    max_linear_velocity=rospy.get_param(f"{param_prefix}/max_linear_velocity", default_config.max_linear_velocity),
                    max_angular_velocity=rospy.get_param(f"{param_prefix}/max_angular_velocity", default_config.max_angular_velocity),
                    safety_timeout=rospy.get_param(f"{param_prefix}/safety_timeout", default_config.safety_timeout)
                )
            else:
                config = default_config
            
            self.robots[robot_name] = config
            self.robot_states[robot_name] = {
                'position': [0.0, 0.0, 0.0],
                'orientation': [0.0, 0.0, 0.0, 1.0],
                'linear_velocity': [0.0, 0.0, 0.0],
                'angular_velocity': [0.0, 0.0, 0.0],
                'joint_positions': {},
                'laser_scan': None,
                'camera_image': None,
                'last_update': time.time()
            }
        
        rospy.loginfo(f"Loaded {len(self.robots)} robot configurations")
    
    def _initialize_ros_interface(self):
        """Initialize ROS 2 publishers and subscribers for all robots."""
        for robot_name, config in self.robots.items():
            namespace = config.namespace
            
            # Publishers for commands
            publishers = {}
            publishers['cmd_vel'] = rospy.Publisher(
                f"/{namespace}/{config.cmd_vel_topic}",
                Twist,
                queue_size=10
            )
            
            if config.enable_manipulation:
                publishers['joint_commands'] = rospy.Publisher(
                    f"/{namespace}/joint_commands",
                    JointState,
                    queue_size=10
                )
                publishers['gripper_command'] = rospy.Publisher(
                    f"/{namespace}/gripper_command",
                    Float64,
                    queue_size=10
                )
            
            publishers['emergency_stop'] = rospy.Publisher(
                f"/{namespace}/emergency_stop",
                Bool,
                queue_size=1
            )
            
            self.robot_publishers[robot_name] = publishers
            
            # Subscribers for feedback
            subscribers = {}
            
            # Odometry subscriber
            subscribers['odom'] = rospy.Subscriber(
                f"/{namespace}/{config.odom_topic}",
                Odometry,
                lambda msg, robot=robot_name: self._odom_callback(robot, msg)
            )
            
            # Joint states subscriber
            subscribers['joint_states'] = rospy.Subscriber(
                f"/{namespace}/{config.joint_states_topic}",
                JointState,
                lambda msg, robot=robot_name: self._joint_states_callback(robot, msg)
            )
            
            # Laser scan subscriber
            if config.enable_lidar:
                subscribers['laser_scan'] = rospy.Subscriber(
                    f"/{namespace}/{config.laser_scan_topic}",
                    LaserScan,
                    lambda msg, robot=robot_name: self._laser_scan_callback(robot, msg)
                )
            
            # Camera subscriber
            if config.enable_camera:
                subscribers['camera'] = rospy.Subscriber(
                    f"/{namespace}/{config.camera_topic}",
                    Image,
                    lambda msg, robot=robot_name: self._camera_callback(robot, msg)
                )
            
            # TF subscriber
            subscribers['tf'] = rospy.Subscriber(
                "/tf",
                TFMessage,
                lambda msg, robot=robot_name: self._tf_callback(robot, msg)
            )
            
            self.robot_subscribers[robot_name] = subscribers
        
        rospy.loginfo("ROS 2 interface initialized for all robots")
    
    def _odom_callback(self, robot_name: str, msg: Odometry):
        """Callback for odometry messages."""
        with self.bridge_lock:
            if robot_name in self.robot_states:
                state = self.robot_states[robot_name]
                state['position'] = [
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z
                ]
                state['orientation'] = [
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w
                ]
                state['linear_velocity'] = [
                    msg.twist.twist.linear.x,
                    msg.twist.twist.linear.y,
                    msg.twist.twist.linear.z
                ]
                state['angular_velocity'] = [
                    msg.twist.twist.angular.x,
                    msg.twist.twist.angular.y,
                    msg.twist.twist.angular.z
                ]
                state['last_update'] = time.time()
    
    def _joint_states_callback(self, robot_name: str, msg: JointState):
        """Callback for joint state messages."""
        with self.bridge_lock:
            if robot_name in self.robot_states:
                state = self.robot_states[robot_name]
                for i, joint_name in enumerate(msg.name):
                    if i < len(msg.position):
                        state['joint_positions'][joint_name] = msg.position[i]
                state['last_update'] = time.time()
    
    def _laser_scan_callback(self, robot_name: str, msg: LaserScan):
        """Callback for laser scan messages."""
        with self.bridge_lock:
            if robot_name in self.robot_states:
                self.robot_states[robot_name]['laser_scan'] = msg
                self.robot_states[robot_name]['last_update'] = time.time()
    
    def _camera_callback(self, robot_name: str, msg: Image):
        """Callback for camera image messages."""
        with self.bridge_lock:
            if robot_name in self.robot_states:
                self.robot_states[robot_name]['camera_image'] = msg
                self.robot_states[robot_name]['last_update'] = time.time()
    
    def _tf_callback(self, robot_name: str, msg: TFMessage):
        """Callback for TF messages."""
        with self.bridge_lock:
            if robot_name in self.robot_states:
                config = self.robots[robot_name]
                for transform in msg.transforms:
                    if transform.header.frame_id == config.map_frame and transform.child_frame_id == config.base_frame:
                        # Update robot pose from TF
                        state = self.robot_states[robot_name]
                        state['position'] = [
                            transform.transform.translation.x,
                            transform.transform.translation.y,
                            transform.transform.translation.z
                        ]
                        state['orientation'] = [
                            transform.transform.rotation.x,
                            transform.transform.rotation.y,
                            transform.transform.rotation.z,
                            transform.transform.rotation.w
                        ]
                        state['last_update'] = time.time()
    
    def send_velocity_command(self, robot_name: str, linear_vel: List[float], angular_vel: List[float]):
        """Send velocity command to a robot."""
        if robot_name not in self.robot_publishers:
            rospy.logerr(f"Robot {robot_name} not found")
            return False
        
        config = self.robots[robot_name]
        
        # Apply velocity limits
        linear_vel = [
            max(-config.max_linear_velocity, min(config.max_linear_velocity, linear_vel[0])),
            max(-config.max_linear_velocity, min(config.max_linear_velocity, linear_vel[1])),
            max(-config.max_linear_velocity, min(config.max_linear_velocity, linear_vel[2]))
        ]
        angular_vel = [
            max(-config.max_angular_velocity, min(config.max_angular_velocity, angular_vel[0])),
            max(-config.max_angular_velocity, min(config.max_angular_velocity, angular_vel[1])),
            max(-config.max_angular_velocity, min(config.max_angular_velocity, angular_vel[2]))
        ]
        
        # Create Twist message
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel[0]
        twist_msg.linear.y = linear_vel[1]
        twist_msg.linear.z = linear_vel[2]
        twist_msg.angular.x = angular_vel[0]
        twist_msg.angular.y = angular_vel[1]
        twist_msg.angular.z = angular_vel[2]
        
        # Publish command
        self.robot_publishers[robot_name]['cmd_vel'].publish(twist_msg)
        
        rospy.logdebug(f"Sent velocity command to {robot_name}: linear={linear_vel}, angular={angular_vel}")
        return True
    
    def send_joint_command(self, robot_name: str, joint_positions: Dict[str, float]):
        """Send joint position command to a robot."""
        if robot_name not in self.robot_publishers:
            rospy.logerr(f"Robot {robot_name} not found")
            return False
        
        config = self.robots[robot_name]
        if not config.enable_manipulation:
            rospy.logerr(f"Robot {robot_name} does not support manipulation")
            return False
        
        # Create JointState message
        joint_msg = JointState()
        joint_msg.name = list(joint_positions.keys())
        joint_msg.position = list(joint_positions.values())
        joint_msg.velocity = [0.0] * len(joint_positions)  # Default velocity
        joint_msg.effort = [0.0] * len(joint_positions)    # Default effort
        
        # Publish command
        self.robot_publishers[robot_name]['joint_commands'].publish(joint_msg)
        
        rospy.logdebug(f"Sent joint command to {robot_name}: {joint_positions}")
        return True
    
    def send_gripper_command(self, robot_name: str, position: float):
        """Send gripper position command to a robot."""
        if robot_name not in self.robot_publishers:
            rospy.logerr(f"Robot {robot_name} not found")
            return False
        
        config = self.robots[robot_name]
        if not config.enable_manipulation:
            rospy.logerr(f"Robot {robot_name} does not support manipulation")
            return False
        
        # Create Float64 message
        gripper_msg = Float64()
        gripper_msg.data = max(0.0, min(1.0, position))  # Clamp between 0 and 1
        
        # Publish command
        self.robot_publishers[robot_name]['gripper_command'].publish(gripper_msg)
        
        rospy.logdebug(f"Sent gripper command to {robot_name}: {position}")
        return True
    
    def emergency_stop(self, robot_name: str = None):
        """Send emergency stop command to robot(s)."""
        if robot_name:
            if robot_name not in self.robot_publishers:
                rospy.logerr(f"Robot {robot_name} not found")
                return False
            
            stop_msg = Bool()
            stop_msg.data = True
            self.robot_publishers[robot_name]['emergency_stop'].publish(stop_msg)
            rospy.logwarn(f"Emergency stop sent to {robot_name}")
        else:
            # Stop all robots
            stop_msg = Bool()
            stop_msg.data = True
            for robot_name in self.robot_publishers:
                self.robot_publishers[robot_name]['emergency_stop'].publish(stop_msg)
            rospy.logwarn("Emergency stop sent to all robots")
        
        return True
    
    def get_robot_state(self, robot_name: str) -> Optional[Dict]:
        """Get current state of a robot."""
        with self.bridge_lock:
            if robot_name in self.robot_states:
                return self.robot_states[robot_name].copy()
            return None
    
    def get_robot_position(self, robot_name: str) -> Optional[List[float]]:
        """Get current position of a robot."""
        state = self.get_robot_state(robot_name)
        if state:
            return state['position']
        return None
    
    def get_robot_orientation(self, robot_name: str) -> Optional[List[float]]:
        """Get current orientation of a robot."""
        state = self.get_robot_state(robot_name)
        if state:
            return state['orientation']
        return None
    
    def get_robot_velocity(self, robot_name: str) -> Optional[Dict[str, List[float]]]:
        """Get current velocity of a robot."""
        state = self.get_robot_state(robot_name)
        if state:
            return {
                'linear': state['linear_velocity'],
                'angular': state['angular_velocity']
            }
        return None
    
    def get_joint_positions(self, robot_name: str) -> Optional[Dict[str, float]]:
        """Get current joint positions of a robot."""
        state = self.get_robot_state(robot_name)
        if state:
            return state['joint_positions'].copy()
        return None
    
    def get_laser_scan(self, robot_name: str) -> Optional[LaserScan]:
        """Get latest laser scan from a robot."""
        state = self.get_robot_state(robot_name)
        if state and state['laser_scan']:
            return state['laser_scan']
        return None
    
    def get_camera_image(self, robot_name: str) -> Optional[Image]:
        """Get latest camera image from a robot."""
        state = self.get_robot_state(robot_name)
        if state and state['camera_image']:
            return state['camera_image']
        return None
    
    def is_robot_connected(self, robot_name: str) -> bool:
        """Check if a robot is connected and receiving updates."""
        state = self.get_robot_state(robot_name)
        if state:
            return (time.time() - state['last_update']) < self.robots[robot_name].safety_timeout
        return False
    
    def get_available_robots(self) -> List[str]:
        """Get list of available robots."""
        return list(self.robots.keys())
    
    def get_robot_config(self, robot_name: str) -> Optional[RobotConfig]:
        """Get configuration of a robot."""
        return self.robots.get(robot_name)
    
    def add_robot(self, config: RobotConfig):
        """Add a new robot to the bridge."""
        with self.bridge_lock:
            self.robots[config.name] = config
            self.robot_states[config.name] = {
                'position': [0.0, 0.0, 0.0],
                'orientation': [0.0, 0.0, 0.0, 1.0],
                'linear_velocity': [0.0, 0.0, 0.0],
                'angular_velocity': [0.0, 0.0, 0.0],
                'joint_positions': {},
                'laser_scan': None,
                'camera_image': None,
                'last_update': time.time()
            }
            
            # Initialize publishers and subscribers for new robot
            self._initialize_robot_interface(config)
            
            rospy.loginfo(f"Added robot: {config.name}")
    
    def _initialize_robot_interface(self, config: RobotConfig):
        """Initialize ROS interface for a specific robot."""
        namespace = config.namespace
        
        # Publishers
        publishers = {}
        publishers['cmd_vel'] = rospy.Publisher(
            f"/{namespace}/{config.cmd_vel_topic}",
            Twist,
            queue_size=10
        )
        
        if config.enable_manipulation:
            publishers['joint_commands'] = rospy.Publisher(
                f"/{namespace}/joint_commands",
                JointState,
                queue_size=10
            )
            publishers['gripper_command'] = rospy.Publisher(
                f"/{namespace}/gripper_command",
                Float64,
                queue_size=10
            )
        
        publishers['emergency_stop'] = rospy.Publisher(
            f"/{namespace}/emergency_stop",
            Bool,
            queue_size=1
        )
        
        self.robot_publishers[config.name] = publishers
        
        # Subscribers
        subscribers = {}
        subscribers['odom'] = rospy.Subscriber(
            f"/{namespace}/{config.odom_topic}",
            Odometry,
            lambda msg, robot=config.name: self._odom_callback(robot, msg)
        )
        
        subscribers['joint_states'] = rospy.Subscriber(
            f"/{namespace}/{config.joint_states_topic}",
            JointState,
            lambda msg, robot=config.name: self._joint_states_callback(robot, msg)
        )
        
        if config.enable_lidar:
            subscribers['laser_scan'] = rospy.Subscriber(
                f"/{namespace}/{config.laser_scan_topic}",
                LaserScan,
                lambda msg, robot=config.name: self._laser_scan_callback(robot, msg)
            )
        
        if config.enable_camera:
            subscribers['camera'] = rospy.Subscriber(
                f"/{namespace}/{config.camera_topic}",
                Image,
                lambda msg, robot=config.name: self._camera_callback(robot, msg)
            )
        
        self.robot_subscribers[config.name] = subscribers
    
    def run(self):
        """Main run loop for the bridge."""
        self.is_running = True
        rospy.loginfo("Isaac Sim Bridge started")
        
        # Main loop
        rate = rospy.Rate(self.simulation_rate)
        while not rospy.is_shutdown() and self.is_running:
            # Check robot connections
            for robot_name in self.robots:
                if not self.is_robot_connected(robot_name):
                    rospy.logwarn(f"Robot {robot_name} not receiving updates")
            
            rate.sleep()
    
    def shutdown(self):
        """Shutdown the bridge."""
        self.is_running = False
        rospy.loginfo("Isaac Sim Bridge shutting down")

# Convenience functions for easy usage
def create_bridge() -> IsaacSimBridge:
    """Create and return an Isaac Sim bridge instance."""
    return IsaacSimBridge()

def send_robot_command(bridge: IsaacSimBridge, robot_name: str, action: str, **kwargs):
    """Send a command to a robot through the bridge."""
    if action == "move":
        linear_vel = kwargs.get('linear_velocity', [0.0, 0.0, 0.0])
        angular_vel = kwargs.get('angular_velocity', [0.0, 0.0, 0.0])
        return bridge.send_velocity_command(robot_name, linear_vel, angular_vel)
    elif action == "joint":
        joint_positions = kwargs.get('positions', {})
        return bridge.send_joint_command(robot_name, joint_positions)
    elif action == "gripper":
        position = kwargs.get('position', 0.0)
        return bridge.send_gripper_command(robot_name, position)
    elif action == "stop":
        return bridge.emergency_stop(robot_name)
    else:
        rospy.logerr(f"Unknown action: {action}")
        return False

if __name__ == '__main__':
    try:
        bridge = IsaacSimBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        bridge.shutdown() 