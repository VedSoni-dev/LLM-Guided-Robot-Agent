#!/usr/bin/env python3
"""
Simulation Runner for FernAssist

This module orchestrates the complete FernAssist pipeline:
AAC Input → LLM Processing → ROS 2 Robot Control → Isaac Sim Execution
"""

import os
import sys
import time
import json
import logging
import threading
import subprocess
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass
from enum import Enum
from datetime import datetime

# ROS 2 imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, Point, Quaternion

# Isaac Sim imports
try:
    import omni
    from omni.isaac.kit import SimulationApp
    from omni.isaac.core.utils import stage, extensions
    from omni.isaac.core.world import World
    from omni.isaac.core.robots import Robot
    from omni.isaac.core.utils.nucleus import get_nucleus_server
    from omni.isaac.core.utils.stage import add_reference_to_stage
    ISAAC_SIM_AVAILABLE = True
except ImportError:
    ISAAC_SIM_AVAILABLE = False

class SimulationMode(Enum):
    """Enumeration of simulation modes."""
    KITCHEN = "kitchen"
    BEDROOM = "bedroom"
    LIVING_ROOM = "living_room"
    OFFICE = "office"
    CUSTOM = "custom"

class RobotType(Enum):
    """Enumeration of robot types."""
    TURTLEBOT = "turtlebot"
    FRANKA = "franka"
    CARTER = "carter"
    HUMANOID = "humanoid"
    CUSTOM = "custom"

@dataclass
class SimulationConfig:
    """Configuration for simulation runner."""
    # Scene Configuration
    scene_mode: SimulationMode = SimulationMode.KITCHEN
    scene_path: str = ""
    enable_physics: bool = True
    physics_dt: float = 1.0 / 60.0
    
    # Robot Configuration
    robots: List[RobotType] = None
    robot_positions: Dict[str, List[float]] = None
    enable_robot_control: bool = True
    
    # Pipeline Configuration
    enable_aac_input: bool = True
    enable_llm_processing: bool = True
    enable_ros_bridge: bool = True
    enable_object_detection: bool = True
    enable_feedback: bool = True
    
    # Logging Configuration
    enable_logging: bool = True
    log_directory: str = "demo/logs"
    enable_screenshots: bool = True
    screenshot_interval: float = 5.0
    enable_video_recording: bool = False
    
    # Performance Configuration
    max_simulation_time: float = 300.0  # 5 minutes
    enable_performance_monitoring: bool = True
    
    # Demo Configuration
    demo_mode: bool = False
    demo_scenarios: List[str] = None
    auto_advance: bool = False
    demo_timeout: float = 60.0

class IsaacSimManager:
    """Manages Isaac Sim simulation environment."""
    
    def __init__(self, config: SimulationConfig):
        self.config = config
        self.simulation_app = None
        self.world = None
        self.robots = {}
        self.is_running = False
        
        if not ISAAC_SIM_AVAILABLE:
            raise RuntimeError("Isaac Sim not available - install Isaac Sim")
    
    def initialize_simulation(self):
        """Initialize Isaac Sim simulation."""
        try:
            # Initialize Isaac Sim
            self.simulation_app = SimulationApp({
                "renderer": "RayTracedLighting",
                "headless": False,
                "physics_dt": self.config.physics_dt,
                "physics_substeps": 1,
                "enable_physics": self.config.enable_physics
            })
            
            # Enable required extensions
            extensions.enable_extension("omni.isaac.core")
            extensions.enable_extension("omni.isaac.ui")
            extensions.enable_extension("omni.isaac.ros_bridge")
            
            # Create world
            self.world = World(physics_dt=self.config.physics_dt, rendering_dt=1.0/60.0)
            
            # Load scene
            self._load_scene()
            
            # Spawn robots
            self._spawn_robots()
            
            # Initialize physics
            if self.config.enable_physics:
                self.world.reset()
            
            self.is_running = True
            logging.info("Isaac Sim simulation initialized successfully")
            
        except Exception as e:
            logging.error(f"Failed to initialize Isaac Sim: {e}")
            raise
    
    def _load_scene(self):
        """Load simulation scene."""
        scene_paths = {
            SimulationMode.KITCHEN: "/Isaac/Environments/Kitchen/kitchen.usd",
            SimulationMode.BEDROOM: "/Isaac/Environments/Bedroom/bedroom.usd",
            SimulationMode.LIVING_ROOM: "/Isaac/Environments/LivingRoom/living_room.usd",
            SimulationMode.OFFICE: "/Isaac/Environments/Office/office.usd"
        }
        
        if self.config.scene_path:
            scene_path = self.config.scene_path
        else:
            scene_path = scene_paths.get(self.config.scene_mode)
        
        if not scene_path:
            logging.warning("No scene path specified, using default")
            scene_path = "/Isaac/Environments/Kitchen/kitchen.usd"
        
        try:
            # Load scene
            add_reference_to_stage(usd_path=scene_path, prim_path="/World")
            logging.info(f"Loaded scene: {scene_path}")
            
        except Exception as e:
            logging.error(f"Failed to load scene {scene_path}: {e}")
            raise
    
    def _spawn_robots(self):
        """Spawn robots in the simulation."""
        if not self.config.robots:
            self.config.robots = [RobotType.TURTLEBOT]
        
        robot_configs = {
            RobotType.TURTLEBOT: {
                "usd_path": "/Isaac/Robots/Turtlebot/turtlebot.usd",
                "default_position": [0.0, 0.0, 0.0],
                "default_orientation": [0.0, 0.0, 0.0, 1.0]
            },
            RobotType.FRANKA: {
                "usd_path": "/Isaac/Robots/Franka/franka.usd",
                "default_position": [1.0, 0.0, 0.0],
                "default_orientation": [0.0, 0.0, 0.0, 1.0]
            },
            RobotType.CARTER: {
                "usd_path": "/Isaac/Robots/Carter/carter.usd",
                "default_position": [0.0, 1.0, 0.0],
                "default_orientation": [0.0, 0.0, 0.0, 1.0]
            },
            RobotType.HUMANOID: {
                "usd_path": "/Isaac/Robots/Humanoid/humanoid.usd",
                "default_position": [0.0, -1.0, 0.0],
                "default_orientation": [0.0, 0.0, 0.0, 1.0]
            }
        }
        
        for robot_type in self.config.robots:
            try:
                config = robot_configs.get(robot_type)
                if not config:
                    logging.warning(f"Unknown robot type: {robot_type}")
                    continue
                
                # Get position and orientation
                position = self.config.robot_positions.get(
                    robot_type.value, 
                    config["default_position"]
                )
                orientation = config["default_orientation"]
                
                # Spawn robot
                robot = Robot(
                    prim_path=f"/World/{robot_type.value}",
                    name=robot_type.value,
                    usd_path=config["usd_path"],
                    position=position,
                    orientation=orientation
                )
                
                self.robots[robot_type.value] = robot
                logging.info(f"Spawned {robot_type.value} at position {position}")
                
            except Exception as e:
                logging.error(f"Failed to spawn {robot_type.value}: {e}")
    
    def run_simulation(self):
        """Run the simulation loop."""
        if not self.is_running:
            logging.error("Simulation not initialized")
            return
        
        try:
            while self.is_running:
                # Step simulation
                self.world.step(render=True)
                
                # Check for shutdown
                if self.simulation_app.is_running() == False:
                    break
                
        except Exception as e:
            logging.error(f"Simulation error: {e}")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up simulation resources."""
        if self.simulation_app:
            self.simulation_app.close()
        self.is_running = False
        logging.info("Isaac Sim simulation cleaned up")

class PipelineManager:
    """Manages the AAC→LLM→ROS 2 pipeline."""
    
    def __init__(self, config: SimulationConfig):
        self.config = config
        self.llm_interpreter = None
        self.ros_bridge = None
        self.object_detector = None
        self.feedback_module = None
        self.is_running = False
        
        # Pipeline state
        self.current_action = None
        self.action_history = []
        self.pipeline_stats = {
            "total_actions": 0,
            "successful_actions": 0,
            "failed_actions": 0,
            "average_processing_time": 0.0
        }
    
    def initialize_pipeline(self):
        """Initialize the complete pipeline."""
        try:
            # Initialize LLM interpreter
            if self.config.enable_llm_processing:
                from scripts.llm_interpreter import LLMInterpreter
                self.llm_interpreter = LLMInterpreter()
                logging.info("LLM interpreter initialized")
            
            # Initialize ROS bridge
            if self.config.enable_ros_bridge:
                from scripts.ros_bridge_utils import IsaacSimBridge
                self.ros_bridge = IsaacSimBridge()
                logging.info("ROS bridge initialized")
            
            # Initialize object detector
            if self.config.enable_object_detection:
                from scripts.detect_objects import ObjectDetector
                self.object_detector = ObjectDetector()
                logging.info("Object detector initialized")
            
            # Initialize feedback module
            if self.config.enable_feedback:
                from scripts.feedback_module import FeedbackModule
                self.feedback_module = FeedbackModule()
                logging.info("Feedback module initialized")
            
            self.is_running = True
            logging.info("Pipeline initialized successfully")
            
        except Exception as e:
            logging.error(f"Failed to initialize pipeline: {e}")
            raise
    
    def process_aac_input(self, aac_text: str) -> Dict:
        """Process AAC input through the complete pipeline."""
        if not self.is_running:
            raise RuntimeError("Pipeline not initialized")
        
        start_time = time.time()
        
        try:
            # Step 1: LLM Processing
            if self.llm_interpreter:
                llm_result = self.llm_interpreter.process_intent(aac_text)
                logging.info(f"LLM processed: {aac_text} -> {llm_result}")
            else:
                llm_result = {"action": "unknown", "object": "unknown"}
            
            # Step 2: Object Detection (if needed)
            detected_objects = []
            if self.object_detector and "object" in llm_result:
                detected_objects = self.object_detector.get_recent_detections()
                logging.info(f"Detected objects: {len(detected_objects)}")
            
            # Step 3: ROS Bridge Execution
            execution_result = None
            if self.ros_bridge:
                execution_result = self._execute_robot_action(llm_result, detected_objects)
                logging.info(f"Robot execution: {execution_result}")
            
            # Step 4: Feedback
            if self.feedback_module:
                self._send_feedback(llm_result, execution_result)
            
            # Update statistics
            processing_time = time.time() - start_time
            self._update_stats(True, processing_time)
            
            # Log action
            action_log = {
                "timestamp": datetime.now().isoformat(),
                "aac_input": aac_text,
                "llm_result": llm_result,
                "detected_objects": len(detected_objects),
                "execution_result": execution_result,
                "processing_time": processing_time
            }
            self.action_history.append(action_log)
            
            return {
                "success": True,
                "llm_result": llm_result,
                "execution_result": execution_result,
                "processing_time": processing_time
            }
            
        except Exception as e:
            processing_time = time.time() - start_time
            self._update_stats(False, processing_time)
            
            logging.error(f"Pipeline processing error: {e}")
            
            # Send error feedback
            if self.feedback_module:
                self.feedback_module.send_error(
                    "PIPELINE_ERROR",
                    f"Failed to process: {aac_text}",
                    context={"error": str(e)}
                )
            
            return {
                "success": False,
                "error": str(e),
                "processing_time": processing_time
            }
    
    def _execute_robot_action(self, llm_result: Dict, detected_objects: List) -> Dict:
        """Execute robot action through ROS bridge."""
        if not self.ros_bridge:
            return {"status": "no_ros_bridge"}
        
        try:
            action = llm_result.get("action", "unknown")
            object_name = llm_result.get("object", "unknown")
            
            # Map LLM actions to robot commands
            if action == "fetch":
                return self.ros_bridge.fetch_object(object_name)
            elif action == "move":
                return self.ros_bridge.move_to_location(object_name)
            elif action == "pick":
                return self.ros_bridge.pick_object(object_name)
            elif action == "place":
                return self.ros_bridge.place_object(object_name)
            elif action == "open":
                return self.ros_bridge.open_door(object_name)
            elif action == "close":
                return self.ros_bridge.close_door(object_name)
            else:
                return {"status": "unknown_action", "action": action}
                
        except Exception as e:
            logging.error(f"Robot action execution error: {e}")
            return {"status": "error", "error": str(e)}
    
    def _send_feedback(self, llm_result: Dict, execution_result: Dict):
        """Send appropriate feedback based on results."""
        if not self.feedback_module:
            return
        
        if execution_result and execution_result.get("status") == "success":
            self.feedback_module.send_feedback(
                message_type="success",
                title="Action Completed",
                content=f"Successfully completed {llm_result.get('action', 'action')}",
                tts_text=f"I have completed the {llm_result.get('action', 'action')}"
            )
        else:
            self.feedback_module.send_error(
                "EXECUTION_FAILED",
                f"Failed to execute {llm_result.get('action', 'action')}",
                context={"llm_result": llm_result, "execution_result": execution_result}
            )
    
    def _update_stats(self, success: bool, processing_time: float):
        """Update pipeline statistics."""
        self.pipeline_stats["total_actions"] += 1
        
        if success:
            self.pipeline_stats["successful_actions"] += 1
        else:
            self.pipeline_stats["failed_actions"] += 1
        
        # Update average processing time
        total_time = self.pipeline_stats["average_processing_time"] * (self.pipeline_stats["total_actions"] - 1)
        total_time += processing_time
        self.pipeline_stats["average_processing_time"] = total_time / self.pipeline_stats["total_actions"]
    
    def get_pipeline_stats(self) -> Dict:
        """Get pipeline statistics."""
        return self.pipeline_stats.copy()
    
    def get_action_history(self) -> List[Dict]:
        """Get action history."""
        return self.action_history.copy()
    
    def cleanup(self):
        """Clean up pipeline resources."""
        self.is_running = False
        logging.info("Pipeline cleaned up")

class DemoLogger:
    """Handles logging and screenshots for demonstrations."""
    
    def __init__(self, config: SimulationConfig):
        self.config = config
        self.log_directory = Path(config.log_directory)
        self.screenshot_directory = self.log_directory / "screenshots"
        self.video_directory = self.log_directory / "videos"
        
        # Create directories
        self.log_directory.mkdir(parents=True, exist_ok=True)
        self.screenshot_directory.mkdir(parents=True, exist_ok=True)
        self.video_directory.mkdir(parents=True, exist_ok=True)
        
        # Setup logging
        self._setup_logging()
        
        # Screenshot thread
        self.screenshot_thread = None
        self.screenshot_running = False
        
        # Demo scenarios
        self.demo_scenarios = self.config.demo_scenarios or [
            "I'm thirsty",
            "Bring me a cup",
            "Open the door",
            "Move to the kitchen",
            "Pick up the book"
        ]
    
    def _setup_logging(self):
        """Setup logging configuration."""
        log_file = self.log_directory / f"simulation_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
        
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler(log_file),
                logging.StreamHandler(sys.stdout)
            ]
        )
        
        logging.info(f"Demo logging initialized: {log_file}")
    
    def start_screenshot_capture(self):
        """Start automatic screenshot capture."""
        if not self.config.enable_screenshots:
            return
        
        self.screenshot_running = True
        self.screenshot_thread = threading.Thread(target=self._screenshot_loop)
        self.screenshot_thread.daemon = True
        self.screenshot_thread.start()
        
        logging.info("Screenshot capture started")
    
    def stop_screenshot_capture(self):
        """Stop automatic screenshot capture."""
        self.screenshot_running = False
        if self.screenshot_thread:
            self.screenshot_thread.join()
        
        logging.info("Screenshot capture stopped")
    
    def _screenshot_loop(self):
        """Screenshot capture loop."""
        while self.screenshot_running:
            try:
                self.capture_screenshot()
                time.sleep(self.config.screenshot_interval)
            except Exception as e:
                logging.error(f"Screenshot error: {e}")
    
    def capture_screenshot(self):
        """Capture a screenshot."""
        try:
            if ISAAC_SIM_AVAILABLE:
                # Capture Isaac Sim screenshot
                import omni.kit.commands
                screenshot_path = self.screenshot_directory / f"screenshot_{int(time.time())}.png"
                
                omni.kit.commands.execute(
                    "Screenshot",
                    output_path=str(screenshot_path),
                    width=1920,
                    height=1080
                )
                
                logging.debug(f"Screenshot captured: {screenshot_path}")
            
        except Exception as e:
            logging.error(f"Failed to capture screenshot: {e}")
    
    def log_action(self, action_data: Dict):
        """Log action data."""
        try:
            log_file = self.log_directory / "actions.json"
            
            # Load existing actions
            actions = []
            if log_file.exists():
                with open(log_file, 'r') as f:
                    actions = json.load(f)
            
            # Add new action
            actions.append(action_data)
            
            # Save updated actions
            with open(log_file, 'w') as f:
                json.dump(actions, f, indent=2)
            
            logging.info(f"Action logged: {action_data.get('aac_input', 'unknown')}")
            
        except Exception as e:
            logging.error(f"Failed to log action: {e}")
    
    def generate_demo_report(self) -> str:
        """Generate demo report."""
        try:
            report_file = self.log_directory / f"demo_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.md"
            
            with open(report_file, 'w') as f:
                f.write("# FernAssist Demo Report\n\n")
                f.write(f"**Date:** {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")
                f.write(f"**Scene:** {self.config.scene_mode.value}\n\n")
                f.write(f"**Robots:** {[r.value for r in self.config.robots]}\n\n")
                
                # Pipeline statistics
                f.write("## Pipeline Statistics\n\n")
                f.write("- Total Actions: {}\n".format(self.pipeline_stats.get("total_actions", 0)))
                f.write("- Successful Actions: {}\n".format(self.pipeline_stats.get("successful_actions", 0)))
                f.write("- Failed Actions: {}\n".format(self.pipeline_stats.get("failed_actions", 0)))
                f.write("- Average Processing Time: {:.2f}s\n\n".format(
                    self.pipeline_stats.get("average_processing_time", 0.0)
                ))
                
                # Action history
                f.write("## Action History\n\n")
                for action in self.action_history[-10:]:  # Last 10 actions
                    f.write(f"- **{action['timestamp']}**: {action['aac_input']}\n")
                    f.write(f"  - LLM Result: {action['llm_result']}\n")
                    f.write(f"  - Processing Time: {action['processing_time']:.2f}s\n\n")
                
                # Screenshots
                screenshots = list(self.screenshot_directory.glob("*.png"))
                if screenshots:
                    f.write("## Screenshots\n\n")
                    for screenshot in screenshots[-5:]:  # Last 5 screenshots
                        f.write(f"- {screenshot.name}\n")
            
            logging.info(f"Demo report generated: {report_file}")
            return str(report_file)
            
        except Exception as e:
            logging.error(f"Failed to generate demo report: {e}")
            return ""

class SimulationRunner:
    """Main simulation runner class."""
    
    def __init__(self, config: SimulationConfig):
        self.config = config
        self.isaac_sim = None
        self.pipeline = None
        self.logger = None
        self.is_running = False
        
        # Demo state
        self.current_scenario = 0
        self.demo_start_time = None
    
    def initialize(self):
        """Initialize the complete simulation system."""
        try:
            logging.info("Initializing FernAssist Simulation Runner")
            
            # Initialize logger
            self.logger = DemoLogger(self.config)
            
            # Initialize Isaac Sim
            if ISAAC_SIM_AVAILABLE:
                self.isaac_sim = IsaacSimManager(self.config)
                self.isaac_sim.initialize_simulation()
            
            # Initialize pipeline
            self.pipeline = PipelineManager(self.config)
            self.pipeline.initialize_pipeline()
            
            # Start screenshot capture
            self.logger.start_screenshot_capture()
            
            self.is_running = True
            logging.info("Simulation runner initialized successfully")
            
        except Exception as e:
            logging.error(f"Failed to initialize simulation runner: {e}")
            raise
    
    def run_demo(self):
        """Run demonstration mode."""
        if not self.is_running:
            raise RuntimeError("Simulation runner not initialized")
        
        self.demo_start_time = time.time()
        logging.info("Starting demo mode")
        
        try:
            # Run demo scenarios
            for i, scenario in enumerate(self.logger.demo_scenarios):
                if not self.is_running:
                    break
                
                self.current_scenario = i
                logging.info(f"Demo scenario {i+1}/{len(self.logger.demo_scenarios)}: {scenario}")
                
                # Process scenario
                result = self.pipeline.process_aac_input(scenario)
                
                # Log action
                self.logger.log_action({
                    "scenario": i + 1,
                    "aac_input": scenario,
                    "result": result,
                    "timestamp": datetime.now().isoformat()
                })
                
                # Wait between scenarios
                if i < len(self.logger.demo_scenarios) - 1:
                    time.sleep(3.0)
            
            # Generate demo report
            report_file = self.logger.generate_demo_report()
            logging.info(f"Demo completed. Report: {report_file}")
            
        except Exception as e:
            logging.error(f"Demo error: {e}")
        finally:
            self.cleanup()
    
    def run_interactive(self):
        """Run interactive mode."""
        if not self.is_running:
            raise RuntimeError("Simulation runner not initialized")
        
        logging.info("Starting interactive mode")
        
        try:
            while self.is_running:
                # Get user input
                user_input = input("Enter AAC input (or 'quit' to exit): ").strip()
                
                if user_input.lower() == 'quit':
                    break
                
                if not user_input:
                    continue
                
                # Process input
                result = self.pipeline.process_aac_input(user_input)
                
                # Log action
                self.logger.log_action({
                    "aac_input": user_input,
                    "result": result,
                    "timestamp": datetime.now().isoformat()
                })
                
                # Display result
                if result["success"]:
                    print(f"✅ Success: {result['llm_result']}")
                else:
                    print(f"❌ Error: {result['error']}")
                
        except KeyboardInterrupt:
            logging.info("Interactive mode interrupted")
        finally:
            self.cleanup()
    
    def run_simulation_only(self):
        """Run Isaac Sim simulation only."""
        if not self.is_running or not self.isaac_sim:
            raise RuntimeError("Simulation not initialized")
        
        logging.info("Starting Isaac Sim simulation")
        
        try:
            self.isaac_sim.run_simulation()
        except Exception as e:
            logging.error(f"Simulation error: {e}")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up all resources."""
        logging.info("Cleaning up simulation runner")
        
        # Stop screenshot capture
        if self.logger:
            self.logger.stop_screenshot_capture()
        
        # Cleanup pipeline
        if self.pipeline:
            self.pipeline.cleanup()
        
        # Cleanup Isaac Sim
        if self.isaac_sim:
            self.isaac_sim.cleanup()
        
        self.is_running = False
        logging.info("Simulation runner cleaned up")

def create_simulation_runner(config: SimulationConfig) -> SimulationRunner:
    """Create and return a simulation runner instance."""
    return SimulationRunner(config)

def run_kitchen_demo():
    """Run kitchen scene demonstration."""
    config = SimulationConfig(
        scene_mode=SimulationMode.KITCHEN,
        robots=[RobotType.TURTLEBOT, RobotType.FRANKA],
        demo_mode=True,
        demo_scenarios=[
            "I'm thirsty, bring me a glass of water",
            "Open the refrigerator",
            "Pick up the apple from the counter",
            "Move to the dining table",
            "Close the cabinet door"
        ]
    )
    
    runner = SimulationRunner(config)
    runner.initialize()
    runner.run_demo()

def run_bedroom_demo():
    """Run bedroom scene demonstration."""
    config = SimulationConfig(
        scene_mode=SimulationMode.BEDROOM,
        robots=[RobotType.CARTER],
        demo_mode=True,
        demo_scenarios=[
            "Turn on the bedside lamp",
            "Bring me my book from the shelf",
            "Open the window",
            "Pick up my phone from the nightstand",
            "Close the bedroom door"
        ]
    )
    
    runner = SimulationRunner(config)
    runner.initialize()
    runner.run_demo()

if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(description="FernAssist Simulation Runner")
    parser.add_argument("--mode", choices=["demo", "interactive", "simulation"], 
                       default="demo", help="Run mode")
    parser.add_argument("--scene", choices=["kitchen", "bedroom", "living_room", "office"],
                       default="kitchen", help="Simulation scene")
    parser.add_argument("--robots", nargs="+", 
                       choices=["turtlebot", "franka", "carter", "humanoid"],
                       default=["turtlebot"], help="Robots to spawn")
    parser.add_argument("--log-dir", default="demo/logs", help="Log directory")
    parser.add_argument("--screenshots", action="store_true", help="Enable screenshots")
    parser.add_argument("--video", action="store_true", help="Enable video recording")
    
    args = parser.parse_args()
    
    # Create configuration
    config = SimulationConfig(
        scene_mode=SimulationMode(args.scene),
        robots=[RobotType(robot) for robot in args.robots],
        log_directory=args.log_dir,
        enable_screenshots=args.screenshots,
        enable_video_recording=args.video
    )
    
    # Create and run simulation
    runner = SimulationRunner(config)
    
    try:
        runner.initialize()
        
        if args.mode == "demo":
            runner.run_demo()
        elif args.mode == "interactive":
            runner.run_interactive()
        elif args.mode == "simulation":
            runner.run_simulation_only()
            
    except Exception as e:
        logging.error(f"Simulation runner error: {e}")
    finally:
        runner.cleanup() 