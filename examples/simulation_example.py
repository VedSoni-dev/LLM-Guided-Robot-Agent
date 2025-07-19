#!/usr/bin/env python3
"""
Simulation Runner Example for FernAssist

This example demonstrates how to use the simulation runner with different
scenes, robots, and demo scenarios.
"""

import time
import json
from scripts.sim_runner import SimulationRunner, SimulationConfig, SimulationMode, RobotType

def run_kitchen_demo():
    """Run kitchen scene demonstration."""
    print("🍳 Running Kitchen Demo")
    print("=" * 50)
    
    # Kitchen demo configuration
    config = SimulationConfig(
        scene_mode=SimulationMode.KITCHEN,
        robots=[RobotType.TURTLEBOT, RobotType.FRANKA],
        demo_mode=True,
        demo_scenarios=[
            "I'm thirsty, bring me a glass of water",
            "Open the refrigerator",
            "Pick up the apple from the counter",
            "Move to the dining table",
            "Close the cabinet door",
            "Turn on the stove",
            "Put the cup in the sink"
        ],
        enable_logging=True,
        log_directory="demo/logs/kitchen_demo",
        enable_screenshots=True,
        screenshot_interval=3.0
    )
    
    # Create and run simulation
    runner = SimulationRunner(config)
    
    try:
        print("🚀 Initializing kitchen demo...")
        runner.initialize()
        
        print("📋 Running kitchen scenarios...")
        runner.run_demo()
        
        print("✅ Kitchen demo completed successfully!")
        
    except Exception as e:
        print(f"❌ Kitchen demo error: {e}")
    finally:
        runner.cleanup()

def run_bedroom_demo():
    """Run bedroom scene demonstration."""
    print("🛏️ Running Bedroom Demo")
    print("=" * 50)
    
    # Bedroom demo configuration
    config = SimulationConfig(
        scene_mode=SimulationMode.BEDROOM,
        robots=[RobotType.CARTER],
        demo_mode=True,
        demo_scenarios=[
            "Turn on the bedside lamp",
            "Bring me my book from the shelf",
            "Open the window",
            "Pick up my phone from the nightstand",
            "Close the bedroom door",
            "Adjust the blinds",
            "Bring me my glasses"
        ],
        enable_logging=True,
        log_directory="demo/logs/bedroom_demo",
        enable_screenshots=True,
        screenshot_interval=3.0
    )
    
    # Create and run simulation
    runner = SimulationRunner(config)
    
    try:
        print("🚀 Initializing bedroom demo...")
        runner.initialize()
        
        print("📋 Running bedroom scenarios...")
        runner.run_demo()
        
        print("✅ Bedroom demo completed successfully!")
        
    except Exception as e:
        print(f"❌ Bedroom demo error: {e}")
    finally:
        runner.cleanup()

def run_living_room_demo():
    """Run living room scene demonstration."""
    print("🛋️ Running Living Room Demo")
    print("=" * 50)
    
    # Living room demo configuration
    config = SimulationConfig(
        scene_mode=SimulationMode.LIVING_ROOM,
        robots=[RobotType.TURTLEBOT, RobotType.HUMANOID],
        demo_mode=True,
        demo_scenarios=[
            "Turn on the TV",
            "Bring me the remote control",
            "Pick up my book from the coffee table",
            "Turn on the lamp",
            "Bring me a drink",
            "Close the curtains",
            "Adjust the thermostat"
        ],
        enable_logging=True,
        log_directory="demo/logs/living_room_demo",
        enable_screenshots=True,
        screenshot_interval=3.0
    )
    
    # Create and run simulation
    runner = SimulationRunner(config)
    
    try:
        print("🚀 Initializing living room demo...")
        runner.initialize()
        
        print("📋 Running living room scenarios...")
        runner.run_demo()
        
        print("✅ Living room demo completed successfully!")
        
    except Exception as e:
        print(f"❌ Living room demo error: {e}")
    finally:
        runner.cleanup()

def run_office_demo():
    """Run office scene demonstration."""
    print("💼 Running Office Demo")
    print("=" * 50)
    
    # Office demo configuration
    config = SimulationConfig(
        scene_mode=SimulationMode.OFFICE,
        robots=[RobotType.FRANKA, RobotType.CARTER],
        demo_mode=True,
        demo_scenarios=[
            "Bring me a pen from the desk",
            "Print the document",
            "Turn on the computer",
            "Bring me a coffee",
            "Organize the papers",
            "Adjust the chair",
            "Turn on the desk lamp"
        ],
        enable_logging=True,
        log_directory="demo/logs/office_demo",
        enable_screenshots=True,
        screenshot_interval=3.0
    )
    
    # Create and run simulation
    runner = SimulationRunner(config)
    
    try:
        print("🚀 Initializing office demo...")
        runner.initialize()
        
        print("📋 Running office scenarios...")
        runner.run_demo()
        
        print("✅ Office demo completed successfully!")
        
    except Exception as e:
        print(f"❌ Office demo error: {e}")
    finally:
        runner.cleanup()

def run_interactive_mode():
    """Run interactive mode for user input."""
    print("🎮 Running Interactive Mode")
    print("=" * 50)
    
    # Interactive mode configuration
    config = SimulationConfig(
        scene_mode=SimulationMode.KITCHEN,
        robots=[RobotType.TURTLEBOT, RobotType.FRANKA],
        demo_mode=False,
        enable_logging=True,
        log_directory="demo/logs/interactive_demo",
        enable_screenshots=True,
        screenshot_interval=5.0
    )
    
    # Create and run simulation
    runner = SimulationRunner(config)
    
    try:
        print("🚀 Initializing interactive mode...")
        runner.initialize()
        
        print("📝 Interactive mode ready!")
        print("   Type your AAC input and press Enter")
        print("   Type 'quit' to exit")
        print()
        
        runner.run_interactive()
        
        print("✅ Interactive mode completed!")
        
    except Exception as e:
        print(f"❌ Interactive mode error: {e}")
    finally:
        runner.cleanup()

def run_simulation_only():
    """Run Isaac Sim simulation only (without pipeline)."""
    print("🎯 Running Simulation Only")
    print("=" * 50)
    
    # Simulation-only configuration
    config = SimulationConfig(
        scene_mode=SimulationMode.KITCHEN,
        robots=[RobotType.TURTLEBOT],
        enable_aac_input=False,
        enable_llm_processing=False,
        enable_ros_bridge=False,
        enable_object_detection=False,
        enable_feedback=False,
        enable_logging=True,
        log_directory="demo/logs/simulation_only",
        enable_screenshots=True,
        screenshot_interval=2.0
    )
    
    # Create and run simulation
    runner = SimulationRunner(config)
    
    try:
        print("🚀 Initializing Isaac Sim only...")
        runner.initialize()
        
        print("🎮 Running Isaac Sim simulation...")
        print("   Press Ctrl+C to stop")
        
        runner.run_simulation_only()
        
        print("✅ Simulation completed!")
        
    except Exception as e:
        print(f"❌ Simulation error: {e}")
    finally:
        runner.cleanup()

def run_custom_scenario():
    """Run a custom scenario with specific configuration."""
    print("🎨 Running Custom Scenario")
    print("=" * 50)
    
    # Custom scenario configuration
    config = SimulationConfig(
        scene_mode=SimulationMode.KITCHEN,
        scene_path="/path/to/custom/scene.usd",  # Custom scene path
        robots=[RobotType.CARTER, RobotType.FRANKA],
        robot_positions={
            "carter": [0.0, 0.0, 0.0],
            "franka": [2.0, 0.0, 0.0]
        },
        demo_mode=True,
        demo_scenarios=[
            "Custom task 1: Find the red object",
            "Custom task 2: Move to the corner",
            "Custom task 3: Pick up the blue item",
            "Custom task 4: Place it on the table"
        ],
        enable_logging=True,
        log_directory="demo/logs/custom_scenario",
        enable_screenshots=True,
        screenshot_interval=2.0,
        enable_video_recording=True,
        max_simulation_time=600.0  # 10 minutes
    )
    
    # Create and run simulation
    runner = SimulationRunner(config)
    
    try:
        print("🚀 Initializing custom scenario...")
        runner.initialize()
        
        print("📋 Running custom scenarios...")
        runner.run_demo()
        
        print("✅ Custom scenario completed successfully!")
        
    except Exception as e:
        print(f"❌ Custom scenario error: {e}")
    finally:
        runner.cleanup()

def run_performance_test():
    """Run performance testing with different configurations."""
    print("⚡ Running Performance Test")
    print("=" * 50)
    
    # Performance test configurations
    test_configs = [
        {
            "name": "Single Robot Test",
            "config": SimulationConfig(
                scene_mode=SimulationMode.KITCHEN,
                robots=[RobotType.TURTLEBOT],
                enable_performance_monitoring=True,
                max_simulation_time=60.0
            )
        },
        {
            "name": "Multi-Robot Test",
            "config": SimulationConfig(
                scene_mode=SimulationMode.KITCHEN,
                robots=[RobotType.TURTLEBOT, RobotType.FRANKA, RobotType.CARTER],
                enable_performance_monitoring=True,
                max_simulation_time=60.0
            )
        },
        {
            "name": "Full Pipeline Test",
            "config": SimulationConfig(
                scene_mode=SimulationMode.BEDROOM,
                robots=[RobotType.CARTER],
                enable_aac_input=True,
                enable_llm_processing=True,
                enable_ros_bridge=True,
                enable_object_detection=True,
                enable_feedback=True,
                enable_performance_monitoring=True,
                max_simulation_time=120.0
            )
        }
    ]
    
    for test in test_configs:
        print(f"\n🧪 Running {test['name']}...")
        
        runner = SimulationRunner(test['config'])
        
        try:
            runner.initialize()
            
            # Run a simple demo
            test['config'].demo_scenarios = ["Test performance scenario"]
            runner.run_demo()
            
            print(f"✅ {test['name']} completed!")
            
        except Exception as e:
            print(f"❌ {test['name']} error: {e}")
        finally:
            runner.cleanup()

def show_demo_menu():
    """Show demo selection menu."""
    print("\n🎬 FernAssist Simulation Demo Menu")
    print("=" * 50)
    print("1. Kitchen Demo (TurtleBot + Franka)")
    print("2. Bedroom Demo (Carter)")
    print("3. Living Room Demo (TurtleBot + Humanoid)")
    print("4. Office Demo (Franka + Carter)")
    print("5. Interactive Mode")
    print("6. Simulation Only")
    print("7. Custom Scenario")
    print("8. Performance Test")
    print("9. Exit")
    print()

def main():
    """Main example function."""
    print("🚀 FernAssist Simulation Runner Examples")
    print("=" * 60)
    
    while True:
        show_demo_menu()
        
        try:
            choice = input("Select a demo (1-9): ").strip()
            
            if choice == "1":
                run_kitchen_demo()
            elif choice == "2":
                run_bedroom_demo()
            elif choice == "3":
                run_living_room_demo()
            elif choice == "4":
                run_office_demo()
            elif choice == "5":
                run_interactive_mode()
            elif choice == "6":
                run_simulation_only()
            elif choice == "7":
                run_custom_scenario()
            elif choice == "8":
                run_performance_test()
            elif choice == "9":
                print("👋 Goodbye!")
                break
            else:
                print("❌ Invalid choice. Please select 1-9.")
                
        except KeyboardInterrupt:
            print("\n👋 Demo interrupted. Goodbye!")
            break
        except Exception as e:
            print(f"❌ Error: {e}")
        
        # Wait between demos
        if choice in ["1", "2", "3", "4", "7", "8"]:
            input("\nPress Enter to continue...")

if __name__ == "__main__":
    main() 