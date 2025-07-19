#!/usr/bin/env python3
"""
Interactive FernAssist Demo

This script allows you to input your own requests and see the system respond.
Perfect for live demonstrations and testing different scenarios.
"""

import time
import json
from datetime import datetime

def print_header():
    """Print a nice header for the demo."""
    print("\n" + "="*60)
    print("🎬 FERNASSIST INTERACTIVE DEMO")
    print("="*60)
    print("AAC-Powered Robot Assistant")
    print("Interactive demonstration")
    print(f"Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("="*60)

def simulate_llm_processing(text):
    """Simulate LLM processing with visual feedback."""
    print(f"\n🧠 LLM Processing: '{text}'")
    print("   🔍 Analyzing intent...")
    time.sleep(1)
    print("   📝 Extracting objects...")
    time.sleep(0.5)
    print("   🎯 Planning robot actions...")
    time.sleep(0.5)
    print("   🤖 Selecting robot...")
    time.sleep(0.3)
    
    # Simulate different responses based on input
    text_lower = text.lower()
    
    if any(word in text_lower for word in ["water", "thirsty", "drink", "bottle"]):
        return {
            "action": "fetch",
            "object": "water bottle",
            "location": "kitchen counter",
            "robot": "TurtleBot",
            "confidence": 0.95
        }
    elif any(word in text_lower for word in ["book", "read", "story"]):
        return {
            "action": "fetch", 
            "object": "book",
            "location": "bedroom shelf",
            "robot": "Carter",
            "confidence": 0.92
        }
    elif any(word in text_lower for word in ["lamp", "light", "turn on", "switch"]):
        return {
            "action": "control",
            "object": "lamp",
            "location": "bedroom",
            "robot": "Franka",
            "confidence": 0.88
        }
    elif any(word in text_lower for word in ["apple", "fruit", "food"]):
        return {
            "action": "fetch",
            "object": "apple",
            "location": "kitchen counter",
            "robot": "TurtleBot",
            "confidence": 0.90
        }
    elif any(word in text_lower for word in ["remote", "tv", "television"]):
        return {
            "action": "fetch",
            "object": "TV remote",
            "location": "living room table",
            "robot": "TurtleBot",
            "confidence": 0.85
        }
    else:
        return {
            "action": "unknown",
            "object": "unknown",
            "location": "unknown",
            "robot": "TurtleBot",
            "confidence": 0.0
        }

def simulate_object_detection():
    """Simulate object detection with YOLOv8."""
    print("\n👁️ Object Detection (YOLOv8)")
    print("   📷 Processing camera feed...")
    time.sleep(0.5)
    print("   🔍 Running YOLOv8 model...")
    time.sleep(0.5)
    print("   🎯 Detecting objects...")
    time.sleep(0.5)
    print("   ✅ Found: water bottle (confidence: 0.95)")
    print("   ✅ Found: cup (confidence: 0.87)")
    print("   ✅ Found: apple (confidence: 0.92)")
    print("   📍 Calculating 3D positions...")
    time.sleep(0.3)
    print("   📊 3D positions calculated")
    print("   🎯 Target object identified")

def simulate_robot_action(action_data):
    """Simulate robot action execution."""
    robot = action_data["robot"]
    action = action_data["action"]
    obj = action_data["object"]
    location = action_data["location"]
    
    print(f"\n🤖 Robot Action: {robot}")
    print(f"   🎯 Action: {action} {obj}")
    print(f"   📍 Location: {location}")
    
    if action == "fetch":
        print("   🚶 Moving to location...")
        time.sleep(1)
        print("   🎯 Locating object...")
        time.sleep(0.5)
        print("   🤏 Extending gripper...")
        time.sleep(0.5)
        print("   🎯 Grasping object...")
        time.sleep(0.5)
        print("   📦 Object secured")
        print("   🚶 Returning to user...")
        time.sleep(1)
        print("   ✅ Task completed!")
    elif action == "control":
        print("   🎛️ Activating device...")
        time.sleep(0.5)
        print("   ✅ Device activated!")
    else:
        print("   ⚠️ Unknown action type")

def simulate_feedback():
    """Simulate multi-modal feedback."""
    print("\n📢 Multi-Modal Feedback")
    print("   🔊 TTS: 'I have completed the task successfully'")
    print("   👁️ Visual: Green checkmark displayed")
    print("   📱 AAC: Success message sent to app")
    print("   ✅ All feedback channels active")

def run_interactive_demo():
    """Run the interactive demo."""
    print_header()
    
    print("\n🎯 Interactive Demo Mode")
    print("   Type your requests and watch the system respond!")
    print("   Examples:")
    print("   • 'I'm thirsty, bring me water'")
    print("   • 'Turn on the lamp'")
    print("   • 'Bring me my book'")
    print("   • 'Get me an apple'")
    print("   • Type 'quit' to exit")
    print()
    
    request_count = 0
    
    while True:
        request_count += 1
        print(f"\n📝 Request #{request_count}")
        user_input = input("   What would you like me to do? ").strip()
        
        if user_input.lower() in ['quit', 'exit', 'q']:
            print("\n👋 Interactive demo ended. Thanks for testing!")
            break
        
        if not user_input:
            print("   ⚠️ Please enter a request.")
            continue
        
        print(f"\n🔄 Processing: '{user_input}'")
        print("-" * 50)
        
        # Process the request
        llm_result = simulate_llm_processing(user_input)
        print(f"   📊 LLM Result: {json.dumps(llm_result, indent=6)}")
        
        simulate_object_detection()
        simulate_robot_action(llm_result)
        simulate_feedback()
        
        print(f"\n✅ Request #{request_count} completed successfully!")
        print("=" * 50)

if __name__ == "__main__":
    run_interactive_demo() 