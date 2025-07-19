#!/usr/bin/env python3
"""
Feedback Module Example for FernAssist

This example demonstrates how to use the multi-modal feedback system
with text-to-speech, visual feedback, and error handling.
"""

import time
import json
from scripts.feedback_module import FeedbackModule, FeedbackType, TTSProvider

def main():
    """Main example function."""
    print("📢 FernAssist Feedback Module Example")
    print("=" * 50)
    
    # Create feedback module
    feedback = FeedbackModule()
    
    # Wait for initialization
    time.sleep(2)
    
    print("✅ Feedback module initialized")
    print(f"   TTS Provider: {feedback.config.tts_provider.value}")
    print(f"   TTS Enabled: {feedback.config.enable_tts}")
    print(f"   Visual Feedback: {feedback.config.enable_visual_feedback}")
    
    # Example 1: Basic TTS feedback
    print("\n📋 Example 1: Basic TTS Feedback")
    print("Speaking welcome message...")
    
    feedback.send_tts("Hello, I am FernAssist. I am ready to help you.")
    time.sleep(3)
    
    # Example 2: Success feedback
    print("\n📋 Example 2: Success Feedback")
    print("Sending success feedback...")
    
    feedback.send_feedback(
        message_type=FeedbackType.SUCCESS,
        title="Action Completed",
        content="Successfully picked up the water bottle",
        tts_text="I have picked up the water bottle",
        visual_icon="check",
        duration=3.0
    )
    time.sleep(4)
    
    # Example 3: Progress feedback
    print("\n📋 Example 3: Progress Feedback")
    print("Sending progress updates...")
    
    progress_messages = [
        "Moving to the kitchen",
        "Approaching the counter",
        "Locating the water bottle",
        "Extending my gripper",
        "Grasping the object"
    ]
    
    for i, message in enumerate(progress_messages, 1):
        feedback.send_feedback(
            message_type=FeedbackType.PROGRESS,
            title=f"Step {i}/5",
            content=message,
            tts_text=message,
            visual_icon="progress",
            duration=2.0
        )
        time.sleep(2.5)
    
    # Example 4: Error handling
    print("\n📋 Example 4: Error Handling")
    print("Simulating error scenarios...")
    
    # Object not found error
    feedback.send_error(
        error_code="OBJECT_NOT_FOUND",
        error_message="I couldn't find the water bottle in the expected location",
        context={"object_name": "water bottle", "expected_location": "kitchen counter"}
    )
    time.sleep(4)
    
    # Path blocked error
    feedback.send_error(
        error_code="PATH_BLOCKED",
        error_message="The path to the kitchen is blocked by furniture",
        context={"destination": "kitchen", "obstacle": "furniture"}
    )
    time.sleep(4)
    
    # Example 5: Warning feedback
    print("\n📋 Example 5: Warning Feedback")
    print("Sending warning messages...")
    
    feedback.send_feedback(
        message_type=FeedbackType.WARNING,
        title="Battery Low",
        content="Battery level is 15%. Please consider charging soon.",
        tts_text="My battery is getting low",
        visual_icon="warning",
        duration=4.0
    )
    time.sleep(5)
    
    # Example 6: Information feedback
    print("\n📋 Example 6: Information Feedback")
    print("Sending informational messages...")
    
    feedback.send_feedback(
        message_type=FeedbackType.INFO,
        title="System Status",
        content="All systems are operational. Sensors are functioning normally.",
        tts_text="All systems are operational",
        visual_icon="info",
        duration=3.0
    )
    time.sleep(4)
    
    # Example 7: Confirmation feedback
    print("\n📋 Example 7: Confirmation Feedback")
    print("Sending confirmation messages...")
    
    feedback.send_feedback(
        message_type=FeedbackType.CONFIRMATION,
        title="Action Confirmed",
        content="You confirmed the action to move to the living room",
        tts_text="Action confirmed, moving to living room",
        visual_icon="check",
        duration=3.0
    )
    time.sleep(4)
    
    # Example 8: Visual feedback only
    print("\n📋 Example 8: Visual Feedback Only")
    print("Sending visual feedback without TTS...")
    
    visual_messages = [
        ("check", "Task completed", "green"),
        ("error", "Connection lost", "red"),
        ("warning", "Low memory", "yellow"),
        ("info", "Processing data", "blue"),
        ("progress", "Downloading updates", "cyan")
    ]
    
    for icon, message, color in visual_messages:
        feedback.send_visual_feedback(icon, message, color, duration=2.0)
        time.sleep(2.5)
    
    # Example 9: Custom error handler
    print("\n📋 Example 9: Custom Error Handler")
    print("Registering and testing custom error handler...")
    
    def custom_handler(error_message: str, context: dict):
        print(f"   Custom handler activated: {error_message}")
        feedback.send_feedback(
            message_type=FeedbackType.INFO,
            title="Custom Recovery",
            content="Custom error recovery strategy activated",
            tts_text="Custom recovery activated",
            visual_icon="custom"
        )
    
    # Register custom handler
    feedback.register_error_handler("CUSTOM_ERROR", custom_handler)
    
    # Test custom error
    feedback.send_error("CUSTOM_ERROR", "This is a custom error for demonstration", context={"demo": True})
    time.sleep(4)
    
    # Example 10: Feedback statistics
    print("\n📋 Example 10: Feedback Statistics")
    print("Displaying feedback statistics...")
    
    stats = feedback.get_feedback_stats()
    
    print("📊 Feedback Statistics:")
    print(f"   Total feedback sent: {stats['total_feedback']}")
    print(f"   Active feedback: {stats['active_feedback']}")
    print(f"   Error count: {stats['error_count']}")
    print(f"   TTS available: {stats['tts_available']}")
    print(f"   Visual feedback enabled: {stats['visual_feedback_enabled']}")
    print(f"   AAC feedback enabled: {stats['aac_feedback_enabled']}")
    
    # Get recent feedback
    recent_feedback = feedback.get_recent_feedback(5)
    print(f"\n📝 Recent feedback ({len(recent_feedback)}):")
    for i, fb in enumerate(recent_feedback, 1):
        print(f"   {i}. {fb.title}: {fb.content}")
    
    # Example 11: Complex feedback scenario
    print("\n📋 Example 11: Complex Feedback Scenario")
    print("Simulating a complex robot task with multiple feedback types...")
    
    # Task: "Bring me a cup from the kitchen"
    task_steps = [
        {
            "step": "Understanding request",
            "feedback": FeedbackType.INFO,
            "message": "I understand you want a cup from the kitchen",
            "tts": "I understand, you want a cup from the kitchen"
        },
        {
            "step": "Planning path",
            "feedback": FeedbackType.PROGRESS,
            "message": "Planning the best route to the kitchen",
            "tts": "Planning my route to the kitchen"
        },
        {
            "step": "Moving to kitchen",
            "feedback": FeedbackType.PROGRESS,
            "message": "Moving to the kitchen",
            "tts": "Moving to the kitchen now"
        },
        {
            "step": "Searching for cup",
            "feedback": FeedbackType.PROGRESS,
            "message": "Looking for a cup in the kitchen",
            "tts": "Looking for a cup"
        },
        {
            "step": "Cup found",
            "feedback": FeedbackType.SUCCESS,
            "message": "Found a cup on the counter",
            "tts": "I found a cup on the counter"
        },
        {
            "step": "Picking up cup",
            "feedback": FeedbackType.PROGRESS,
            "message": "Picking up the cup",
            "tts": "Picking up the cup"
        },
        {
            "step": "Returning to user",
            "feedback": FeedbackType.PROGRESS,
            "message": "Bringing the cup to you",
            "tts": "Bringing the cup to you"
        },
        {
            "step": "Task completed",
            "feedback": FeedbackType.SUCCESS,
            "message": "Here is your cup from the kitchen",
            "tts": "Here is your cup from the kitchen"
        }
    ]
    
    for i, step in enumerate(task_steps, 1):
        print(f"   Step {i}: {step['step']}")
        feedback.send_feedback(
            message_type=step['feedback'],
            title=f"Step {i}: {step['step']}",
            content=step['message'],
            tts_text=step['tts'],
            visual_icon="progress" if step['feedback'] == FeedbackType.PROGRESS else "check",
            duration=2.0
        )
        time.sleep(2.5)
    
    # Example 12: Error recovery scenario
    print("\n📋 Example 12: Error Recovery Scenario")
    print("Simulating error recovery...")
    
    # Simulate object not found with recovery suggestions
    feedback.send_error(
        error_code="OBJECT_NOT_FOUND",
        error_message="I couldn't find the red cup you requested",
        context={"object_name": "red cup", "searched_locations": ["counter", "cabinet", "sink"]}
    )
    time.sleep(3)
    
    # Provide recovery suggestions
    feedback.send_feedback(
        message_type=FeedbackType.INFO,
        title="Suggestions",
        content="I found a blue cup and a white cup. Would you like one of those instead?",
        tts_text="I found a blue cup and a white cup. Would you like one of those instead?",
        visual_icon="help",
        duration=4.0
    )
    time.sleep(5)
    
    # Final completion message
    print("\n📋 Example 13: Completion Message")
    feedback.send_feedback(
        message_type=FeedbackType.SUCCESS,
        title="Examples Completed",
        content="All feedback examples have been demonstrated successfully",
        tts_text="All feedback examples completed successfully",
        visual_icon="check",
        duration=3.0
    )
    time.sleep(4)
    
    print("\n✅ Feedback module example completed successfully!")
    
    # Cleanup
    feedback.shutdown()

if __name__ == "__main__":
    main() 