#!/usr/bin/env python3
"""
Test script for FernAssist Feedback Module

This script tests the multi-modal feedback system including TTS, visual feedback,
and error handling capabilities.
"""

import os
import sys
import time
import json
import threading
from pathlib import Path

def test_tts_availability():
    """Test TTS provider availability."""
    print("\n🗣️ Testing TTS Availability")
    print("=" * 50)
    
    # Test pyttsx3
    try:
        import pyttsx3
        engine = pyttsx3.init()
        voices = engine.getProperty('voices')
        print("✅ pyttsx3 is available")
        print(f"   Available voices: {len(voices)}")
        for i, voice in enumerate(voices[:3]):  # Show first 3 voices
            print(f"   {i+1}. {voice.name}")
        return True
    except ImportError:
        print("❌ pyttsx3 not available")
        print("💡 Install with: pip install pyttsx3")
    except Exception as e:
        print(f"❌ pyttsx3 error: {e}")
    
    # Test gTTS
    try:
        from gtts import gTTS
        import pygame
        pygame.mixer.init()
        print("✅ gTTS is available")
        return True
    except ImportError:
        print("❌ gTTS not available")
        print("💡 Install with: pip install gtts pygame")
    except Exception as e:
        print(f"❌ gTTS error: {e}")
    
    return False

def test_feedback_module_creation():
    """Test feedback module creation and initialization."""
    print("\n📢 Testing Feedback Module Creation")
    print("=" * 50)
    
    try:
        from scripts.feedback_module import FeedbackModule, FeedbackType, TTSProvider
        
        # Create feedback module
        feedback_module = FeedbackModule()
        
        print("✅ Feedback Module created successfully")
        print(f"   TTS Provider: {feedback_module.config.tts_provider.value}")
        print(f"   TTS Enabled: {feedback_module.config.enable_tts}")
        print(f"   Visual Feedback: {feedback_module.config.enable_visual_feedback}")
        print(f"   Error Recovery: {feedback_module.config.enable_error_recovery}")
        print(f"   AAC Feedback: {feedback_module.config.enable_aac_feedback}")
        
        return feedback_module
        
    except Exception as e:
        print(f"❌ Failed to create feedback module: {e}")
        return None

def test_tts_functionality(feedback_module):
    """Test text-to-speech functionality."""
    print("\n🎤 Testing TTS Functionality")
    print("=" * 50)
    
    if not feedback_module:
        print("❌ No feedback module available")
        return False
    
    if not feedback_module.config.enable_tts:
        print("⚠️ TTS is disabled in configuration")
        return False
    
    # Test basic TTS
    test_messages = [
        "Hello, I am FernAssist",
        "I am ready to help you",
        "Action completed successfully",
        "I couldn't find the object"
    ]
    
    print("🧪 Testing TTS messages:")
    for i, message in enumerate(test_messages, 1):
        print(f"   {i}. Speaking: '{message}'")
        feedback_module.send_tts(message)
        time.sleep(2)  # Wait for speech to complete
    
    print("✅ TTS functionality test completed")
    return True

def test_visual_feedback(feedback_module):
    """Test visual feedback functionality."""
    print("\n👁️ Testing Visual Feedback")
    print("=" * 50)
    
    if not feedback_module:
        print("❌ No feedback module available")
        return False
    
    if not feedback_module.config.enable_visual_feedback:
        print("⚠️ Visual feedback is disabled in configuration")
        return False
    
    # Test different types of visual feedback
    test_feedback = [
        ("check", "Action completed", "green"),
        ("error", "Object not found", "red"),
        ("warning", "Battery low", "yellow"),
        ("info", "Processing request", "blue"),
        ("progress", "Moving to location", "cyan")
    ]
    
    print("🧪 Testing visual feedback:")
    for icon, message, color in test_feedback:
        print(f"   Sending: {icon} - {message} ({color})")
        feedback_module.send_visual_feedback(icon, message, color, duration=2.0)
        time.sleep(2.5)  # Wait for feedback to display
    
    print("✅ Visual feedback test completed")
    return True

def test_error_handling(feedback_module):
    """Test error handling functionality."""
    print("\n⚠️ Testing Error Handling")
    print("=" * 50)
    
    if not feedback_module:
        print("❌ No feedback module available")
        return False
    
    # Test different error types
    test_errors = [
        ("OBJECT_NOT_FOUND", "I couldn't find the water bottle", {"object_name": "water bottle"}),
        ("PATH_BLOCKED", "The path to the kitchen is blocked", {"destination": "kitchen"}),
        ("GRIPPER_FAILED", "Failed to pick up the cup", {"object_name": "cup"}),
        ("BATTERY_LOW", "Battery level is 15%", {"battery_level": "15%"}),
        ("NETWORK_ERROR", "Connection to server failed", {"server": "main_server"})
    ]
    
    print("🧪 Testing error handling:")
    for error_code, error_message, context in test_errors:
        print(f"   Testing: {error_code}")
        feedback_module.send_error(error_code, error_message, context=context)
        time.sleep(3)  # Wait for error feedback
    
    print("✅ Error handling test completed")
    return True

def test_feedback_types(feedback_module):
    """Test different feedback types."""
    print("\n📋 Testing Feedback Types")
    print("=" * 50)
    
    if not feedback_module:
        print("❌ No feedback module available")
        return False
    
    from scripts.feedback_module import FeedbackType
    
    # Test different feedback types
    test_feedback = [
        (FeedbackType.SUCCESS, "Success", "Action completed successfully", "check"),
        (FeedbackType.ERROR, "Error", "Something went wrong", "error"),
        (FeedbackType.WARNING, "Warning", "Please check the object location", "warning"),
        (FeedbackType.INFO, "Info", "Processing your request", "info"),
        (FeedbackType.PROGRESS, "Progress", "Moving to the kitchen", "progress"),
        (FeedbackType.CONFIRMATION, "Confirmation", "Action confirmed", "check")
    ]
    
    print("🧪 Testing feedback types:")
    for feedback_type, title, content, icon in test_feedback:
        print(f"   {feedback_type.value}: {title}")
        feedback_module.send_feedback(
            message_type=feedback_type,
            title=title,
            content=content,
            tts_text=content,
            visual_icon=icon,
            duration=2.0
        )
        time.sleep(2.5)
    
    print("✅ Feedback types test completed")
    return True

def test_aac_integration(feedback_module):
    """Test AAC app integration."""
    print("\n📱 Testing AAC Integration")
    print("=" * 50)
    
    if not feedback_module:
        print("❌ No feedback module available")
        return False
    
    if not feedback_module.config.enable_aac_feedback:
        print("⚠️ AAC feedback is disabled in configuration")
        return False
    
    # Test AAC feedback (this will fail if no AAC server is running)
    try:
        feedback_module._send_aac_feedback("check", "Test message", "green", 2.0)
        print("✅ AAC feedback sent (server may not be running)")
    except Exception as e:
        print(f"⚠️ AAC feedback failed (expected if no server): {e}")
    
    return True

def test_feedback_queue(feedback_module):
    """Test feedback queue processing."""
    print("\n📬 Testing Feedback Queue")
    print("=" * 50)
    
    if not feedback_module:
        print("❌ No feedback module available")
        return False
    
    # Send multiple feedback messages quickly
    print("🧪 Sending multiple feedback messages:")
    for i in range(5):
        feedback_module.send_feedback(
            message_type=FeedbackType.INFO,
            title=f"Test {i+1}",
            content=f"This is test message {i+1}",
            tts_text=f"Test {i+1}",
            visual_icon="info",
            duration=1.0
        )
        print(f"   Queued message {i+1}")
    
    # Wait for processing
    time.sleep(6)
    
    # Check queue status
    stats = feedback_module.get_feedback_stats()
    print(f"✅ Queue test completed")
    print(f"   Total feedback: {stats['total_feedback']}")
    print(f"   Error count: {stats['error_count']}")
    
    return True

def test_feedback_statistics(feedback_module):
    """Test feedback statistics and history."""
    print("\n📊 Testing Feedback Statistics")
    print("=" * 50)
    
    if not feedback_module:
        print("❌ No feedback module available")
        return False
    
    # Get statistics
    stats = feedback_module.get_feedback_stats()
    
    print("📈 Feedback Statistics:")
    print(f"   Total feedback: {stats['total_feedback']}")
    print(f"   Active feedback: {stats['active_feedback']}")
    print(f"   Error count: {stats['error_count']}")
    print(f"   TTS available: {stats['tts_available']}")
    print(f"   Visual feedback enabled: {stats['visual_feedback_enabled']}")
    print(f"   AAC feedback enabled: {stats['aac_feedback_enabled']}")
    
    # Get recent feedback
    recent_feedback = feedback_module.get_recent_feedback(5)
    print(f"\n📝 Recent feedback ({len(recent_feedback)}):")
    for i, feedback in enumerate(recent_feedback, 1):
        print(f"   {i}. {feedback.title}: {feedback.content}")
    
    return True

def test_error_recovery(feedback_module):
    """Test error recovery strategies."""
    print("\n🔄 Testing Error Recovery")
    print("=" * 50)
    
    if not feedback_module:
        print("❌ No feedback module available")
        return False
    
    if not feedback_module.config.enable_error_recovery:
        print("⚠️ Error recovery is disabled in configuration")
        return False
    
    # Test custom error handler
    def custom_error_handler(error_message: str, context: dict):
        print(f"   Custom handler: {error_message}")
        feedback_module.send_feedback(
            message_type=FeedbackType.INFO,
            title="Custom Recovery",
            content="Custom error recovery activated",
            tts_text="Custom recovery activated",
            visual_icon="custom"
        )
    
    # Register custom handler
    feedback_module.register_error_handler("CUSTOM_ERROR", custom_error_handler)
    
    # Test custom error
    print("🧪 Testing custom error handler:")
    feedback_module.send_error("CUSTOM_ERROR", "This is a custom error", context={"test": True})
    time.sleep(3)
    
    print("✅ Error recovery test completed")
    return True

def show_usage_instructions():
    """Show usage instructions for the feedback module."""
    print("\n📖 Usage Instructions")
    print("=" * 50)
    print("1. Install TTS dependencies:")
    print("   pip install pyttsx3 gtts pygame")
    print()
    print("2. Launch feedback module:")
    print("   ros2 launch fernassist feedback_module.launch.py")
    print()
    print("3. Test with different TTS providers:")
    print("   ros2 launch fernassist feedback_module.launch.py tts_provider:=gtts")
    print()
    print("4. Monitor feedback topics:")
    print("   ros2 topic echo /fernassist/feedback")
    print("   ros2 topic echo /fernassist/errors")
    print("   ros2 topic echo /fernassist/tts")
    print()
    print("5. Use in Python:")
    print("   from scripts.feedback_module import FeedbackModule")
    print("   feedback = FeedbackModule()")
    print("   feedback.send_feedback(FeedbackType.SUCCESS, 'Success', 'Action completed')")

def main():
    """Main test function."""
    print("📢 FernAssist Feedback Module Test")
    print("=" * 60)
    
    # Test TTS availability
    tts_available = test_tts_availability()
    
    # Test feedback module creation
    feedback_module = test_feedback_module_creation()
    
    if feedback_module:
        # Test various functionalities
        test_tts_functionality(feedback_module)
        test_visual_feedback(feedback_module)
        test_error_handling(feedback_module)
        test_feedback_types(feedback_module)
        test_aac_integration(feedback_module)
        test_feedback_queue(feedback_module)
        test_feedback_statistics(feedback_module)
        test_error_recovery(feedback_module)
        
        # Cleanup
        feedback_module.shutdown()
    
    # Show usage instructions
    show_usage_instructions()
    
    print("\n🎉 Feedback Module test completed!")
    print(f"\nResults:")
    print(f"  TTS Available: {'✅' if tts_available else '❌'}")
    print(f"  Feedback Module Created: {'✅' if feedback_module else '❌'}")

if __name__ == "__main__":
    main() 