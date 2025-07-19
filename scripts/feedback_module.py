#!/usr/bin/env python3
"""
Feedback Module for FernAssist

This module provides multi-modal feedback to users including text-to-speech,
visual feedback to AAC apps, and comprehensive error handling.
"""

import rospy
import json
import time
import threading
import queue
from typing import Dict, List, Optional, Callable
from dataclasses import dataclass
from enum import Enum
from pathlib import Path

# ROS 2 message imports
from std_msgs.msg import String, Bool, Header
from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
from fernassist.msg import FeedbackMessage, ErrorMessage, TTSMessage

# TTS imports
try:
    import pyttsx3
    TTS_AVAILABLE = True
except ImportError:
    TTS_AVAILABLE = False
    rospy.logwarn("pyttsx3 not available - TTS disabled")

try:
    from gtts import gTTS
    import pygame
    GTTS_AVAILABLE = True
except ImportError:
    GTTS_AVAILABLE = False
    rospy.logwarn("gTTS not available - Google TTS disabled")

class FeedbackType(Enum):
    """Enumeration of feedback types."""
    SUCCESS = "success"
    ERROR = "error"
    WARNING = "warning"
    INFO = "info"
    PROGRESS = "progress"
    CONFIRMATION = "confirmation"

class TTSProvider(Enum):
    """Enumeration of TTS providers."""
    PYTTSX3 = "pyttsx3"
    GTTS = "gtts"
    ESPEAK = "espeak"
    FESTIVAL = "festival"

@dataclass
class FeedbackConfig:
    """Configuration for feedback system."""
    # TTS Configuration
    tts_provider: TTSProvider = TTSProvider.PYTTSX3
    tts_voice: str = "default"
    tts_rate: int = 150
    tts_volume: float = 0.8
    enable_tts: bool = True
    
    # Visual Feedback Configuration
    enable_visual_feedback: bool = True
    visual_duration: float = 3.0
    enable_animations: bool = True
    
    # Error Handling Configuration
    enable_error_recovery: bool = True
    max_retry_attempts: int = 3
    error_timeout: float = 10.0
    
    # AAC Integration Configuration
    aac_feedback_endpoint: str = "http://localhost:8080/aac/feedback"
    enable_aac_feedback: bool = True
    
    # Logging Configuration
    enable_feedback_logging: bool = True
    feedback_log_file: str = "/tmp/fernassist_feedback.log"

@dataclass
class FeedbackMessage:
    """Feedback message structure."""
    message_type: FeedbackType
    title: str
    content: str
    tts_text: Optional[str] = None
    visual_icon: Optional[str] = None
    duration: float = 3.0
    priority: int = 1
    requires_confirmation: bool = False
    error_code: Optional[str] = None
    timestamp: float = 0.0

class FeedbackModule:
    """Multi-modal feedback system for FernAssist."""
    
    def __init__(self):
        rospy.init_node('feedback_module', anonymous=True)
        
        # Configuration
        self.config = self._load_config()
        
        # State tracking
        self.feedback_queue = queue.Queue()
        self.active_feedback = None
        self.feedback_history = []
        self.error_count = 0
        self.feedback_lock = threading.Lock()
        
        # TTS initialization
        self.tts_engine = None
        self._initialize_tts()
        
        # Initialize ROS 2 interface
        self._initialize_ros_interface()
        
        # Start feedback processing thread
        self.is_running = False
        self.feedback_thread = threading.Thread(target=self._feedback_processor)
        self.feedback_thread.daemon = True
        self.feedback_thread.start()
        
        # Error handling
        self.error_handlers = {}
        self._register_default_error_handlers()
        
        rospy.loginfo("Feedback Module initialized")
    
    def _load_config(self) -> FeedbackConfig:
        """Load feedback configuration from ROS parameters."""
        config = FeedbackConfig()
        
        # Load from ROS parameters
        config.tts_provider = TTSProvider(rospy.get_param('~tts_provider', config.tts_provider.value))
        config.tts_voice = rospy.get_param('~tts_voice', config.tts_voice)
        config.tts_rate = rospy.get_param('~tts_rate', config.tts_rate)
        config.tts_volume = rospy.get_param('~tts_volume', config.tts_volume)
        config.enable_tts = rospy.get_param('~enable_tts', config.enable_tts)
        
        config.enable_visual_feedback = rospy.get_param('~enable_visual_feedback', config.enable_visual_feedback)
        config.visual_duration = rospy.get_param('~visual_duration', config.visual_duration)
        config.enable_animations = rospy.get_param('~enable_animations', config.enable_animations)
        
        config.enable_error_recovery = rospy.get_param('~enable_error_recovery', config.enable_error_recovery)
        config.max_retry_attempts = rospy.get_param('~max_retry_attempts', config.max_retry_attempts)
        config.error_timeout = rospy.get_param('~error_timeout', config.error_timeout)
        
        config.aac_feedback_endpoint = rospy.get_param('~aac_feedback_endpoint', config.aac_feedback_endpoint)
        config.enable_aac_feedback = rospy.get_param('~enable_aac_feedback', config.enable_aac_feedback)
        
        config.enable_feedback_logging = rospy.get_param('~enable_feedback_logging', config.enable_feedback_logging)
        config.feedback_log_file = rospy.get_param('~feedback_log_file', config.feedback_log_file)
        
        return config
    
    def _initialize_tts(self):
        """Initialize text-to-speech engine."""
        if not self.config.enable_tts:
            return
        
        try:
            if self.config.tts_provider == TTSProvider.PYTTSX3 and TTS_AVAILABLE:
                self.tts_engine = pyttsx3.init()
                self.tts_engine.setProperty('rate', self.config.tts_rate)
                self.tts_engine.setProperty('volume', self.config.tts_volume)
                
                # Set voice if specified
                if self.config.tts_voice != "default":
                    voices = self.tts_engine.getProperty('voices')
                    for voice in voices:
                        if self.config.tts_voice in voice.name:
                            self.tts_engine.setProperty('voice', voice.id)
                            break
                
                rospy.loginfo("TTS initialized with pyttsx3")
                
            elif self.config.tts_provider == TTSProvider.GTTS and GTTS_AVAILABLE:
                pygame.mixer.init()
                rospy.loginfo("TTS initialized with gTTS")
                
            else:
                rospy.logwarn("TTS provider not available - TTS disabled")
                
        except Exception as e:
            rospy.logerr(f"Failed to initialize TTS: {e}")
            self.tts_engine = None
    
    def _initialize_ros_interface(self):
        """Initialize ROS 2 publishers and subscribers."""
        # Publishers
        self.feedback_publisher = rospy.Publisher(
            '/fernassist/feedback',
            FeedbackMessage,
            queue_size=10
        )
        
        self.error_publisher = rospy.Publisher(
            '/fernassist/errors',
            ErrorMessage,
            queue_size=10
        )
        
        self.tts_publisher = rospy.Publisher(
            '/fernassist/tts',
            TTSMessage,
            queue_size=10
        )
        
        self.visual_feedback_publisher = rospy.Publisher(
            '/fernassist/visual_feedback',
            MarkerArray,
            queue_size=10
        )
        
        # Subscribers
        self.action_feedback_subscriber = rospy.Subscriber(
            '/fernassist/action_feedback',
            String,
            self._action_feedback_callback
        )
        
        self.error_subscriber = rospy.Subscriber(
            '/fernassist/error_reports',
            String,
            self._error_callback
        )
        
        self.confirmation_subscriber = rospy.Subscriber(
            '/fernassist/confirmations',
            Bool,
            self._confirmation_callback
        )
    
    def _action_feedback_callback(self, msg: String):
        """Process action feedback from robot controller."""
        try:
            feedback_data = json.loads(msg.data)
            self.send_feedback(
                message_type=FeedbackType.SUCCESS,
                title="Action Completed",
                content=feedback_data.get('message', 'Action completed successfully'),
                tts_text=feedback_data.get('tts_text'),
                visual_icon=feedback_data.get('icon', 'check')
            )
        except Exception as e:
            rospy.logerr(f"Error processing action feedback: {e}")
    
    def _error_callback(self, msg: String):
        """Process error reports from system components."""
        try:
            error_data = json.loads(msg.data)
            self.send_error(
                error_code=error_data.get('error_code', 'UNKNOWN'),
                error_message=error_data.get('message', 'An error occurred'),
                severity=error_data.get('severity', 'ERROR'),
                context=error_data.get('context', {})
            )
        except Exception as e:
            rospy.logerr(f"Error processing error report: {e}")
    
    def _confirmation_callback(self, msg: Bool):
        """Process user confirmations."""
        if msg.data:
            self.send_feedback(
                message_type=FeedbackType.CONFIRMATION,
                title="Confirmed",
                content="Action confirmed by user",
                tts_text="Action confirmed",
                visual_icon="check"
            )
        else:
            self.send_feedback(
                message_type=FeedbackType.WARNING,
                title="Cancelled",
                content="Action cancelled by user",
                tts_text="Action cancelled",
                visual_icon="cancel"
            )
    
    def send_feedback(self, message_type: FeedbackType, title: str, content: str,
                     tts_text: Optional[str] = None, visual_icon: Optional[str] = None,
                     duration: float = 3.0, priority: int = 1, requires_confirmation: bool = False):
        """Send multi-modal feedback to user."""
        feedback_msg = FeedbackMessage(
            message_type=message_type,
            title=title,
            content=content,
            tts_text=tts_text or content,
            visual_icon=visual_icon,
            duration=duration,
            priority=priority,
            requires_confirmation=requires_confirmation,
            timestamp=time.time()
        )
        
        # Add to feedback queue
        self.feedback_queue.put(feedback_msg)
        
        # Log feedback
        if self.config.enable_feedback_logging:
            self._log_feedback(feedback_msg)
    
    def send_error(self, error_code: str, error_message: str, severity: str = "ERROR",
                  context: Dict = None, auto_recovery: bool = True):
        """Send error feedback with optional recovery."""
        # Create error message
        error_msg = ErrorMessage()
        error_msg.error_code = error_code
        error_msg.error_message = error_message
        error_msg.severity = severity
        error_msg.timestamp = rospy.Time.now()
        error_msg.context = json.dumps(context or {})
        
        # Publish error
        self.error_publisher.publish(error_msg)
        
        # Create feedback message
        feedback_type = FeedbackType.ERROR if severity == "ERROR" else FeedbackType.WARNING
        icon = "error" if severity == "ERROR" else "warning"
        
        self.send_feedback(
            message_type=feedback_type,
            title=f"Error: {error_code}",
            content=error_message,
            tts_text=self._generate_error_tts(error_code, error_message),
            visual_icon=icon,
            duration=5.0,
            priority=10
        )
        
        # Handle error recovery
        if auto_recovery and self.config.enable_error_recovery:
            self._handle_error_recovery(error_code, error_message, context)
        
        # Update error count
        with self.feedback_lock:
            self.error_count += 1
    
    def send_tts(self, text: str, voice: Optional[str] = None, rate: Optional[int] = None):
        """Send text-to-speech message."""
        if not self.config.enable_tts:
            return
        
        # Create TTS message
        tts_msg = TTSMessage()
        tts_msg.text = text
        tts_msg.voice = voice or self.config.tts_voice
        tts_msg.rate = rate or self.config.tts_rate
        tts_msg.timestamp = rospy.Time.now()
        
        # Publish TTS message
        self.tts_publisher.publish(tts_msg)
        
        # Speak using local TTS engine
        self._speak_text(text, voice, rate)
    
    def send_visual_feedback(self, icon: str, message: str, color: str = "green",
                           position: List[float] = None, duration: float = 3.0):
        """Send visual feedback to AAC app or display."""
        # Create visual marker
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "feedback"
        marker.id = int(time.time() * 1000) % 1000
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # Set position
        if position:
            marker.pose.position.x = position[0]
            marker.pose.position.y = position[1]
            marker.pose.position.z = position[2]
        else:
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 2.0
        
        # Set text and appearance
        marker.text = f"{icon} {message}"
        marker.scale.z = 0.2  # Text size
        
        # Set color
        if color == "green":
            marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 0.0
        elif color == "red":
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.0
        elif color == "yellow":
            marker.color.r, marker.color.g, marker.color.b = 1.0, 1.0, 0.0
        elif color == "blue":
            marker.color.r, marker.color.g, marker.color.b = 0.0, 0.0, 1.0
        else:
            marker.color.r, marker.color.g, marker.color.b = 1.0, 1.0, 1.0
        
        marker.color.a = 1.0
        
        # Set lifetime
        marker.lifetime = rospy.Duration(duration)
        
        # Publish marker
        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        self.visual_feedback_publisher.publish(marker_array)
        
        # Send to AAC app if enabled
        if self.config.enable_aac_feedback:
            self._send_aac_feedback(icon, message, color, duration)
    
    def _speak_text(self, text: str, voice: Optional[str] = None, rate: Optional[int] = None):
        """Speak text using TTS engine."""
        if not self.tts_engine:
            return
        
        try:
            if self.config.tts_provider == TTSProvider.PYTTSX3 and TTS_AVAILABLE:
                # Set properties if provided
                if rate:
                    self.tts_engine.setProperty('rate', rate)
                if voice:
                    voices = self.tts_engine.getProperty('voices')
                    for v in voices:
                        if voice in v.name:
                            self.tts_engine.setProperty('voice', v.id)
                            break
                
                # Speak text
                self.tts_engine.say(text)
                self.tts_engine.runAndWait()
                
            elif self.config.tts_provider == TTSProvider.GTTS and GTTS_AVAILABLE:
                # Generate speech file
                tts = gTTS(text=text, lang='en')
                temp_file = f"/tmp/fernassist_tts_{int(time.time())}.mp3"
                tts.save(temp_file)
                
                # Play audio
                pygame.mixer.music.load(temp_file)
                pygame.mixer.music.play()
                
                # Wait for playback to complete
                while pygame.mixer.music.get_busy():
                    pygame.time.Clock().tick(10)
                
                # Clean up
                import os
                os.remove(temp_file)
                
        except Exception as e:
            rospy.logerr(f"TTS error: {e}")
    
    def _send_aac_feedback(self, icon: str, message: str, color: str, duration: float):
        """Send visual feedback to AAC app."""
        try:
            import requests
            
            feedback_data = {
                'icon': icon,
                'message': message,
                'color': color,
                'duration': duration,
                'timestamp': time.time()
            }
            
            response = requests.post(
                self.config.aac_feedback_endpoint,
                json=feedback_data,
                timeout=2.0
            )
            
            if response.status_code == 200:
                rospy.logdebug("AAC feedback sent successfully")
            else:
                rospy.logwarn(f"AAC feedback failed: {response.status_code}")
                
        except Exception as e:
            rospy.logdebug(f"AAC feedback error: {e}")
    
    def _generate_error_tts(self, error_code: str, error_message: str) -> str:
        """Generate user-friendly TTS text for errors."""
        error_tts_map = {
            'OBJECT_NOT_FOUND': "I couldn't find the object you requested",
            'PATH_BLOCKED': "The path is blocked, I cannot move there",
            'GRIPPER_FAILED': "I couldn't pick up the object",
            'MOTION_FAILED': "I couldn't complete the movement",
            'SENSOR_ERROR': "There's a problem with my sensors",
            'BATTERY_LOW': "My battery is getting low",
            'NETWORK_ERROR': "I'm having trouble connecting",
            'PERMISSION_DENIED': "I don't have permission to do that",
            'TIMEOUT': "The operation took too long",
            'UNKNOWN_ERROR': "Something went wrong, please try again"
        }
        
        return error_tts_map.get(error_code, error_message)
    
    def _handle_error_recovery(self, error_code: str, error_message: str, context: Dict):
        """Handle error recovery based on error type."""
        if error_code in self.error_handlers:
            handler = self.error_handlers[error_code]
            handler(error_message, context)
        else:
            # Default error handling
            self._default_error_handler(error_code, error_message, context)
    
    def _register_default_error_handlers(self):
        """Register default error handlers."""
        self.error_handlers = {
            'OBJECT_NOT_FOUND': self._handle_object_not_found,
            'PATH_BLOCKED': self._handle_path_blocked,
            'GRIPPER_FAILED': self._handle_gripper_failed,
            'MOTION_FAILED': self._handle_motion_failed,
            'SENSOR_ERROR': self._handle_sensor_error,
            'BATTERY_LOW': self._handle_battery_low,
            'NETWORK_ERROR': self._handle_network_error,
            'PERMISSION_DENIED': self._handle_permission_denied,
            'TIMEOUT': self._handle_timeout
        }
    
    def _handle_object_not_found(self, error_message: str, context: Dict):
        """Handle object not found error."""
        object_name = context.get('object_name', 'the object')
        
        self.send_feedback(
            message_type=FeedbackType.WARNING,
            title="Object Not Found",
            content=f"I couldn't find {object_name}. Please check if it's in the expected location.",
            tts_text=f"I couldn't find {object_name}",
            visual_icon="search",
            duration=4.0
        )
        
        # Suggest alternatives
        self.send_feedback(
            message_type=FeedbackType.INFO,
            title="Suggestions",
            content="You can try: 1) Check the object location 2) Use a different name 3) Ask me to search",
            tts_text="Would you like me to search for it or try a different approach?",
            visual_icon="help",
            duration=5.0
        )
    
    def _handle_path_blocked(self, error_message: str, context: Dict):
        """Handle path blocked error."""
        self.send_feedback(
            message_type=FeedbackType.WARNING,
            title="Path Blocked",
            content="The path to the destination is blocked. I cannot move there safely.",
            tts_text="The path is blocked, I cannot move there",
            visual_icon="blocked",
            duration=4.0
        )
        
        # Suggest alternatives
        self.send_feedback(
            message_type=FeedbackType.INFO,
            title="Alternatives",
            content="You can try: 1) Clear the path 2) Choose a different destination 3) Ask me to find an alternative route",
            tts_text="Would you like me to find an alternative route?",
            visual_icon="route",
            duration=5.0
        )
    
    def _handle_gripper_failed(self, error_message: str, context: Dict):
        """Handle gripper failure error."""
        self.send_feedback(
            message_type=FeedbackType.ERROR,
            title="Gripper Failed",
            content="I couldn't pick up the object. The gripper may be blocked or the object is too heavy.",
            tts_text="I couldn't pick up the object",
            visual_icon="gripper_error",
            duration=4.0
        )
    
    def _handle_motion_failed(self, error_message: str, context: Dict):
        """Handle motion failure error."""
        self.send_feedback(
            message_type=FeedbackType.ERROR,
            title="Motion Failed",
            content="I couldn't complete the movement. There may be an obstacle or mechanical issue.",
            tts_text="I couldn't complete the movement",
            visual_icon="motion_error",
            duration=4.0
        )
    
    def _handle_sensor_error(self, error_message: str, context: Dict):
        """Handle sensor error."""
        self.send_feedback(
            message_type=FeedbackType.ERROR,
            title="Sensor Error",
            content="There's a problem with my sensors. I may not be able to see or navigate properly.",
            tts_text="There's a problem with my sensors",
            visual_icon="sensor_error",
            duration=4.0
        )
    
    def _handle_battery_low(self, error_message: str, context: Dict):
        """Handle low battery warning."""
        battery_level = context.get('battery_level', 'low')
        
        self.send_feedback(
            message_type=FeedbackType.WARNING,
            title="Low Battery",
            content=f"My battery is {battery_level}. Please consider charging me soon.",
            tts_text=f"My battery is {battery_level}",
            visual_icon="battery_low",
            duration=4.0
        )
    
    def _handle_network_error(self, error_message: str, context: Dict):
        """Handle network error."""
        self.send_feedback(
            message_type=FeedbackType.WARNING,
            title="Network Error",
            content="I'm having trouble connecting to the network. Some features may be limited.",
            tts_text="I'm having trouble connecting",
            visual_icon="network_error",
            duration=4.0
        )
    
    def _handle_permission_denied(self, error_message: str, context: Dict):
        """Handle permission denied error."""
        self.send_feedback(
            message_type=FeedbackType.WARNING,
            title="Permission Denied",
            content="I don't have permission to perform this action. Please check your settings.",
            tts_text="I don't have permission to do that",
            visual_icon="permission_denied",
            duration=4.0
        )
    
    def _handle_timeout(self, error_message: str, context: Dict):
        """Handle timeout error."""
        self.send_feedback(
            message_type=FeedbackType.WARNING,
            title="Operation Timeout",
            content="The operation took too long and timed out. Please try again.",
            tts_text="The operation took too long",
            visual_icon="timeout",
            duration=4.0
        )
    
    def _default_error_handler(self, error_code: str, error_message: str, context: Dict):
        """Default error handler for unknown errors."""
        self.send_feedback(
            message_type=FeedbackType.ERROR,
            title="System Error",
            content=f"An unexpected error occurred: {error_message}",
            tts_text="Something went wrong, please try again",
            visual_icon="error",
            duration=4.0
        )
    
    def _feedback_processor(self):
        """Background thread for processing feedback queue."""
        self.is_running = True
        
        while self.is_running and not rospy.is_shutdown():
            try:
                # Get feedback from queue with timeout
                feedback_msg = self.feedback_queue.get(timeout=1.0)
                
                with self.feedback_lock:
                    self.active_feedback = feedback_msg
                    self.feedback_history.append(feedback_msg)
                    
                    # Limit history size
                    if len(self.feedback_history) > 100:
                        self.feedback_history = self.feedback_history[-100:]
                
                # Process feedback
                self._process_feedback(feedback_msg)
                
                # Wait for duration
                time.sleep(feedback_msg.duration)
                
                with self.feedback_lock:
                    self.active_feedback = None
                
            except queue.Empty:
                continue
            except Exception as e:
                rospy.logerr(f"Error in feedback processor: {e}")
    
    def _process_feedback(self, feedback_msg: FeedbackMessage):
        """Process individual feedback message."""
        try:
            # Send TTS
            if feedback_msg.tts_text:
                self.send_tts(feedback_msg.tts_text)
            
            # Send visual feedback
            if feedback_msg.visual_icon:
                color = self._get_feedback_color(feedback_msg.message_type)
                self.send_visual_feedback(
                    feedback_msg.visual_icon,
                    feedback_msg.content,
                    color,
                    duration=feedback_msg.duration
                )
            
            # Publish ROS message
            ros_feedback_msg = FeedbackMessage()
            ros_feedback_msg.message_type = feedback_msg.message_type.value
            ros_feedback_msg.title = feedback_msg.title
            ros_feedback_msg.content = feedback_msg.content
            ros_feedback_msg.timestamp = rospy.Time.now()
            self.feedback_publisher.publish(ros_feedback_msg)
            
        except Exception as e:
            rospy.logerr(f"Error processing feedback: {e}")
    
    def _get_feedback_color(self, message_type: FeedbackType) -> str:
        """Get color for feedback type."""
        color_map = {
            FeedbackType.SUCCESS: "green",
            FeedbackType.ERROR: "red",
            FeedbackType.WARNING: "yellow",
            FeedbackType.INFO: "blue",
            FeedbackType.PROGRESS: "blue",
            FeedbackType.CONFIRMATION: "green"
        }
        return color_map.get(message_type, "white")
    
    def _log_feedback(self, feedback_msg: FeedbackMessage):
        """Log feedback to file."""
        try:
            log_entry = {
                'timestamp': feedback_msg.timestamp,
                'type': feedback_msg.message_type.value,
                'title': feedback_msg.title,
                'content': feedback_msg.content,
                'priority': feedback_msg.priority
            }
            
            with open(self.config.feedback_log_file, 'a') as f:
                f.write(json.dumps(log_entry) + '\n')
                
        except Exception as e:
            rospy.logerr(f"Error logging feedback: {e}")
    
    def get_feedback_stats(self) -> Dict:
        """Get feedback statistics."""
        with self.feedback_lock:
            return {
                'total_feedback': len(self.feedback_history),
                'active_feedback': self.active_feedback is not None,
                'error_count': self.error_count,
                'tts_available': self.tts_engine is not None,
                'visual_feedback_enabled': self.config.enable_visual_feedback,
                'aac_feedback_enabled': self.config.enable_aac_feedback
            }
    
    def get_recent_feedback(self, count: int = 10) -> List[FeedbackMessage]:
        """Get recent feedback history."""
        with self.feedback_lock:
            return self.feedback_history[-count:]
    
    def clear_feedback_history(self):
        """Clear feedback history."""
        with self.feedback_lock:
            self.feedback_history.clear()
    
    def register_error_handler(self, error_code: str, handler: Callable):
        """Register custom error handler."""
        self.error_handlers[error_code] = handler
    
    def run(self):
        """Main run loop for the feedback module."""
        rospy.loginfo("Feedback Module started - ready to provide feedback...")
        rospy.spin()
    
    def shutdown(self):
        """Shutdown the feedback module."""
        self.is_running = False
        rospy.loginfo("Feedback Module shutting down")

# Convenience functions
def create_feedback_module() -> FeedbackModule:
    """Create and return a feedback module instance."""
    return FeedbackModule()

def send_success_feedback(module: FeedbackModule, title: str, content: str, tts_text: str = None):
    """Send success feedback."""
    module.send_feedback(FeedbackType.SUCCESS, title, content, tts_text, "check")

def send_error_feedback(module: FeedbackModule, error_code: str, error_message: str, context: Dict = None):
    """Send error feedback."""
    module.send_error(error_code, error_message, context=context)

if __name__ == '__main__':
    try:
        feedback_module = FeedbackModule()
        feedback_module.run()
    except rospy.ROSInterruptException:
        feedback_module.shutdown() 