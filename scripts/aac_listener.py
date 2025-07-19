#!/usr/bin/env python3
"""
AAC Listener Module for FernAssist

This module handles communication with Augmentative and Alternative Communication (AAC) devices.
It supports multiple input methods and processes partial/non-grammatical phrases for the LLM interpreter.
"""

import rospy
import json
import time
import threading
import websocket
import requests
from pathlib import Path
from datetime import datetime
from std_msgs.msg import String
from fernassist.msg import AACInput, UserIntent

class AACListener:
    """Listens for AAC device inputs from multiple sources and processes them."""
    
    def __init__(self):
        rospy.init_node('aac_listener', anonymous=True)
        
        # Publishers
        self.intent_publisher = rospy.Publisher('/fernassist/user_intent', UserIntent, queue_size=10)
        self.raw_input_publisher = rospy.Publisher('/aac/raw_input', String, queue_size=10)
        
        # Configuration parameters
        self.confidence_threshold = rospy.get_param('~confidence_threshold', 0.3)
        self.input_timeout = rospy.get_param('~input_timeout', 10.0)
        self.device_polling_rate = rospy.get_param('~device_polling_rate', 10.0)
        
        # Input method configuration
        self.enable_file_input = rospy.get_param('~enable_file_input', True)
        self.enable_api_input = rospy.get_param('~enable_api_input', False)
        self.enable_websocket_input = rospy.get_param('~enable_websocket_input', False)
        self.enable_mock_input = rospy.get_param('~enable_mock_input', True)
        
        # File input configuration
        self.input_file_path = rospy.get_param('~input_file_path', '/tmp/aac_input.txt')
        self.last_file_modification = 0
        
        # API input configuration
        self.api_endpoint = rospy.get_param('~api_endpoint', 'http://localhost:8080/aac/input')
        self.api_polling_interval = rospy.get_param('~api_polling_interval', 5.0)
        
        # WebSocket configuration
        self.websocket_url = rospy.get_param('~websocket_url', 'ws://localhost:8080/aac/ws')
        
        # Mock input configuration
        self.mock_inputs = [
            "I'm thirsty",
            "go to kitchen",
            "pick up cup",
            "I want water",
            "help me",
            "I'm tired",
            "turn on light",
            "open door",
            "I need help",
            "where is my phone"
        ]
        self.mock_input_index = 0
        
        # State tracking
        self.last_input_time = time.time()
        self.input_buffer = []
        self.processing_lock = threading.Lock()
        
        # Initialize input methods
        self._initialize_input_methods()
        
        rospy.loginfo("AAC Listener initialized with multiple input methods")
    
    def _initialize_input_methods(self):
        """Initialize different input methods based on configuration."""
        if self.enable_file_input:
            self._setup_file_input()
        
        if self.enable_api_input:
            self._setup_api_input()
        
        if self.enable_websocket_input:
            self._setup_websocket_input()
        
        if self.enable_mock_input:
            self._setup_mock_input()
    
    def _setup_file_input(self):
        """Setup file-based input monitoring."""
        try:
            # Create input file if it doesn't exist
            input_file = Path(self.input_file_path)
            input_file.parent.mkdir(parents=True, exist_ok=True)
            
            if not input_file.exists():
                input_file.write_text("# AAC Input File\n# Add user inputs here, one per line\n")
            
            self.last_file_modification = input_file.stat().st_mtime
            
            # Start file monitoring thread
            file_thread = threading.Thread(target=self._monitor_file_input, daemon=True)
            file_thread.start()
            
            rospy.loginfo(f"File input monitoring enabled: {self.input_file_path}")
            
        except Exception as e:
            rospy.logerr(f"Failed to setup file input: {e}")
    
    def _setup_api_input(self):
        """Setup API-based input polling."""
        try:
            # Start API polling thread
            api_thread = threading.Thread(target=self._poll_api_input, daemon=True)
            api_thread.start()
            
            rospy.loginfo(f"API input polling enabled: {self.api_endpoint}")
            
        except Exception as e:
            rospy.logerr(f"Failed to setup API input: {e}")
    
    def _setup_websocket_input(self):
        """Setup WebSocket-based input listening."""
        try:
            # Start WebSocket thread
            ws_thread = threading.Thread(target=self._websocket_listener, daemon=True)
            ws_thread.start()
            
            rospy.loginfo(f"WebSocket input enabled: {self.websocket_url}")
            
        except Exception as e:
            rospy.logerr(f"Failed to setup WebSocket input: {e}")
    
    def _setup_mock_input(self):
        """Setup mock input for testing and development."""
        try:
            # Start mock input thread
            mock_thread = threading.Thread(target=self._generate_mock_input, daemon=True)
            mock_thread.start()
            
            rospy.loginfo("Mock input generation enabled")
            
        except Exception as e:
            rospy.logerr(f"Failed to setup mock input: {e}")
    
    def _monitor_file_input(self):
        """Monitor file for new AAC inputs."""
        while not rospy.is_shutdown():
            try:
                input_file = Path(self.input_file_path)
                
                if input_file.exists():
                    current_modification = input_file.stat().st_mtime
                    
                    if current_modification > self.last_file_modification:
                        # Read new lines
                        with open(input_file, 'r') as f:
                            lines = f.readlines()
                        
                        # Process new lines
                        for line in lines[self.last_file_modification:]:
                            line = line.strip()
                            if line and not line.startswith('#'):
                                self._process_raw_input(line, "file")
                        
                        self.last_file_modification = current_modification
                
                time.sleep(1.0)  # Check every second
                
            except Exception as e:
                rospy.logerr(f"Error in file monitoring: {e}")
                time.sleep(5.0)
    
    def _poll_api_input(self):
        """Poll API endpoint for new AAC inputs."""
        while not rospy.is_shutdown():
            try:
                response = requests.get(self.api_endpoint, timeout=5.0)
                
                if response.status_code == 200:
                    data = response.json()
                    
                    if 'inputs' in data:
                        for input_data in data['inputs']:
                            text = input_data.get('text', '')
                            confidence = input_data.get('confidence', 1.0)
                            input_type = input_data.get('type', 'api')
                            
                            if text:
                                self._process_raw_input(text, input_type, confidence)
                
                time.sleep(self.api_polling_interval)
                
            except requests.exceptions.RequestException as e:
                rospy.logdebug(f"API polling failed (expected if no server): {e}")
                time.sleep(self.api_polling_interval)
            except Exception as e:
                rospy.logerr(f"Error in API polling: {e}")
                time.sleep(self.api_polling_interval)
    
    def _websocket_listener(self):
        """Listen for WebSocket AAC inputs."""
        try:
            def on_message(ws, message):
                try:
                    data = json.loads(message)
                    text = data.get('text', '')
                    confidence = data.get('confidence', 1.0)
                    input_type = data.get('type', 'websocket')
                    
                    if text:
                        self._process_raw_input(text, input_type, confidence)
                        
                except json.JSONDecodeError:
                    # Treat as plain text
                    self._process_raw_input(message, 'websocket')
                except Exception as e:
                    rospy.logerr(f"Error processing WebSocket message: {e}")
            
            def on_error(ws, error):
                rospy.logerr(f"WebSocket error: {error}")
            
            def on_close(ws, close_status_code, close_msg):
                rospy.loginfo("WebSocket connection closed")
            
            def on_open(ws):
                rospy.loginfo("WebSocket connection established")
            
            # Create WebSocket connection
            ws = websocket.WebSocketApp(
                self.websocket_url,
                on_open=on_open,
                on_message=on_message,
                on_error=on_error,
                on_close=on_close
            )
            
            ws.run_forever()
            
        except Exception as e:
            rospy.logerr(f"WebSocket listener failed: {e}")
    
    def _generate_mock_input(self):
        """Generate mock AAC inputs for testing."""
        while not rospy.is_shutdown():
            try:
                # Generate mock input every 30 seconds
                time.sleep(30.0)
                
                if self.mock_inputs:
                    mock_text = self.mock_inputs[self.mock_input_index]
                    self.mock_input_index = (self.mock_input_index + 1) % len(self.mock_inputs)
                    
                    # Add some randomness to confidence
                    import random
                    confidence = random.uniform(0.7, 1.0)
                    
                    self._process_raw_input(mock_text, "mock", confidence)
                    
                    rospy.loginfo(f"Generated mock input: {mock_text}")
                
            except Exception as e:
                rospy.logerr(f"Error generating mock input: {e}")
                time.sleep(10.0)
    
    def _process_raw_input(self, text, input_type="unknown", confidence=1.0):
        """Process raw AAC input and convert to user intent."""
        with self.processing_lock:
            try:
                # Clean and validate input
                text = text.strip()
                if not text:
                    return
                
                # Update last input time
                self.last_input_time = time.time()
                
                # Publish raw input for logging
                raw_msg = String()
                raw_msg.data = json.dumps({
                    'text': text,
                    'type': input_type,
                    'confidence': confidence,
                    'timestamp': datetime.now().isoformat()
                })
                self.raw_input_publisher.publish(raw_msg)
                
                # Create AAC input message
                aac_msg = AACInput()
                aac_msg.text = text
                aac_msg.confidence = confidence
                aac_msg.input_type = input_type
                aac_msg.timestamp = rospy.Time.now()
                aac_msg.device_id = f"{input_type}_source"
                
                # Process the AAC input
                self._process_aac_input(aac_msg)
                
                rospy.loginfo(f"Processed {input_type} input: '{text}' (confidence: {confidence:.2f})")
                
            except Exception as e:
                rospy.logerr(f"Error processing raw input: {e}")
    
    def _process_aac_input(self, aac_msg):
        """Process incoming AAC input and convert to user intent."""
        try:
            # Extract text and metadata from AAC input
            text = aac_msg.text
            confidence = aac_msg.confidence
            input_type = aac_msg.input_type
            
            # Apply confidence threshold
            if confidence < self.confidence_threshold:
                rospy.logwarn(f"Low confidence input ignored: '{text}' (confidence: {confidence:.2f})")
                return
            
            # Preprocess text for partial/non-grammatical phrases
            processed_text = self._preprocess_text(text)
            
            # Create user intent message
            intent_msg = UserIntent()
            intent_msg.raw_text = processed_text
            intent_msg.confidence = confidence
            intent_msg.timestamp = rospy.Time.now()
            intent_msg.intent_type = self._classify_intent(processed_text)
            intent_msg.entities = self._extract_entities(processed_text)
            
            # Publish user intent
            self.intent_publisher.publish(intent_msg)
            
            rospy.loginfo(f"Published user intent: '{processed_text}' -> {intent_msg.intent_type}")
            
        except Exception as e:
            rospy.logerr(f"Error processing AAC input: {e}")
    
    def _preprocess_text(self, text):
        """Preprocess text to handle partial/non-grammatical phrases."""
        # Convert to lowercase for consistency
        text = text.lower().strip()
        
        # Handle common AAC patterns
        if text.startswith("i'm ") or text.startswith("i am "):
            # Keep as is - these are often complete thoughts
            pass
        elif text.startswith("i want ") or text.startswith("i need "):
            # Keep as is - these are clear intentions
            pass
        elif text in ["thirsty", "hungry", "tired", "help", "water", "food"]:
            # Single words that are clear intentions
            pass
        elif len(text.split()) <= 3:
            # Short phrases - keep as is
            pass
        else:
            # For longer text, try to extract the main intention
            # This is a simple heuristic - could be enhanced with NLP
            words = text.split()
            if len(words) > 5:
                # Take first few words as main intention
                text = " ".join(words[:3])
        
        return text
    
    def _classify_intent(self, text):
        """Classify the type of user intent."""
        text_lower = text.lower()
        
        # Basic intent classification
        if any(word in text_lower for word in ["thirsty", "water", "drink"]):
            return "need_drink"
        elif any(word in text_lower for word in ["hungry", "food", "eat"]):
            return "need_food"
        elif any(word in text_lower for word in ["tired", "sleep", "rest"]):
            return "need_rest"
        elif any(word in text_lower for word in ["help", "assist"]):
            return "need_help"
        elif any(word in text_lower for word in ["go", "move", "walk"]):
            return "movement"
        elif any(word in text_lower for word in ["pick", "grab", "get"]):
            return "manipulation"
        elif any(word in text_lower for word in ["turn", "open", "close"]):
            return "environment_control"
        else:
            return "general"
    
    def _extract_entities(self, text):
        """Extract entities from the text."""
        entities = []
        text_lower = text.lower()
        
        # Simple entity extraction
        locations = ["kitchen", "bedroom", "bathroom", "living room", "garden"]
        objects = ["cup", "water", "food", "phone", "book", "light", "door"]
        actions = ["go", "get", "pick", "turn", "open", "close", "help"]
        
        for location in locations:
            if location in text_lower:
                entities.append(location)
        
        for obj in objects:
            if obj in text_lower:
                entities.append(obj)
        
        for action in actions:
            if action in text_lower:
                entities.append(action)
        
        return entities
    
    def get_status(self):
        """Get current status of the AAC listener."""
        return {
            'active_input_methods': {
                'file': self.enable_file_input,
                'api': self.enable_api_input,
                'websocket': self.enable_websocket_input,
                'mock': self.enable_mock_input
            },
            'last_input_time': self.last_input_time,
            'input_buffer_size': len(self.input_buffer),
            'confidence_threshold': self.confidence_threshold
        }
    
    def run(self):
        """Main run loop."""
        rospy.loginfo("AAC Listener started - listening for inputs...")
        rospy.spin()

if __name__ == '__main__':
    try:
        listener = AACListener()
        listener.run()
    except rospy.ROSInterruptException:
        pass 