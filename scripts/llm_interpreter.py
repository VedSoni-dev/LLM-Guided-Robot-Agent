#!/usr/bin/env python3
"""
LLM Interpreter Module for FernAssist

This module processes user intents from AAC inputs and uses OpenAI GPT-4 API
(or HuggingFace Transformers as fallback) to interpret natural language commands
into structured robot actions with context memory.
"""

import rospy
import json
import os
import time
import threading
from datetime import datetime
from pathlib import Path
from std_msgs.msg import String
from fernassist.msg import UserIntent, RobotAction, LLMResponse

class LLMInterpreter:
    """Interprets user intents using OpenAI GPT-4 or HuggingFace Transformers."""
    
    def __init__(self):
        rospy.init_node('llm_interpreter', anonymous=True)
        
        # Publishers
        self.action_publisher = rospy.Publisher('/fernassist/robot_action', RobotAction, queue_size=10)
        self.response_publisher = rospy.Publisher('/fernassist/llm_response', LLMResponse, queue_size=10)
        
        # Subscribers
        rospy.Subscriber('/fernassist/user_intent', UserIntent, self.process_user_intent)
        
        # Configuration parameters
        self.llm_model = rospy.get_param('~llm_model', 'gpt-4')
        self.max_tokens = rospy.get_param('~max_tokens', 200)
        self.temperature = rospy.get_param('~temperature', 0.7)
        self.api_timeout = rospy.get_param('~api_timeout', 30.0)
        self.retry_attempts = rospy.get_param('~retry_attempts', 3)
        
        # Context memory configuration
        self.enable_context_memory = rospy.get_param('~enable_context_memory', True)
        self.max_context_length = rospy.get_param('~max_context_length', 10)
        self.context_file_path = rospy.get_param('~context_file_path', '/tmp/fernassist_context.json')
        
        # Initialize LLM and context memory
        self._initialize_llm()
        self._initialize_context_memory()
        
        # State tracking
        self.conversation_history = []
        self.context_lock = threading.Lock()
        
        rospy.loginfo("LLM Interpreter initialized")
    
    def _initialize_llm(self):
        """Initialize the LLM (OpenAI GPT-4 or HuggingFace fallback)."""
        self.llm_type = "unknown"
        self.llm_model_instance = None
        
        # Try OpenAI GPT-4 first
        if self._initialize_openai():
            self.llm_type = "openai"
            rospy.loginfo(f"OpenAI GPT-4 initialized with model: {self.llm_model}")
            return
        
        # Fallback to HuggingFace Transformers
        if self._initialize_huggingface():
            self.llm_type = "huggingface"
            rospy.loginfo("HuggingFace Transformers initialized as fallback")
            return
        
        # Fallback to mock responses
        self.llm_type = "mock"
        rospy.logwarn("No LLM available - using mock responses")
    
    def _initialize_openai(self):
        """Initialize OpenAI GPT-4 API."""
        try:
            import openai
            
            # Check for API key
            api_key = os.getenv('OPENAI_API_KEY')
            if not api_key:
                rospy.logwarn("OPENAI_API_KEY not found in environment variables")
                return False
            
            # Configure OpenAI
            openai.api_key = api_key
            
            # Test API connection
            try:
                response = openai.ChatCompletion.create(
                    model="gpt-3.5-turbo",  # Use cheaper model for test
                    messages=[{"role": "user", "content": "Hello"}],
                    max_tokens=10,
                    timeout=5
                )
                self.openai_client = openai
                return True
            except Exception as e:
                rospy.logwarn(f"OpenAI API test failed: {e}")
                return False
                
        except ImportError:
            rospy.logwarn("OpenAI package not installed")
            return False
        except Exception as e:
            rospy.logerr(f"Failed to initialize OpenAI: {e}")
            return False
    
    def _initialize_huggingface(self):
        """Initialize HuggingFace Transformers as fallback."""
        try:
            from transformers import pipeline, AutoTokenizer, AutoModelForCausalLM
            
            # Use a smaller, faster model for local inference
            model_name = "microsoft/DialoGPT-medium"  # Good for conversation
            
            self.tokenizer = AutoTokenizer.from_pretrained(model_name)
            self.model = AutoModelForCausalLM.from_pretrained(model_name)
            
            # Test the model
            test_input = "Hello"
            inputs = self.tokenizer.encode(test_input, return_tensors="pt")
            outputs = self.model.generate(inputs, max_length=50, pad_token_id=self.tokenizer.eos_token_id)
            
            rospy.loginfo("HuggingFace model loaded successfully")
            return True
            
        except ImportError:
            rospy.logwarn("Transformers package not installed")
            return False
        except Exception as e:
            rospy.logerr(f"Failed to initialize HuggingFace: {e}")
            return False
    
    def _initialize_context_memory(self):
        """Initialize context memory system."""
        if not self.enable_context_memory:
            return
        
        try:
            # Create context file if it doesn't exist
            context_file = Path(self.context_file_path)
            context_file.parent.mkdir(parents=True, exist_ok=True)
            
            if context_file.exists():
                with open(context_file, 'r') as f:
                    self.conversation_history = json.load(f)
                rospy.loginfo(f"Loaded {len(self.conversation_history)} conversation entries")
            else:
                self.conversation_history = []
                self._save_context_memory()
                rospy.loginfo("Created new context memory file")
                
        except Exception as e:
            rospy.logerr(f"Failed to initialize context memory: {e}")
            self.conversation_history = []
    
    def _save_context_memory(self):
        """Save conversation history to file."""
        if not self.enable_context_memory:
            return
        
        try:
            with open(self.context_file_path, 'w') as f:
                json.dump(self.conversation_history, f, indent=2)
        except Exception as e:
            rospy.logerr(f"Failed to save context memory: {e}")
    
    def process_user_intent(self, intent_msg):
        """Process user intent and generate robot action using LLM."""
        try:
            raw_text = intent_msg.raw_text
            confidence = intent_msg.confidence
            
            # Skip processing if confidence is too low
            if confidence < 0.3:
                rospy.logwarn(f"Low confidence input ignored: {raw_text} (confidence: {confidence})")
                return
            
            # Add to conversation history
            self._add_to_context(raw_text, intent_msg.intent_type, intent_msg.entities)
            
            # Generate LLM prompt with context
            prompt = self._create_prompt(raw_text, intent_msg.intent_type, intent_msg.entities)
            
            # Call LLM
            start_time = time.time()
            llm_response = self._call_llm(prompt)
            processing_time = time.time() - start_time
            
            # Parse LLM response into robot action
            robot_action = self._parse_llm_response(llm_response, raw_text)
            
            # Publish robot action
            self.action_publisher.publish(robot_action)
            
            # Publish LLM response for logging/debugging
            response_msg = LLMResponse()
            response_msg.original_text = raw_text
            response_msg.llm_response = llm_response
            response_msg.timestamp = rospy.Time.now()
            response_msg.processing_time = processing_time
            response_msg.model_used = f"{self.llm_type}_{self.llm_model}"
            self.response_publisher.publish(response_msg)
            
            rospy.loginfo(f"Generated action for: {raw_text} (processing time: {processing_time:.2f}s)")
            
        except Exception as e:
            rospy.logerr(f"Error processing user intent: {e}")
    
    def _add_to_context(self, text, intent_type, entities):
        """Add user input to conversation context."""
        with self.context_lock:
            entry = {
                'timestamp': datetime.now().isoformat(),
                'text': text,
                'intent_type': intent_type,
                'entities': entities,
                'user_input': True
            }
            
            self.conversation_history.append(entry)
            
            # Limit context length
            if len(self.conversation_history) > self.max_context_length:
                self.conversation_history = self.conversation_history[-self.max_context_length:]
            
            # Save to file
            self._save_context_memory()
    
    def _create_prompt(self, user_text, intent_type, entities):
        """Create a prompt for the LLM based on user input and context."""
        system_prompt = """
You are an AI assistant for a robot that helps nonverbal individuals.
Convert the user's natural language input into a structured robot action.

Available actions:
- fetch: Get an object for the user (parameters: object, location)
- move: Move to a location (parameters: destination, speed)
- speak: Generate speech output (parameters: message, volume)
- gesture: Perform a gesture (parameters: gesture_type, duration)
- wait: Wait for a specified time (parameters: duration_seconds)
- help: Provide assistance (parameters: assistance_type)

Respond with JSON format only:
{
    "action": "ACTION_TYPE",
    "parameters": {...},
    "confidence": 0.0-1.0,
    "explanation": "Brief explanation"
}

Examples:
- User: "I'm thirsty" → {"action": "fetch", "parameters": {"object": "water bottle"}, "confidence": 0.9, "explanation": "User needs water"}
- User: "go to kitchen" → {"action": "move", "parameters": {"destination": "kitchen"}, "confidence": 0.9, "explanation": "User wants to move to kitchen"}
- User: "pick up cup" → {"action": "fetch", "parameters": {"object": "cup"}, "confidence": 0.8, "explanation": "User wants the cup"}
"""
        
        # Add context if available
        context_text = ""
        if self.conversation_history and len(self.conversation_history) > 1:
            context_text = "\n\nRecent conversation context:\n"
            for entry in self.conversation_history[-3:]:  # Last 3 entries
                if entry.get('user_input'):
                    context_text += f"- User: {entry['text']}\n"
        
        # Add current input information
        current_input = f"""
Current input:
- Text: "{user_text}"
- Intent type: {intent_type}
- Entities: {entities}
"""
        
        return f"{system_prompt}{context_text}{current_input}\n\nResponse:"
    
    def _call_llm(self, prompt):
        """Call the appropriate LLM (OpenAI, HuggingFace, or mock)."""
        if self.llm_type == "openai":
            return self._call_openai(prompt)
        elif self.llm_type == "huggingface":
            return self._call_huggingface(prompt)
        else:
            return self._get_mock_response(prompt)
    
    def _call_openai(self, prompt):
        """Call OpenAI GPT-4 API."""
        for attempt in range(self.retry_attempts):
            try:
                response = self.openai_client.ChatCompletion.create(
                    model=self.llm_model,
                    messages=[
                        {"role": "system", "content": "You are a helpful AI assistant for a robot."},
                        {"role": "user", "content": prompt}
                    ],
                    max_tokens=self.max_tokens,
                    temperature=self.temperature,
                    timeout=self.api_timeout
                )
                
                if response.choices and response.choices[0].message.content:
                    return response.choices[0].message.content.strip()
                else:
                    rospy.logwarn(f"Empty response from OpenAI (attempt {attempt + 1})")
                    
            except Exception as e:
                rospy.logwarn(f"OpenAI API call failed (attempt {attempt + 1}): {e}")
                if attempt == self.retry_attempts - 1:
                    rospy.logerr("All OpenAI API attempts failed, using fallback")
                    return self._get_mock_response(prompt)
        
        return self._get_mock_response(prompt)
    
    def _call_huggingface(self, prompt):
        """Call HuggingFace Transformers model."""
        try:
            # Encode the prompt
            inputs = self.tokenizer.encode(prompt, return_tensors="pt", truncation=True, max_length=512)
            
            # Generate response
            outputs = self.model.generate(
                inputs,
                max_length=inputs.shape[1] + self.max_tokens,
                pad_token_id=self.tokenizer.eos_token_id,
                temperature=self.temperature,
                do_sample=True,
                top_p=0.9
            )
            
            # Decode response
            response = self.tokenizer.decode(outputs[0], skip_special_tokens=True)
            
            # Extract only the new part (after the prompt)
            if prompt in response:
                response = response.split(prompt)[-1].strip()
            
            return response
            
        except Exception as e:
            rospy.logerr(f"HuggingFace inference failed: {e}")
            return self._get_mock_response(prompt)
    
    def _get_mock_response(self, prompt):
        """Generate a mock response when LLM is not available."""
        # Extract user input from prompt
        user_input = ""
        if "Current input:" in prompt:
            lines = prompt.split("Current input:")[-1].split("\n")
            for line in lines:
                if 'Text:' in line:
                    user_input = line.split('"')[1] if '"' in line else line.split(":")[-1].strip()
                    break
        
        user_input = user_input.lower()
        
        # Generate structured response based on input
        if any(word in user_input for word in ["thirsty", "water", "drink"]):
            return json.dumps({
                "action": "fetch",
                "parameters": {"object": "water bottle", "location": "kitchen"},
                "confidence": 0.9,
                "explanation": "User is thirsty and needs water"
            })
        elif any(word in user_input for word in ["hungry", "food", "eat"]):
            return json.dumps({
                "action": "fetch",
                "parameters": {"object": "food", "location": "kitchen"},
                "confidence": 0.8,
                "explanation": "User is hungry and needs food"
            })
        elif any(word in user_input for word in ["go", "move", "walk", "kitchen"]):
            return json.dumps({
                "action": "move",
                "parameters": {"destination": "kitchen", "speed": "normal"},
                "confidence": 0.9,
                "explanation": "User wants to move to kitchen"
            })
        elif any(word in user_input for word in ["pick", "grab", "get", "cup"]):
            return json.dumps({
                "action": "fetch",
                "parameters": {"object": "cup", "location": "current"},
                "confidence": 0.8,
                "explanation": "User wants to get the cup"
            })
        elif any(word in user_input for word in ["help", "assist"]):
            return json.dumps({
                "action": "help",
                "parameters": {"assistance_type": "general"},
                "confidence": 0.7,
                "explanation": "User needs assistance"
            })
        else:
            return json.dumps({
                "action": "speak",
                "parameters": {"message": "I understand you said: " + user_input},
                "confidence": 0.5,
                "explanation": "General response to user input"
            })
    
    def _parse_llm_response(self, llm_response, original_text):
        """Parse LLM response into RobotAction message."""
        try:
            # Clean the response to extract JSON
            response_text = llm_response.strip()
            if response_text.startswith('```json'):
                response_text = response_text[7:]
            if response_text.endswith('```'):
                response_text = response_text[:-3]
            
            response_data = json.loads(response_text.strip())
            
            action_msg = RobotAction()
            action_msg.action_type = response_data.get("action", "UNKNOWN")
            action_msg.parameters = json.dumps(response_data.get("parameters", {}))
            action_msg.confidence = response_data.get("confidence", 0.0)
            action_msg.original_text = original_text
            action_msg.timestamp = rospy.Time.now()
            action_msg.status = "pending"
            
            return action_msg
            
        except json.JSONDecodeError as e:
            rospy.logerr(f"Failed to parse LLM response: {e}")
            rospy.logerr(f"Raw response: {llm_response}")
            # Return a default action
            action_msg = RobotAction()
            action_msg.action_type = "UNKNOWN"
            action_msg.parameters = "{}"
            action_msg.confidence = 0.0
            action_msg.original_text = original_text
            action_msg.timestamp = rospy.Time.now()
            action_msg.status = "failed"
            return action_msg
    
    def get_context_memory(self):
        """Get current conversation context."""
        with self.context_lock:
            return {
                'conversation_history': self.conversation_history,
                'context_length': len(self.conversation_history),
                'max_context_length': self.max_context_length,
                'context_file': self.context_file_path
            }
    
    def clear_context_memory(self):
        """Clear conversation context."""
        with self.context_lock:
            self.conversation_history = []
            self._save_context_memory()
            rospy.loginfo("Context memory cleared")
    
    def get_status(self):
        """Get current status of the LLM interpreter."""
        return {
            'llm_type': self.llm_type,
            'llm_model': self.llm_model,
            'context_memory_enabled': self.enable_context_memory,
            'context_length': len(self.conversation_history),
            'max_context_length': self.max_context_length
        }
    
    def run(self):
        """Main run loop."""
        rospy.loginfo("LLM Interpreter started - ready to process user intents...")
        rospy.spin()

if __name__ == '__main__':
    try:
        interpreter = LLMInterpreter()
        interpreter.run()
    except rospy.ROSInterruptException:
        pass 