#!/usr/bin/env python3
"""
Test script for FernAssist LLM Interpreter

This script demonstrates the LLM interpreter functionality with different input examples
and shows how context memory works.
"""

import os
import sys
import json
import time
from pathlib import Path

def test_openai_integration():
    """Test OpenAI GPT-4 integration."""
    print("\n🤖 Testing OpenAI GPT-4 Integration")
    print("=" * 50)
    
    api_key = os.getenv('OPENAI_API_KEY')
    if not api_key:
        print("❌ OPENAI_API_KEY not found in environment variables")
        print("💡 Run: python3 setup_openai.py")
        return False
    
    try:
        import openai
        openai.api_key = api_key
        
        # Test with FernAssist prompt
        test_inputs = [
            "I'm thirsty",
            "go to kitchen",
            "pick up cup",
            "I want water",
            "help me"
        ]
        
        print("🧪 Testing with sample inputs:")
        for i, user_input in enumerate(test_inputs, 1):
            print(f"\n{i}. Input: '{user_input}'")
            
            prompt = create_fernassist_prompt(user_input, "general", [])
            
            response = openai.ChatCompletion.create(
                model="gpt-4",
                messages=[
                    {"role": "system", "content": "You are a helpful AI assistant for a robot."},
                    {"role": "user", "content": prompt}
                ],
                max_tokens=200,
                temperature=0.7
            )
            
            if response.choices and response.choices[0].message.content:
                result = response.choices[0].message.content.strip()
                print(f"   Response: {result}")
                
                # Try to parse JSON
                try:
                    parsed = json.loads(result)
                    print(f"   ✅ Parsed: {parsed.get('action', 'unknown')} action")
                except:
                    print(f"   ⚠️  Could not parse as JSON")
            else:
                print("   ❌ No response received")
        
        return True
        
    except Exception as e:
        print(f"❌ OpenAI test failed: {e}")
        return False

def test_huggingface_fallback():
    """Test HuggingFace Transformers fallback."""
    print("\n🤗 Testing HuggingFace Transformers Fallback")
    print("=" * 50)
    
    try:
        from transformers import AutoTokenizer, AutoModelForCausalLM
        import torch
        
        # Use a smaller model for testing
        model_name = "microsoft/DialoGPT-small"
        
        print(f"📥 Loading model: {model_name}")
        tokenizer = AutoTokenizer.from_pretrained(model_name)
        model = AutoModelForCausalLM.from_pretrained(model_name)
        
        # Test with simple input
        test_input = "I'm thirsty"
        print(f"🧪 Testing input: '{test_input}'")
        
        inputs = tokenizer.encode(test_input, return_tensors="pt")
        outputs = model.generate(
            inputs,
            max_length=inputs.shape[1] + 50,
            pad_token_id=tokenizer.eos_token_id,
            temperature=0.7,
            do_sample=True
        )
        
        response = tokenizer.decode(outputs[0], skip_special_tokens=True)
        print(f"   Response: {response}")
        print("   ✅ HuggingFace model working")
        
        return True
        
    except ImportError:
        print("❌ Transformers package not installed")
        return False
    except Exception as e:
        print(f"❌ HuggingFace test failed: {e}")
        return False

def test_context_memory():
    """Test context memory functionality."""
    print("\n🧠 Testing Context Memory")
    print("=" * 50)
    
    context_file = Path("/tmp/fernassist_context.json")
    
    # Create sample conversation history
    sample_history = [
        {
            "timestamp": "2024-01-01T10:00:00",
            "text": "I'm thirsty",
            "intent_type": "need_drink",
            "entities": ["water"],
            "user_input": True
        },
        {
            "timestamp": "2024-01-01T10:01:00",
            "text": "go to kitchen",
            "intent_type": "movement",
            "entities": ["kitchen"],
            "user_input": True
        },
        {
            "timestamp": "2024-01-01T10:02:00",
            "text": "pick up cup",
            "intent_type": "manipulation",
            "entities": ["cup"],
            "user_input": True
        }
    ]
    
    # Save sample context
    with open(context_file, 'w') as f:
        json.dump(sample_history, f, indent=2)
    
    print(f"✅ Created sample context file: {context_file}")
    print(f"📝 Sample conversation history:")
    for i, entry in enumerate(sample_history, 1):
        print(f"   {i}. {entry['text']} → {entry['intent_type']}")
    
    # Test context loading
    try:
        with open(context_file, 'r') as f:
            loaded_history = json.load(f)
        print(f"✅ Successfully loaded {len(loaded_history)} conversation entries")
        return True
    except Exception as e:
        print(f"❌ Failed to load context: {e}")
        return False

def test_structured_output():
    """Test structured command output parsing."""
    print("\n📋 Testing Structured Command Output")
    print("=" * 50)
    
    test_cases = [
        {
            "input": "I'm thirsty",
            "expected": {"action": "fetch", "object": "water bottle"}
        },
        {
            "input": "go to kitchen",
            "expected": {"action": "move", "destination": "kitchen"}
        },
        {
            "input": "pick up cup",
            "expected": {"action": "fetch", "object": "cup"}
        },
        {
            "input": "I want water",
            "expected": {"action": "fetch", "object": "water"}
        },
        {
            "input": "help me",
            "expected": {"action": "help", "assistance_type": "general"}
        }
    ]
    
    print("🧪 Testing input → output mapping:")
    for test_case in test_cases:
        user_input = test_case["input"]
        expected = test_case["expected"]
        
        print(f"\nInput: '{user_input}'")
        print(f"Expected: {expected}")
        
        # Generate mock response
        mock_response = generate_mock_response(user_input)
        print(f"Mock Output: {mock_response}")
        
        # Try to parse
        try:
            parsed = json.loads(mock_response)
            action = parsed.get("action", "unknown")
            parameters = parsed.get("parameters", {})
            print(f"✅ Parsed: {action} with parameters {parameters}")
        except:
            print("❌ Failed to parse as JSON")

def create_fernassist_prompt(user_text, intent_type, entities):
    """Create a FernAssist-style prompt."""
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
"""
    
    current_input = f"""
Current input:
- Text: "{user_text}"
- Intent type: {intent_type}
- Entities: {entities}
"""
    
    return f"{system_prompt}{current_input}\n\nResponse:"

def generate_mock_response(user_input):
    """Generate a mock response for testing."""
    user_input = user_input.lower()
    
    if any(word in user_input for word in ["thirsty", "water", "drink"]):
        return json.dumps({
            "action": "fetch",
            "parameters": {"object": "water bottle", "location": "kitchen"},
            "confidence": 0.9,
            "explanation": "User is thirsty and needs water"
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

def show_usage_instructions():
    """Show usage instructions for the LLM interpreter."""
    print("\n📖 Usage Instructions")
    print("=" * 50)
    print("1. Set up OpenAI API (recommended):")
    print("   python3 setup_openai.py")
    print()
    print("2. Set up HuggingFace (fallback):")
    print("   pip install transformers torch")
    print()
    print("3. Test the integration:")
    print("   python3 test_openai_integration.py")
    print()
    print("4. Monitor context memory:")
    print("   cat /tmp/fernassist_context.json")
    print()
    print("5. Launch FernAssist:")
    print("   ros2 launch fernassist fernassist.launch.py")

def main():
    """Main test function."""
    print("🤖 FernAssist LLM Interpreter Test")
    print("=" * 60)
    
    # Test different components
    openai_success = test_openai_integration()
    huggingface_success = test_huggingface_fallback()
    context_success = test_context_memory()
    structured_success = test_structured_output()
    
    # Show usage instructions
    show_usage_instructions()
    
    print("\n🎉 LLM Interpreter test completed!")
    print(f"\nResults:")
    print(f"  OpenAI GPT-4: {'✅' if openai_success else '❌'}")
    print(f"  HuggingFace: {'✅' if huggingface_success else '❌'}")
    print(f"  Context Memory: {'✅' if context_success else '❌'}")
    print(f"  Structured Output: {'✅' if structured_success else '❌'}")

if __name__ == "__main__":
    main() 