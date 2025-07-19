#!/usr/bin/env python3
"""
FernAssist OpenAI API Setup Script

This script helps users set up and test their OpenAI API integration.
"""

import os
import sys
import getpass
import subprocess
from pathlib import Path

def check_python_dependencies():
    """Check if required Python packages are installed."""
    try:
        import openai
        print("✅ openai package is installed")
        return True
    except ImportError:
        print("❌ openai package is not installed")
        print("Installing required packages...")
        try:
            subprocess.check_call([sys.executable, "-m", "pip", "install", "openai"])
            print("✅ openai package installed successfully")
            return True
        except subprocess.CalledProcessError:
            print("❌ Failed to install openai package")
            return False

def get_api_key():
    """Get the OpenAI API key from user input."""
    print("\n🔑 Setting up OpenAI API Key")
    print("=" * 50)
    print("1. Visit: https://platform.openai.com/api-keys")
    print("2. Sign in with your OpenAI account")
    print("3. Click 'Create new secret key'")
    print("4. Copy the generated API key")
    print("=" * 50)
    
    api_key = getpass.getpass("Enter your OpenAI API key (input will be hidden): ").strip()
    
    if not api_key:
        print("❌ No API key provided")
        return None
    
    if len(api_key) < 20:
        print("❌ API key seems too short. Please check your input.")
        return None
    
    return api_key

def test_api_key(api_key):
    """Test the API key with a simple request."""
    try:
        import openai
        
        # Configure the API
        openai.api_key = api_key
        
        # Test with a simple request (using cheaper model)
        response = openai.ChatCompletion.create(
            model="gpt-3.5-turbo",
            messages=[{"role": "user", "content": "Say 'Hello from FernAssist!'"}],
            max_tokens=20
        )
        
        if response.choices and response.choices[0].message.content:
            print("✅ API key is valid and working!")
            print(f"Test response: {response.choices[0].message.content}")
            return True
        else:
            print("❌ API key test failed - no response received")
            return False
            
    except Exception as e:
        print(f"❌ API key test failed: {e}")
        return False

def save_api_key(api_key):
    """Save the API key to environment file."""
    # Create .env file
    env_file = Path(".env")
    
    # Read existing content
    env_content = ""
    if env_file.exists():
        with open(env_file, 'r') as f:
            env_content = f.read()
    
    # Update or add OPENAI_API_KEY
    lines = env_content.split('\n')
    new_lines = []
    key_found = False
    
    for line in lines:
        if line.startswith('OPENAI_API_KEY='):
            new_lines.append(f'OPENAI_API_KEY={api_key}')
            key_found = True
        else:
            new_lines.append(line)
    
    if not key_found:
        new_lines.append(f'OPENAI_API_KEY={api_key}')
    
    # Write back to file
    with open(env_file, 'w') as f:
        f.write('\n'.join(new_lines))
    
    print("✅ API key saved to .env file")
    
    # Also add to bashrc for convenience
    bashrc_path = Path.home() / ".bashrc"
    bashrc_content = f'\n# FernAssist OpenAI API Key\nexport OPENAI_API_KEY="{api_key}"\n'
    
    with open(bashrc_path, 'a') as f:
        f.write(bashrc_content)
    
    print("✅ API key added to ~/.bashrc for automatic loading")

def create_test_script():
    """Create a simple test script for users."""
    test_script = """#!/usr/bin/env python3
\"\"\"
Simple test script for FernAssist OpenAI integration
\"\"\"

import os
import openai

def test_fernassist_integration():
    # Load API key
    api_key = os.getenv('OPENAI_API_KEY')
    if not api_key:
        print("❌ OPENAI_API_KEY not found in environment variables")
        return False
    
    try:
        # Configure OpenAI
        openai.api_key = api_key
        
        # Test with a FernAssist-style prompt
        prompt = '''
        You are an AI assistant for a robot that helps nonverbal individuals.
        Convert the user's natural language input into a structured robot action.
        
        Available actions: fetch, move, speak, gesture, wait, help
        
        User input: "I'm thirsty"
        
        Respond with JSON format:
        {
            "action": "ACTION_TYPE",
            "parameters": {...},
            "confidence": 0.0-1.0,
            "explanation": "Brief explanation"
        }
        '''
        
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
            print("✅ FernAssist integration test successful!")
            print(f"Response: {response.choices[0].message.content}")
            return True
        else:
            print("❌ No response received from OpenAI")
            return False
        
    except Exception as e:
        print(f"❌ Integration test failed: {e}")
        return False

if __name__ == "__main__":
    test_fernassist_integration()
"""
    
    with open("test_openai_integration.py", "w") as f:
        f.write(test_script)
    
    # Make it executable
    os.chmod("test_openai_integration.py", 0o755)
    print("✅ Created test script: test_openai_integration.py")

def show_cost_information():
    """Show cost information for OpenAI API."""
    print("\n💰 OpenAI API Cost Information")
    print("=" * 40)
    print("GPT-4 Pricing (as of 2024):")
    print("- Input: $0.03 per 1K tokens")
    print("- Output: $0.06 per 1K tokens")
    print()
    print("GPT-3.5-turbo Pricing:")
    print("- Input: $0.0015 per 1K tokens")
    print("- Output: $0.002 per 1K tokens")
    print()
    print("💡 Tips:")
    print("- GPT-3.5-turbo is much cheaper for testing")
    print("- Set usage limits in your OpenAI dashboard")
    print("- Monitor usage regularly")

def main():
    """Main setup function."""
    print("🤖 FernAssist OpenAI API Setup")
    print("=" * 40)
    
    # Check dependencies
    if not check_python_dependencies():
        print("❌ Setup failed due to missing dependencies")
        return False
    
    # Get API key
    api_key = get_api_key()
    if not api_key:
        print("❌ Setup failed - no valid API key provided")
        return False
    
    # Test API key
    print("\n🧪 Testing API key...")
    if not test_api_key(api_key):
        print("❌ Setup failed - API key test failed")
        return False
    
    # Save API key
    print("\n💾 Saving API key...")
    save_api_key(api_key)
    
    # Create test script
    print("\n📝 Creating test script...")
    create_test_script()
    
    # Show cost information
    show_cost_information()
    
    print("\n🎉 Setup completed successfully!")
    print("\nNext steps:")
    print("1. Source your bashrc: source ~/.bashrc")
    print("2. Test the integration: python3 test_openai_integration.py")
    print("3. Launch FernAssist: ros2 launch fernassist fernassist.launch.py")
    
    return True

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1) 