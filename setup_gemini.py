#!/usr/bin/env python3
"""
FernAssist Gemini API Setup Script

This script helps users set up and test their Google Gemini API integration.
"""

import os
import sys
import getpass
import subprocess
from pathlib import Path

def check_python_dependencies():
    """Check if required Python packages are installed."""
    try:
        import google.generativeai
        print("✅ google-generativeai package is installed")
        return True
    except ImportError:
        print("❌ google-generativeai package is not installed")
        print("Installing required packages...")
        try:
            subprocess.check_call([sys.executable, "-m", "pip", "install", "google-generativeai"])
            print("✅ google-generativeai package installed successfully")
            return True
        except subprocess.CalledProcessError:
            print("❌ Failed to install google-generativeai package")
            return False

def get_api_key():
    """Get the Gemini API key from user input."""
    print("\n🔑 Setting up Google Gemini API Key")
    print("=" * 50)
    print("1. Visit: https://makersuite.google.com/app/apikey")
    print("2. Sign in with your Google account")
    print("3. Click 'Create API Key'")
    print("4. Copy the generated API key")
    print("=" * 50)
    
    api_key = getpass.getpass("Enter your Gemini API key (input will be hidden): ").strip()
    
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
        import google.generativeai as genai
        
        # Configure the API
        genai.configure(api_key=api_key)
        
        # Test with a simple request
        model = genai.GenerativeModel('gemini-pro')
        response = model.generate_content("Say 'Hello from FernAssist!'")
        
        if response.text:
            print("✅ API key is valid and working!")
            print(f"Test response: {response.text}")
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
    
    # Update or add GEMINI_API_KEY
    lines = env_content.split('\n')
    new_lines = []
    key_found = False
    
    for line in lines:
        if line.startswith('GEMINI_API_KEY='):
            new_lines.append(f'GEMINI_API_KEY={api_key}')
            key_found = True
        else:
            new_lines.append(line)
    
    if not key_found:
        new_lines.append(f'GEMINI_API_KEY={api_key}')
    
    # Write back to file
    with open(env_file, 'w') as f:
        f.write('\n'.join(new_lines))
    
    print("✅ API key saved to .env file")
    
    # Also add to bashrc for convenience
    bashrc_path = Path.home() / ".bashrc"
    bashrc_content = f'\n# FernAssist Gemini API Key\nexport GEMINI_API_KEY="{api_key}"\n'
    
    with open(bashrc_path, 'a') as f:
        f.write(bashrc_content)
    
    print("✅ API key added to ~/.bashrc for automatic loading")

def create_test_script():
    """Create a simple test script for users."""
    test_script = """#!/usr/bin/env python3
\"\"\"
Simple test script for FernAssist Gemini integration
\"\"\"

import os
import google.generativeai as genai

def test_fernassist_integration():
    # Load API key
    api_key = os.getenv('GEMINI_API_KEY')
    if not api_key:
        print("❌ GEMINI_API_KEY not found in environment variables")
        return False
    
    try:
        # Configure Gemini
        genai.configure(api_key=api_key)
        model = genai.GenerativeModel('gemini-pro')
        
        # Test with a FernAssist-style prompt
        prompt = '''
        You are an AI assistant for a robot that helps nonverbal individuals.
        Convert the user's natural language input into a structured robot action.
        
        Available actions: MOVE, GRAB, PLACE, SPEAK, GESTURE
        
        User input: "go to the kitchen"
        
        Respond with JSON format:
        {
            "action": "ACTION_TYPE",
            "parameters": {...},
            "confidence": 0.0-1.0,
            "explanation": "Brief explanation"
        }
        '''
        
        response = model.generate_content(prompt)
        print("✅ FernAssist integration test successful!")
        print(f"Response: {response.text}")
        return True
        
    except Exception as e:
        print(f"❌ Integration test failed: {e}")
        return False

if __name__ == "__main__":
    test_fernassist_integration()
"""
    
    with open("test_gemini_integration.py", "w") as f:
        f.write(test_script)
    
    # Make it executable
    os.chmod("test_gemini_integration.py", 0o755)
    print("✅ Created test script: test_gemini_integration.py")

def main():
    """Main setup function."""
    print("🤖 FernAssist Gemini API Setup")
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
    
    print("\n🎉 Setup completed successfully!")
    print("\nNext steps:")
    print("1. Source your bashrc: source ~/.bashrc")
    print("2. Test the integration: python3 test_gemini_integration.py")
    print("3. Launch FernAssist: ros2 launch fernassist fernassist.launch.py")
    
    return True

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1) 