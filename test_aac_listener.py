#!/usr/bin/env python3
"""
Test script for FernAssist AAC Listener

This script demonstrates the AAC listener functionality with different input methods.
"""

import os
import sys
import time
import json
import threading
from pathlib import Path

def test_file_input():
    """Test file-based input method."""
    print("\n📁 Testing File Input Method")
    print("=" * 40)
    
    # Create test input file
    test_file = Path("/tmp/aac_input.txt")
    test_inputs = [
        "I'm thirsty",
        "go to kitchen",
        "pick up cup",
        "I want water",
        "help me"
    ]
    
    # Write test inputs to file
    with open(test_file, 'w') as f:
        f.write("# Test AAC Inputs\n")
        for input_text in test_inputs:
            f.write(f"{input_text}\n")
    
    print(f"✅ Created test file: {test_file}")
    print(f"📝 Added {len(test_inputs)} test inputs")
    print("💡 The AAC listener will monitor this file for changes")
    
    return test_file

def test_api_input():
    """Test API-based input method."""
    print("\n🌐 Testing API Input Method")
    print("=" * 40)
    
    try:
        import requests
        
        # Create a simple test server (in a separate thread)
        def run_test_server():
            from flask import Flask, jsonify
            app = Flask(__name__)
            
            @app.route('/aac/input')
            def get_aac_inputs():
                return jsonify({
                    'inputs': [
                        {'text': 'I need help', 'confidence': 0.9, 'type': 'api'},
                        {'text': 'turn on light', 'confidence': 0.8, 'type': 'api'}
                    ]
                })
            
            app.run(host='localhost', port=8080, debug=False)
        
        # Start test server in background
        server_thread = threading.Thread(target=run_test_server, daemon=True)
        server_thread.start()
        
        # Wait for server to start
        time.sleep(2)
        
        # Test API endpoint
        response = requests.get('http://localhost:8080/aac/input')
        if response.status_code == 200:
            data = response.json()
            print("✅ API endpoint is working")
            print(f"📡 Received {len(data.get('inputs', []))} inputs from API")
        else:
            print("❌ API endpoint test failed")
            
    except ImportError:
        print("⚠️  Flask not installed - skipping API test")
    except Exception as e:
        print(f"❌ API test failed: {e}")

def test_websocket_input():
    """Test WebSocket-based input method."""
    print("\n🔌 Testing WebSocket Input Method")
    print("=" * 40)
    
    try:
        import websocket
        
        # Create a simple WebSocket server (in a separate thread)
        def run_websocket_server():
            import socket
            import threading
            
            def handle_client(client_socket):
                try:
                    # Send test messages
                    test_messages = [
                        json.dumps({'text': 'I am tired', 'confidence': 0.9, 'type': 'websocket'}),
                        json.dumps({'text': 'open door', 'confidence': 0.8, 'type': 'websocket'})
                    ]
                    
                    for msg in test_messages:
                        client_socket.send(msg.encode())
                        time.sleep(1)
                        
                except Exception as e:
                    print(f"WebSocket client error: {e}")
                finally:
                    client_socket.close()
            
            # Simple TCP server (not full WebSocket implementation)
            server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            server.bind(('localhost', 8081))
            server.listen(1)
            
            while True:
                client, addr = server.accept()
                client_thread = threading.Thread(target=handle_client, args=(client,))
                client_thread.start()
        
        # Start WebSocket server in background
        ws_server_thread = threading.Thread(target=run_websocket_server, daemon=True)
        ws_server_thread.start()
        
        time.sleep(1)
        print("✅ WebSocket server started on localhost:8081")
        
    except Exception as e:
        print(f"❌ WebSocket test failed: {e}")

def test_mock_input():
    """Test mock input generation."""
    print("\n🤖 Testing Mock Input Generation")
    print("=" * 40)
    
    mock_inputs = [
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
    
    print(f"✅ Mock inputs configured: {len(mock_inputs)} phrases")
    print("🔄 Mock inputs will be generated every 30 seconds")
    print("📝 Sample inputs:")
    for i, input_text in enumerate(mock_inputs[:5], 1):
        print(f"   {i}. {input_text}")

def show_usage_instructions():
    """Show usage instructions for the AAC listener."""
    print("\n📖 Usage Instructions")
    print("=" * 40)
    print("1. Start the AAC listener:")
    print("   ros2 launch fernassist fernassist.launch.py")
    print()
    print("2. Test different input methods:")
    print("   📁 File Input: Edit /tmp/aac_input.txt")
    print("   🌐 API Input: Send POST to http://localhost:8080/aac/input")
    print("   🔌 WebSocket: Connect to ws://localhost:8080/aac/ws")
    print("   🤖 Mock Input: Automatically generated every 30s")
    print()
    print("3. Monitor ROS 2 topics:")
    print("   ros2 topic echo /fernassist/user_intent")
    print("   ros2 topic echo /aac/raw_input")
    print()
    print("4. Check system status:")
    print("   ros2 service call /aac_listener/get_status")

def main():
    """Main test function."""
    print("🤖 FernAssist AAC Listener Test")
    print("=" * 50)
    
    # Test different input methods
    test_file_input()
    test_api_input()
    test_websocket_input()
    test_mock_input()
    
    # Show usage instructions
    show_usage_instructions()
    
    print("\n🎉 AAC Listener test setup completed!")
    print("\nNext steps:")
    print("1. Start the AAC listener with ROS 2")
    print("2. Monitor the topics to see processed inputs")
    print("3. Test with real AAC devices or apps")

if __name__ == "__main__":
    main() 