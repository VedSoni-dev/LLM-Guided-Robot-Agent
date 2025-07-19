# FernAssist Architecture

## Overview

FernAssist is an AI-powered assistive robotics system designed to help nonverbal individuals communicate and interact with their environment through Augmentative and Alternative Communication (AAC) devices, Google's Gemini Large Language Model, and ROS 2.

## System Architecture

### Core Components

1. **AAC Listener** (`scripts/aac_listener.py`)
   - Interfaces with AAC devices (tablets, speech-generating devices, etc.)
   - Processes raw AAC inputs and converts them to standardized messages
   - Publishes `UserIntent` messages for further processing

2. **LLM Interpreter** (`scripts/llm_interpreter.py`)
   - Receives `UserIntent` messages from the AAC Listener
   - Uses Google's Gemini AI to interpret natural language commands
   - Generates structured `RobotAction` messages
   - Handles context awareness and intent clarification
   - Includes fallback mock responses when API is unavailable

3. **Robot Controller** (`scripts/robot_controller.py`)
   - Executes robot actions based on LLM interpretations
   - Manages movement, manipulation, speech, and gesture commands
   - Implements safety protocols and emergency stops
   - Provides feedback on action completion

4. **System Monitor** (`scripts/system_monitor.py`)
   - Monitors health and status of all system components
   - Provides real-time feedback and error handling
   - Tracks system performance metrics
   - Manages component timeouts and recovery

### Communication Flow

```
AAC Device → AAC Listener → UserIntent → LLM Interpreter → RobotAction → Robot Controller → Robot
                ↓              ↓              ↓              ↓
            System Monitor ← System Status ← Health Check ← Component Status
```

### Message Types

- **AACInput**: Raw input from AAC devices
- **UserIntent**: Processed user intentions
- **RobotAction**: Structured robot commands
- **LLMResponse**: Gemini AI processing results
- **SystemStatus**: Overall system health

### ROS 2 Topics

- `/aac/input` - AAC device inputs
- `/fernassist/user_intent` - Processed user intentions
- `/fernassist/robot_action` - Robot action commands
- `/fernassist/llm_response` - Gemini AI processing results
- `/fernassist/system_status` - System health status
- `/cmd_vel` - Robot movement commands
- `/speech/output` - Speech synthesis commands
- `/gesture/command` - Gesture execution commands

## Gemini AI Integration

### API Configuration
- **Model**: `gemini-pro` (default)
- **Free Tier**: 15 requests/minute, 1500 requests/day
- **Authentication**: Environment variable `GEMINI_API_KEY`
- **Fallback**: Mock responses when API unavailable

### Prompt Engineering
The system uses carefully crafted prompts to:
- Convert natural language to structured robot actions
- Handle ambiguous inputs gracefully
- Provide confidence scores for actions
- Generate explanations for transparency

### Error Handling
- **Retry Logic**: Multiple attempts for failed API calls
- **Timeout Protection**: 30-second timeout for API requests
- **Graceful Degradation**: Mock responses when API fails
- **Logging**: Comprehensive error tracking and debugging

## Safety Features

1. **Confidence Thresholds**: Actions are only executed when confidence levels meet minimum thresholds
2. **Emergency Stop**: Immediate halt capability for safety
3. **Timeout Protection**: Automatic timeout for long-running operations
4. **Collision Detection**: Obstacle avoidance and safe zone monitoring
5. **Component Health Monitoring**: Continuous monitoring of all system components

## Configuration

The system is configured through YAML files in `ros2_ws/src/fernassist/config/`:
- Gemini model selection and parameters
- Safety thresholds and timeouts
- Component-specific settings
- Logging and monitoring preferences

## Deployment

The system is deployed using ROS 2 launch files:
- `fernassist.launch.py` - Main system launch file
- Component-specific launch files for individual testing
- Simulation and real robot deployment configurations

## Cost Considerations

### Free Tier Benefits
- **No Credit Card Required**: Free tier works without payment information
- **Generous Limits**: 15 requests/minute, 1500 requests/day
- **Production Ready**: Suitable for development and small-scale deployments

### Scaling Options
- **Paid Tier**: $0.00025 per 1K characters (input) + $0.0005 per 1K characters (output)
- **Enterprise**: Custom pricing for large-scale deployments
- **Self-Hosted**: Option to run local models for complete privacy 