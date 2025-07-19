# FernAssist: AI-powered Assistive Robotics

**FernAssist** is an innovative AI-powered assistive robotics system designed to help nonverbal individuals communicate and interact with their environment through Augmentative and Alternative Communication (AAC) devices, Google's Gemini Large Language Model, and ROS 2.

## 🌟 Overview

FernAssist bridges the gap between AAC devices and robotic assistance, enabling nonverbal individuals to:
- **Communicate** through natural language processing using Google's Gemini AI
- **Control** robots using intuitive AAC inputs
- **Navigate** their environment safely and independently
- **Interact** with objects and perform daily tasks
- **Express** themselves through speech synthesis and gestures

## 🏗️ System Architecture

The FernAssist system consists of four core components:

1. **AAC Listener** - Interfaces with AAC devices and processes user inputs
2. **LLM Interpreter** - Uses Google's Gemini AI to understand and interpret user intentions
3. **Robot Controller** - Executes robot actions based on AI interpretations
4. **System Monitor** - Ensures system health and provides real-time feedback

## 📁 Project Structure

```
FernAssist/
├── scripts/                    # Python scripts for core components
│   ├── aac_listener.py        # AAC device communication
│   ├── llm_interpreter.py     # Gemini AI-based natural language processing
│   ├── robot_controller.py    # Robot action execution
│   └── system_monitor.py      # System health monitoring
├── ros2_ws/                   # ROS 2 workspace
│   └── src/fernassist/        # ROS 2 package
│       ├── msg/               # Message definitions
│       ├── srv/               # Service definitions
│       ├── action/            # Action definitions
│       ├── launch/            # Launch files
│       ├── config/            # Configuration files
│       └── scripts/           # ROS 2 node scripts
├── demo/                      # Screenshots, GIFs, and demonstrations
├── logs/                      # Weekly build logs and system logs
├── docs/                      # Documentation and architecture diagrams
└── requirements.txt           # Python dependencies
```

## 🚀 Quick Start

### Prerequisites

- **ROS 2** (Humble or later)
- **Python 3.8+**
- **Google Gemini API Access** (Free tier available)

### Setting Up Google Gemini API

1. **Get a free API key:**
   - Visit [Google AI Studio](https://makersuite.google.com/app/apikey)
   - Sign in with your Google account
   - Create a new API key (free tier includes generous usage limits)

2. **Set up environment variable:**
   ```bash
   export GEMINI_API_KEY="your-api-key-here"
   ```
   
   Or add to your `.bashrc`:
   ```bash
   echo 'export GEMINI_API_KEY="your-api-key-here"' >> ~/.bashrc
   source ~/.bashrc
   ```

### Installation

1. **Clone the repository:**
   ```bash
   git clone https://github.com/your-org/fernassist.git
   cd fernassist
   ```

2. **Install Python dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

3. **Build the ROS 2 workspace:**
   ```bash
   cd ros2_ws
   colcon build
   source install/setup.bash
   ```

4. **Launch the system:**
   ```bash
   ros2 launch fernassist fernassist.launch.py
   ```

## 🔧 Configuration

Configure FernAssist through the YAML configuration file:

```yaml
# ros2_ws/src/fernassist/config/fernassist_config.yaml
llm_interpreter:
  ros__parameters:
    llm_model: "gemini-pro"
    max_tokens: 150
    temperature: 0.7
    api_timeout: 30.0
    retry_attempts: 3

aac_listener:
  ros__parameters:
    confidence_threshold: 0.5
    input_timeout: 10.0
```

## 📡 ROS 2 Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| `/aac/input` | `AACInput` | Raw AAC device inputs |
| `/fernassist/user_intent` | `UserIntent` | Processed user intentions |
| `/fernassist/robot_action` | `RobotAction` | Robot action commands |
| `/fernassist/llm_response` | `LLMResponse` | Gemini AI processing results |
| `/fernassist/system_status` | `String` | System health status |
| `/cmd_vel` | `Twist` | Robot movement commands |
| `/speech/output` | `String` | Speech synthesis commands |

## 🛡️ Safety Features

- **Confidence Thresholds**: Actions only execute when confidence levels meet minimum thresholds
- **Emergency Stop**: Immediate halt capability for safety
- **Timeout Protection**: Automatic timeout for long-running operations
- **Collision Detection**: Obstacle avoidance and safe zone monitoring
- **Component Health Monitoring**: Continuous monitoring of all system components

## 🤖 Supported Actions

FernAssist supports various robot actions:

- **Movement**: Navigate to locations, follow paths
- **Manipulation**: Pick up, place, and manipulate objects
- **Communication**: Speech synthesis and text-to-speech
- **Gestures**: Non-verbal communication through robot gestures
- **Environmental Interaction**: Turn on/off lights, open doors, etc.

## 📊 Monitoring and Logging

The system provides comprehensive monitoring:

- **Real-time Health Checks**: Component status and connectivity
- **Performance Metrics**: Processing times and success rates
- **Error Tracking**: Detailed error logs and recovery procedures
- **Usage Statistics**: Input patterns and system utilization

## 🧪 Testing

Run the test suite:

```bash
# Unit tests
pytest scripts/tests/

# ROS 2 tests
cd ros2_ws
colcon test --packages-select fernassist
```

## 📚 Documentation

- [Architecture Guide](docs/architecture.md) - Detailed system design
- [API Reference](docs/api.md) - Message and service definitions
- [Deployment Guide](docs/deployment.md) - Production deployment instructions
- [Troubleshooting](docs/troubleshooting.md) - Common issues and solutions

## 🤝 Contributing

We welcome contributions! Please see our [Contributing Guidelines](CONTRIBUTING.md) for details.

### Development Setup

1. Fork the repository
2. Create a feature branch: `git checkout -b feature/amazing-feature`
3. Make your changes and add tests
4. Run the test suite: `pytest`
5. Submit a pull request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- **Google AI** for providing the Gemini API and free tier access
- **ROS 2 Community** for the robotics framework
- **AAC Device Manufacturers** for communication interfaces
- **Assistive Technology Community** for insights and feedback

## 📞 Support

- **Issues**: [GitHub Issues](https://github.com/your-org/fernassist/issues)
- **Discussions**: [GitHub Discussions](https://github.com/your-org/fernassist/discussions)
- **Email**: fernassist@example.com

## 🔮 Roadmap

- [ ] **Multi-language Support**: Internationalization for global accessibility
- [ ] **Advanced Gesture Recognition**: Computer vision-based gesture understanding
- [ ] **Emotional Intelligence**: Emotion-aware responses and interactions
- [ ] **Mobile App**: Companion mobile application for caregivers
- [ ] **Cloud Integration**: Remote monitoring and updates
- [ ] **Custom AAC Support**: Integration with more AAC devices and protocols

## 💰 Cost Information

FernAssist uses Google's Gemini API which offers:
- **Free Tier**: 15 requests per minute, 1500 requests per day
- **Paid Tier**: $0.00025 per 1K characters (input) + $0.0005 per 1K characters (output)
- **No Credit Card Required**: Free tier works without payment information

This makes FernAssist accessible for development, testing, and small-scale deployments without any cost.

---

**FernAssist** - Empowering nonverbal individuals through AI-powered robotics 🤖💙
