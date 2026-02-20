# MicroVIT - Autonomous Multi-Robot System

## Overview

MicroVIT is a production-ready, multi-device robotics system featuring:
- **Robot1**: Jetson Nano (ROS1 master) + Jetson Orin (ROS2 compute) with MicroViT vision processing
- **Controller**: Raspberry Pi for message analysis and command routing
- **Helper Robot**: Task execution robot

Each component runs independent local AI using Ollama with lightweight models.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    MQTT BROKER (Central)                    │
└─────────────────────────────────────────────────────────────┘
         ▲                    ▲                    ▲
         │                    │                    │
    ┌────┴────┐          ┌────┴────┐          ┌────┴────┐
    │ ROBOT1  │          │CONTROLLER│         │ HELPER │
    │         │          │         │         │  ROBOT  │
    │ Nano    │◄──XML-RPC──►│  (RPi)  │         │         │
    │ (ROS1)  │          │         │         │         │
    │         │          │         │         │         │
    │ Orin    │          │         │         │         │
    │ (ROS2)  │          │         │         │         │
    └─────────┘          └─────────┘         └─────────┘
```

## Quick Start

### Prerequisites

- **Nano**: Jetson Nano with ROS Melodic, USB camera, LiDAR (optional)
- **Orin**: Jetson Orin with ROS Humble, Ollama installed
- **Controller**: Raspberry Pi with Ollama and MQTT broker
- **Helper Robot**: Jetson Orin (or similar) with Ollama

### Network Setup

Default IPs (update in config files):
- Nano: `10.13.68.184`
- Orin: `10.13.68.159`
- Controller: `10.13.68.48`
- Helper Robot: `10.13.68.XXX` (update if using)

### Installation Steps

#### 1. Nano (ROS1 Master)

```bash
# SSH to Nano
ssh jetbot@10.13.68.184

# Copy MicroVIT folder to Nano
scp -r MicroVIT/robot1/nano_ros1_master jetbot@10.13.68.184:~/MicroVIT/robot1/

# Build ROS workspace
cd ~/MicroVIT/robot1/nano_ros1_master
source /opt/ros/melodic/setup.bash
catkin_make
source devel/setup.bash

# Configure camera device (if different from /dev/video0)
# Edit: config/device_config.yaml

# Run in development mode
./scripts/run_dev.sh
```

#### 2. Orin (ROS2 Compute)

```bash
# SSH to Orin
ssh jetbot@10.13.68.159

# Copy MicroVIT folder to Orin
scp -r MicroVIT/robot1/orin_ros2_compute jetbot@10.13.68.159:~/MicroVIT/robot1/
scp -r MicroVIT/common jetbot@10.13.68.159:~/MicroVIT/

# Setup Python environment
cd ~/MicroVIT/robot1/orin_ros2_compute
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt

# Configure environment
cp config/.env.template config/.env
# Edit config/.env with your settings

# Install Ollama model
OLLAMA_NO_GPU=1 ollama pull qwen2.5:0.5b

# Run in development mode
./scripts/run_dev.sh
```

#### 3. Controller (Raspberry Pi)

```bash
# SSH to Controller
ssh pi@<controller_ip>

# Copy MicroVIT folder
scp -r MicroVIT/controller_rpi pi@<controller_ip>:~/MicroVIT/
scp -r MicroVIT/common pi@<controller_ip>:~/MicroVIT/

# Setup Python environment
cd ~/MicroVIT/controller_rpi
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt

# Configure environment
cp config/.env.template config/.env
# Edit config/.env

# Install MQTT broker (if not installed)
sudo apt-get install mosquitto mosquitto-clients

# Install Ollama model
OLLAMA_NO_GPU=1 ollama pull qwen2.5:0.5b

# Run in development mode
./scripts/run_dev.sh
```

#### 4. Helper Robot

```bash
# SSH to Helper Robot
ssh jetbot@<helper_ip>

# Copy MicroVIT folder
scp -r MicroVIT/helper_robot jetbot@<helper_ip>:~/MicroVIT/
scp -r MicroVIT/common jetbot@<helper_ip>:~/MicroVIT/

# Setup Python environment
cd ~/MicroVIT/helper_robot
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt

# Configure environment
cp config/.env.template config/.env
# Edit config/.env

# Install Ollama model
OLLAMA_NO_GPU=1 ollama pull qwen2.5:0.5b

# Run in development mode
./scripts/run_dev.sh
```

## Production Deployment

### Systemd Services

Copy systemd service files and enable:

**Nano:**
```bash
sudo cp MicroVIT/robot1/nano_ros1_master/systemd/*.service /etc/systemd/system/
sudo systemctl enable robot1-nano.service
sudo systemctl enable robot1-nano-bringup.service
sudo systemctl start robot1-nano.service
sudo systemctl start robot1-nano-bringup.service
```

**Orin:**
```bash
sudo cp MicroVIT/robot1/orin_ros2_compute/systemd/*.service /etc/systemd/system/
sudo systemctl enable robot1-orin-ai.service
sudo systemctl start robot1-orin-ai.service
```

**Controller:**
```bash
sudo cp MicroVIT/controller_rpi/systemd/*.service /etc/systemd/system/
sudo systemctl enable controller-ai.service
sudo systemctl start controller-ai.service
```

**Helper Robot:**
```bash
sudo cp MicroVIT/helper_robot/systemd/*.service /etc/systemd/system/
sudo systemctl enable helper-robot-ai.service
sudo systemctl start helper-robot-ai.service
```

## Configuration

All configuration files are in YAML format under each component's `config/` directory:

- `common/config/mqtt_config.yaml` - MQTT broker and topic configuration
- `common/config/ros_config.yaml` - ROS topic and parameter configuration
- `robot1/nano_ros1_master/config/device_config.yaml` - Nano hardware configuration
- `robot1/orin_ros2_compute/config/device_config.yaml` - Orin AI configuration
- `controller_rpi/config/device_config.yaml` - Controller configuration
- `helper_robot/config/device_config.yaml` - Helper robot configuration

Environment variables (`.env` files) are used for sensitive settings like passwords and API keys.

## Message Flow

1. **Robot1 (Nano)** captures camera image and LiDAR data
2. **Robot1 (Nano)** publishes sensor data via XML-RPC to **Robot1 (Orin)**
3. **Robot1 (Orin)** processes image with MicroViT and generates AI message
4. **Robot1 (Orin)** publishes alert/telemetry to MQTT broker
5. **Controller** receives alert, analyzes, and publishes command
6. **Helper Robot** receives command and executes task

## Debugging

### Check Network Connectivity
```bash
cd MicroVIT/tools/diagnostics
./ping_all.sh
```

### Check ROS Topics
```bash
# ROS1 (Nano)
./check_topics.sh ros1

# ROS2 (Orin)
./check_topics.sh ros2
```

### Check MQTT
```bash
cd MicroVIT/tools/networking
python3 check_mqtt.py <broker_ip> <port>
```

### Test LiDAR
```bash
cd MicroVIT/tools/lidar
python3 uart_read_test.py /dev/ttyUSB0 115200
```

## Expected Topics

### ROS1 (Nano)
- `/cmd_vel` - Velocity commands
- `/scan` - LiDAR scan data
- `/odom` - Odometry data

### ROS2 (Orin)
- `/cmd_vel` - Velocity commands (bridged from ROS1)
- `/scan` - LiDAR scan data (bridged from ROS1)
- `/odom` - Odometry data (bridged from ROS1)

### MQTT Topics
- `robots/{robot_id}/telemetry` - Robot telemetry
- `robots/{robot_id}/alerts` - Obstacle/alert messages
- `robots/{robot_id}/status` - Robot status updates
- `robots/{robot_id}/commands` - Commands from controller
- `controller/analysis` - Controller analysis results
- `helper/tasks` - Tasks for helper robot
- `helper/status` - Helper robot status

## Troubleshooting

See `docs/troubleshooting.md` for detailed troubleshooting guides including:
- **LiDAR connection issues (80008000 / RESULT_OPERATION_TIMEOUT)** - See `docs/LIDAR_RESOLVE.md` and `docs/lidar_troubleshooting.md`; run `tools/lidar/detailed_lidar_checkup.sh` on the Nano.
- Camera initialization problems
- ROS topic connectivity
- MQTT broker connection
- Ollama GPU memory issues

### Quick LiDAR Fix

If `/scan` topic is not publishing:
1. Run detailed checkup on Nano: `scp tools/lidar/detailed_lidar_checkup.sh jetbot@<nano-ip>:~/` then `bash ~/detailed_lidar_checkup.sh` (see `docs/LIDAR_RESOLVE.md`).
2. Enable real LiDAR: `./tools/lidar/enable_real_lidar.sh enable 256000` (or `115200` if checkup shows that).
3. See `docs/LIDAR_RESOLVE.md` and `docs/lidar_troubleshooting.md` for power/baud steps.

## Project Structure

```
MicroVIT/
├── README.md                    # This file
├── docs/                        # Documentation
│   ├── architecture.md          # Detailed architecture
│   ├── sequence_diagrams.md     # Message flow diagrams
│   └── troubleshooting.md       # Troubleshooting guides
├── common/                      # Shared code and configs
│   ├── config/                 # Common configuration files
│   ├── utils/                  # Shared utilities (MicroViT, etc.)
│   └── message_schema/         # Message schema definitions
├── robot1/                      # Robot1 components
│   ├── nano_ros1_master/       # Nano ROS1 master
│   ├── orin_ros2_compute/      # Orin ROS2 compute
│   └── bridge_ros1_ros2/       # ROS1-ROS2 bridge (XML-RPC)
├── controller_rpi/             # Controller (Raspberry Pi)
├── helper_robot/                # Helper Robot
├── tools/                       # Diagnostic and utility tools
│   ├── diagnostics/            # System diagnostics
│   ├── lidar/                  # LiDAR testing tools
│   └── networking/             # Network testing tools
└── logs/                        # Runtime logs (gitignored)
```

## License

[Add your license here]

## Support

For issues and questions, see `docs/troubleshooting.md` or check the logs in `logs/` directory.
