# Complete Setup Guide - MicroVIT Robotics System

## Overview

This guide provides step-by-step instructions for setting up the complete MicroVIT robotics system on multiple devices. The system consists of:

- **Robot1**: Jetson Nano (ROS1 master) + Jetson Orin (ROS2 compute with MicroViT)
- **Controller**: Raspberry Pi (or any Linux machine) for message analysis
- **Helper Robot**: Task execution robot (optional)

---

## Table of Contents

1. [Prerequisites](#1-prerequisites)
2. [Repository Setup](#2-repository-setup)
3. [Device Configuration](#3-device-configuration)
4. [Nano Setup (Robot1 Base)](#4-nano-setup-robot1-base)
5. [Orin Setup (Robot1 Compute)](#5-orin-setup-robot1-compute)
6. [Controller Setup](#6-controller-setup)
7. [Helper Robot Setup](#7-helper-robot-setup)
8. [Starting the System](#8-starting-the-system)
9. [Verification and Testing](#9-verification-and-testing)
10. [Troubleshooting](#10-troubleshooting)

---

## 1. Prerequisites

### 1.1 Hardware Requirements

**Jetson Nano (Robot1 Base):**
- Jetson Nano with JetPack installed
- USB camera (MJPEG capable)
- Motor driver connected at `/dev/ttyACM0`
- (Optional) LiDAR sensor at `/dev/ttyUSB0`
- Network connectivity

**Jetson Orin (Robot1 Compute):**
- Jetson Orin with Ubuntu 22.04+ (or similar)
- Network connectivity to Nano
- Minimum 8GB RAM (for CPU-only AI processing)

**Controller (Raspberry Pi or PC):**
- Raspberry Pi 4+ or any Linux machine
- Python 3.9+
- Network connectivity
- MQTT broker (Mosquitto) installed

**Helper Robot (Optional):**
- Jetson Orin or similar device
- Python 3.9+
- Network connectivity

### 1.2 Software Requirements

**All Devices:**
- Python 3.9 or higher
- `python3-venv` package
- `python3-pip` package
- Git (for cloning repository)

**Nano Specific:**
- ROS Melodic (ROS1)
- `catkin` build tools
- OpenCV (usually pre-installed with JetPack)

**Orin Specific:**
- Ollama installed ([Installation Guide](https://ollama.com))
- PyTorch (CPU version) - will be installed via requirements.txt

**Controller Specific:**
- Mosquitto MQTT broker

---

## 2. Repository Setup

### 2.1 Clone the Repository

**Option A: Clone on Development Machine, then Copy to Devices**

```bash
# On your development machine
git clone https://github.com/divyas155/MicroVIT-With-XML-RPC.git
cd MicroVIT-With-XML-RPC/MicroVIT
```

**Option B: Clone Directly on Each Device**

```bash
# On each device (Nano, Orin, Controller, Helper Robot)
git clone https://github.com/divyas155/MicroVIT-With-XML-RPC.git
cd MicroVIT-With-XML-RPC/MicroVIT
```

### 2.2 Copy Files to Devices (if using Option A)

```bash
# From development machine, copy to each device:

# Nano
scp -r MicroVIT robot@<nano_ip>:~/MicroVIT

# Orin
scp -r MicroVIT robot@<orin_ip>:~/MicroVIT

# Controller
scp -r MicroVIT pi@<controller_ip>:~/MicroVIT

# Helper Robot (if using)
scp -r MicroVIT robot@<helper_ip>:~/MicroVIT
```

**Note:** Replace `robot`, `pi` with your actual usernames and IP addresses with your device IPs.

---

## 3. Device Configuration

### 3.1 Network Configuration

Ensure all devices are on the same network and can communicate:

```bash
# Test connectivity from each device
ping <nano_ip>
ping <orin_ip>
ping <controller_ip>
```

### 3.2 Default IP Addresses (Update as Needed)

- **Nano**: `10.13.68.184` (update in Orin's `.env` file)
- **Orin**: `10.13.68.159` (update in Controller's config if needed)
- **Controller**: Update in your network configuration
- **Helper Robot**: Update in your network configuration

---

## 4. Nano Setup (Robot1 Base)

### 4.1 SSH to Nano

```bash
ssh robot@<nano_ip>
# Replace 'robot' with your username
```

### 4.2 Navigate to MicroVIT Folder

```bash
cd ~/MicroVIT/robot1/nano_ros1_master
```

### 4.3 Build ROS Workspace

```bash
# Source ROS environment
source /opt/ros/melodic/setup.bash

# Build the workspace (first time only, or after code changes)
catkin_make

# Source the workspace
source devel/setup.bash
```

**Expected Output:**
```
Base path: /home/robot/MicroVIT/robot1/nano_ros1_master
Source space: /home/robot/MicroVIT/robot1/nano_ros1_master/src
...
[100%] Built target jetbot_nano_bringup
```

### 4.4 Configure Camera Device

**Check available camera devices:**
```bash
ls -l /dev/video*
```

**Update launch file if camera is not `/dev/video0`:**
```bash
nano launch/nano_bringup_full.launch
```

Find this line and update if needed:
```xml
<param name="camera_device" value="/dev/video0" />
```

Change to `/dev/video1` or your camera device if different.

### 4.5 Configure Device Settings (Optional)

**Edit device configuration:**
```bash
nano config/device_config.yaml
```

Key settings:
- `camera.device`: Camera device path
- `lidar.type`: `"dummy"` (default) or `"rplidar"` if you have real LiDAR
- `motors.port`: Motor driver port (usually `/dev/ttyACM0`)

### 4.6 Verify ROS Package

```bash
source /opt/ros/melodic/setup.bash
source devel/setup.bash
rospack find jetbot_nano_bringup
# Should show: /home/robot/MicroVIT/robot1/nano_ros1_master/src/jetbot_nano_bringup
```

**Nano setup is complete!** Proceed to Orin setup.

---

## 5. Orin Setup (Robot1 Compute)

### 5.1 SSH to Orin

```bash
ssh robot@<orin_ip>
# Replace 'robot' with your username
```

### 5.2 Navigate to MicroVIT Folder

```bash
cd ~/MicroVIT/robot1/orin_ros2_compute
```

### 5.3 Create Python Virtual Environment

```bash
# Create virtual environment
python3 -m venv venv

# Activate virtual environment
source venv/bin/activate

# Upgrade pip
pip install --upgrade pip
```

### 5.4 Install Python Dependencies

```bash
# Install from requirements.txt
pip install -r requirements.txt

# This will install:
# - Core packages: requests, paho-mqtt, python-dotenv, opencv-python, pillow, numpy
# - AI packages: torch, torchvision, transformers (large downloads, may take 10-15 minutes)
```

**Note:** If installation times out, retry with increased timeout:
```bash
pip install --timeout 600 -r requirements.txt
```

### 5.5 Verify Critical Packages

```bash
source venv/bin/activate
python3 -c "import requests; import paho.mqtt.client; import cv2; import numpy; print('✅ Core packages OK')"
python3 -c "import torch; import transformers; print('✅ AI packages OK')"
```

### 5.6 Configure Environment Variables

```bash
cd config

# Copy template to actual .env file
cp .env.template .env

# Edit .env file
nano .env
```

**Configure the following variables:**

```bash
# MicroViT Configuration
USE_MICROVIT=true
MICROVIT_MODEL_NAME=apple/mobilevit-small
MICROVIT_VARIANT=S1
MICROVIT_USE_CPU=true

# Nano Connection
NANO_IP=10.13.68.184          # Update with your Nano IP
NANO_PORT=8000

# Ollama Configuration
OLLAMA_HOST=http://localhost:11434
OLLAMA_MODEL=qwen2.5:0.5b
OLLAMA_TEXT_MODEL=qwen2.5:0.5b
OLLAMA_NO_GPU=1                # CPU mode (safer, avoids GPU OOM)
OLLAMA_MAX_TOKENS=500
OLLAMA_CONTEXT_LENGTH=512

# MQTT Configuration
MQTT_BROKER_HOST=localhost      # Update if broker is on different machine
MQTT_BROKER_PORT=1883
ROBOT_ID=robot1_orin
```

**Save and exit** (Ctrl+X, then Y, then Enter in nano).

### 5.7 Install Ollama Model

**Check if Ollama is installed:**
```bash
which ollama
# If not found, install: curl -fsSL https://ollama.ai/install.sh | sh
```

**Pull the required model:**
```bash
OLLAMA_NO_GPU=1 ollama pull qwen2.5:0.5b
```

**Verify model is available:**
```bash
OLLAMA_NO_GPU=1 ollama list | grep qwen2.5:0.5b
# Should show: qwen2.5:0.5b
```

### 5.8 Create Logs Directory

```bash
mkdir -p src/logs
```

**Orin setup is complete!** Proceed to Controller setup (optional).

---

## 6. Controller Setup

### 6.1 SSH to Controller

```bash
ssh pi@<controller_ip>
# Replace 'pi' with your username
```

### 6.2 Navigate to MicroVIT Folder

```bash
cd ~/MicroVIT/controller_rpi
```

### 6.3 Create Python Virtual Environment

```bash
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip
```

### 6.4 Install Python Dependencies

```bash
pip install -r requirements.txt
```

### 6.5 Configure Environment Variables

```bash
cd config
cp .env.template .env
nano .env
```

**Configure:**
```bash
MQTT_BROKER_HOST=localhost      # If broker is on this machine
MQTT_BROKER_PORT=1883
OLLAMA_HOST=http://localhost:11434
OLLAMA_MODEL=qwen2.5:0.5b
OLLAMA_NO_GPU=1
CONTROLLER_ID=central_controller
```

### 6.6 Install MQTT Broker (Mosquitto)

```bash
sudo apt-get update
sudo apt-get install -y mosquitto mosquitto-clients

# Enable and start service
sudo systemctl enable mosquitto
sudo systemctl start mosquitto

# Verify it's running
sudo systemctl status mosquitto
```

### 6.7 Install Ollama Model (if Controller runs AI)

```bash
OLLAMA_NO_GPU=1 ollama pull qwen2.5:0.5b
```

**Controller setup is complete!**

---

## 7. Helper Robot Setup

### 7.1 SSH to Helper Robot

```bash
ssh robot@<helper_ip>
```

### 7.2 Navigate to MicroVIT Folder

```bash
cd ~/MicroVIT/helper_robot
```

### 7.3 Create Python Virtual Environment

```bash
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip
```

### 7.4 Install Python Dependencies

```bash
pip install -r requirements.txt
```

### 7.5 Configure Environment Variables

```bash
cd config
cp .env.template .env
nano .env
```

**Configure:**
```bash
MQTT_BROKER_HOST=<controller_ip>
MQTT_BROKER_PORT=1883
OLLAMA_HOST=http://localhost:11434
OLLAMA_MODEL=qwen2.5:0.5b
OLLAMA_NO_GPU=1
ROBOT_ID=helper_robot
FLASK_PORT=5000
```

### 7.6 Install Ollama Model

```bash
OLLAMA_NO_GPU=1 ollama pull qwen2.5:0.5b
```

**Helper Robot setup is complete!**

---

## 8. Starting the System

### 8.1 Start Nano Services

**Terminal 1 (Nano) - ROS Master:**
```bash
ssh robot@<nano_ip>
cd ~/MicroVIT/robot1/nano_ros1_master
source /opt/ros/melodic/setup.bash
source devel/setup.bash
roscore
```

**Keep this terminal open!**

**Terminal 2 (Nano) - Bringup:**
```bash
ssh robot@<nano_ip>
cd ~/MicroVIT/robot1/nano_ros1_master
source /opt/ros/melodic/setup.bash
source devel/setup.bash
roslaunch jetbot_nano_bringup nano_bringup_full.launch
```

**Expected Output:**
```
[jetbot_motor_driver-1] process started with pid [xxxxx]
[nano_lidar_dummy-2] process started with pid [xxxxx]
[nano_xmlrpc_server-3] process started with pid [xxxxx]
[Nano REALTIME] ✅ Camera initialized on /dev/video0 (MJPEG 640x480)
[Nano XML-RPC Server REALTIME] Started on 0.0.0.0:8000
```

**Keep this terminal open!**

### 8.2 Start Orin Services

**Terminal 3 (Orin) - Ollama:**
```bash
ssh robot@<orin_ip>
OLLAMA_NO_GPU=1 ollama serve
```

**Keep this terminal open!**

**Terminal 4 (Orin) - AI Service:**
```bash
ssh robot@<orin_ip>
cd ~/MicroVIT/robot1/orin_ros2_compute
source venv/bin/activate

# Load environment variables
export $(cat config/.env | grep -v '^#' | xargs)

# Set defaults
export NANO_IP=${NANO_IP:-"10.13.68.184"}
export NANO_PORT=${NANO_PORT:-8000}
export OLLAMA_NO_GPU=${OLLAMA_NO_GPU:-1}

# Run AI service
cd src
python3 robot1_ai_service_realtime.py --simulation
```

**Expected Output:**
```
============================================================
Robot1 AI Service - REALTIME MODE
Using REAL camera, LiDAR, and odometry from Nano
============================================================
✅ XML-RPC connected to Nano at http://10.13.68.184:8000
   Camera available: True
✅ MQTT Connected successfully
✅ Ollama connected. Available models: ['qwen2.5:0.5b']
🚀 Using MicroViT model for fast image preprocessing
✅ MicroViT model loaded successfully!
🤖 Robot1 AI Service started in REALTIME MODE
Starting continuous REALTIME detection every 30 seconds
✅ Captured REAL image from Nano (640x480)
✅ Generated AI message with MicroViT+Ollama (FULL):
[Detailed natural language message appears here]
```

**Keep this terminal open to monitor AI messages!**

### 8.3 Start Controller (Optional)

**Terminal 5 (Controller):**
```bash
ssh pi@<controller_ip>
cd ~/MicroVIT/controller_rpi
source venv/bin/activate
cd src
python3 controller_ai_service.py
```

### 8.4 Start Helper Robot (Optional)

**Terminal 6 (Helper Robot):**
```bash
ssh robot@<helper_ip>
cd ~/MicroVIT/helper_robot
source venv/bin/activate
cd src
python3 helper_robot_ai_service.py
```

---

## 9. Verification and Testing

### 9.1 Verify Nano Services

**On Nano (new terminal):**
```bash
source /opt/ros/melodic/setup.bash
source ~/MicroVIT/robot1/nano_ros1_master/devel/setup.bash

# Check ROS nodes
rosnode list
# Should show: /jetbot_motor_driver, /nano_lidar_dummy, /nano_xmlrpc_server

# Check topics
rostopic list | grep -E 'cmd_vel|scan|odom'
# Should show: /cmd_vel, /scan, /odom

# Test XML-RPC connection
python3 -c "import xmlrpc.client; n=xmlrpc.client.ServerProxy('http://localhost:8000'); s=n.get_robot_status(); print('Camera:', s.get('camera_available')); print('Status:', s.get('status_message'))"
# Should show: Camera: True
```

### 9.2 Verify Orin Services

**On Orin (new terminal):**
```bash
# Check Ollama
curl -s http://localhost:11434/api/tags | grep qwen2.5:0.5b
# Should show the model

# Check AI service process
ps aux | grep robot1_ai_service_realtime | grep -v grep
# Should show the process running
```

### 9.3 Test MQTT Connectivity

**On Controller or any machine:**
```bash
cd ~/MicroVIT/tools/networking
python3 check_mqtt.py <broker_ip> 1883
```

### 9.4 Test Network Connectivity

**From any machine:**
```bash
cd ~/MicroVIT/tools/diagnostics
./ping_all.sh
```

### 9.5 Monitor AI Messages

**Watch Orin Terminal 4** for AI messages every ~30 seconds. You should see:
- `✅ Captured REAL image from Nano (640x480)`
- `✅ Generated AI message with MicroViT+Ollama (FULL):`
- Full natural language messages describing the scene

---

## 10. Troubleshooting

### 10.1 Camera Not Initializing

**Symptoms:** `Camera available: False` or camera errors

**Solutions:**
```bash
# Check camera device
ls -l /dev/video*

# Update launch file
nano ~/MicroVIT/robot1/nano_ros1_master/launch/nano_bringup_full.launch
# Change camera_device parameter

# Test camera directly
v4l2-ctl --list-devices
```

### 10.2 Ollama Model Not Found

**Symptoms:** `Ollama generation failed` or model errors

**Solutions:**
```bash
# Re-pull model
OLLAMA_NO_GPU=1 ollama pull qwen2.5:0.5b

# Verify model
OLLAMA_NO_GPU=1 ollama list

# Restart Ollama
pkill -9 ollama
OLLAMA_NO_GPU=1 ollama serve
```

### 10.3 MicroViT Not Loading

**Symptoms:** Simple fallback messages instead of detailed AI messages

**Solutions:**
```bash
# Check if torch is installed
cd ~/MicroVIT/robot1/orin_ros2_compute
source venv/bin/activate
python3 -c "import torch; print('Torch:', torch.__version__)"

# If not installed:
pip install --timeout 600 torch torchvision --index-url https://download.pytorch.org/whl/cpu
pip install --timeout 600 transformers

# Check .env file
cat config/.env | grep USE_MICROVIT
# Should show: USE_MICROVIT=true
```

### 10.4 XML-RPC Connection Failed

**Symptoms:** `XML-RPC connection to Nano failed`

**Solutions:**
```bash
# On Orin, test connection
curl http://<nano_ip>:8000

# On Nano, check if server is running
ps aux | grep xmlrpc_server

# Check firewall
sudo ufw allow 8000/tcp
```

### 10.5 MQTT Connection Issues

**Symptoms:** `MQTT connection failed`

**Solutions:**
```bash
# Check if broker is running
sudo systemctl status mosquitto

# Start broker if not running
sudo systemctl start mosquitto

# Check firewall
sudo ufw allow 1883/tcp

# Test connection
mosquitto_sub -t "test" -v
```

### 10.6 Python Package Installation Timeout

**Symptoms:** `ReadTimeoutError` during pip install

**Solutions:**
```bash
# Increase timeout
pip install --timeout 600 -r requirements.txt

# Install packages individually
pip install --timeout 600 requests paho-mqtt python-dotenv
pip install --timeout 600 opencv-python pillow numpy
pip install --timeout 600 torch torchvision transformers
```

### 10.7 ROS Package Not Found

**Symptoms:** `Cannot locate node` or package errors

**Solutions:**
```bash
# Rebuild workspace
cd ~/MicroVIT/robot1/nano_ros1_master
source /opt/ros/melodic/setup.bash
catkin_make
source devel/setup.bash

# Verify package
rospack find jetbot_nano_bringup
```

---

## 11. Customization

### 11.1 Change Detection Interval

**On Orin, when starting AI service:**
```bash
python3 robot1_ai_service_realtime.py --simulation --interval 10
# Changes interval to 10 seconds (default is 30)
```

### 11.2 Change AI Model

**Edit `.env` file on Orin:**
```bash
nano ~/MicroVIT/robot1/orin_ros2_compute/config/.env
# Change:
OLLAMA_MODEL=tinyllama  # Instead of qwen2.5:0.5b
OLLAMA_TEXT_MODEL=tinyllama
```

**Then pull new model:**
```bash
OLLAMA_NO_GPU=1 ollama pull tinyllama
```

### 11.3 Change Robot ID

**Edit `.env` files:**
- Orin: `ROBOT_ID=robot1_orin` → `ROBOT_ID=my_robot`
- Controller: `CONTROLLER_ID=central_controller` → `CONTROLLER_ID=my_controller`
- Helper: `ROBOT_ID=helper_robot` → `ROBOT_ID=my_helper`

### 11.4 Change MQTT Topics

**Edit code files:**
- Orin: `robot1/orin_ros2_compute/src/robot1_ai_service_realtime.py`
- Controller: `controller_rpi/src/controller_ai_service.py`
- Helper: `helper_robot/src/helper_robot_ai_service.py`

Or update `common/config/mqtt_config.yaml` and ensure code reads from it.

---

## 12. Production Deployment

### 12.1 Using Systemd Services

**Copy service files:**
```bash
# On Nano
sudo cp ~/MicroVIT/robot1/nano_ros1_master/systemd/*.service /etc/systemd/system/

# On Orin
sudo cp ~/MicroVIT/robot1/orin_ros2_compute/systemd/*.service /etc/systemd/system/

# On Controller
sudo cp ~/MicroVIT/controller_rpi/systemd/*.service /etc/systemd/system/

# On Helper Robot
sudo cp ~/MicroVIT/helper_robot/systemd/*.service /etc/systemd/system/
```

**Update service files with correct paths:**
```bash
sudo nano /etc/systemd/system/robot1-nano.service
# Update WorkingDirectory and paths
```

**Enable and start services:**
```bash
sudo systemctl enable robot1-nano.service
sudo systemctl enable robot1-nano-bringup.service
sudo systemctl enable robot1-orin-ai.service
sudo systemctl start robot1-nano.service
sudo systemctl start robot1-nano-bringup.service
sudo systemctl start robot1-orin-ai.service
```

**Check status:**
```bash
sudo systemctl status robot1-nano.service
sudo systemctl status robot1-orin-ai.service
```

---

## 13. Quick Reference

### 13.1 Essential Commands

**Stop all services:**
```bash
# Nano
pkill -9 -f 'ros|xmlrpc|motor|lidar'; killall -9 rosmaster

# Orin
pkill -9 -f 'ollama|robot1_ai'; killall -9 ollama
```

**Start services (quick):**
```bash
# Nano
cd ~/MicroVIT/robot1/nano_ros1_master && ./scripts/run_dev.sh

# Orin
cd ~/MicroVIT/robot1/orin_ros2_compute && ./scripts/run_dev.sh
```

### 13.2 File Locations

- **Nano ROS workspace**: `~/MicroVIT/robot1/nano_ros1_master/`
- **Orin AI service**: `~/MicroVIT/robot1/orin_ros2_compute/`
- **Controller service**: `~/MicroVIT/controller_rpi/`
- **Helper robot**: `~/MicroVIT/helper_robot/`
- **Config files**: `*/config/.env` and `*/config/device_config.yaml`
- **Logs**: `*/src/logs/` or `*/logs/`

### 13.3 Important Ports

- **XML-RPC (Nano)**: 8000
- **MQTT Broker**: 1883
- **Ollama**: 11434
- **Flask (Helper Robot)**: 5000

---

## 14. Support and Documentation

- **Main README**: `MicroVIT/README.md`
- **Quick Start**: `MicroVIT/QUICK_START.md`
- **Restart Guide**: `MicroVIT/RESTART_GUIDE.md`
- **Architecture**: `MicroVIT/docs/architecture.md`
- **Troubleshooting**: `MicroVIT/docs/troubleshooting.md`
- **Sequence Diagrams**: `MicroVIT/docs/sequence_diagrams.md`

---

## 15. Next Steps

After successful setup:

1. **Monitor AI messages** on Orin terminal
2. **Test motor control** via ROS topics
3. **Configure Controller** to analyze and route commands
4. **Set up Helper Robot** for task execution
5. **Customize** for your specific use case

---

## Summary Checklist

- [ ] Repository cloned on all devices
- [ ] Nano ROS workspace built
- [ ] Orin Python environment created and packages installed
- [ ] Ollama installed and model pulled on Orin
- [ ] `.env` files configured on all devices
- [ ] MQTT broker installed and running (Controller)
- [ ] Camera device verified on Nano
- [ ] All services started successfully
- [ ] AI messages generating on Orin
- [ ] Network connectivity verified

**Congratulations! Your MicroVIT robotics system is now set up and running!**
