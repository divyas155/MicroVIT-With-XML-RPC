# System Architecture

## Overview

MicroVIT is a distributed robotics system with four main components:
1. **Robot1 Nano** - ROS1 master handling motors, sensors, and camera
2. **Robot1 Orin** - ROS2 compute node running MicroViT vision processing and AI
3. **Controller** - Raspberry Pi analyzing messages and routing commands
4. **Helper Robot** - Task execution robot

## Component Details

### Robot1 Nano (ROS1 Master)

**Hardware:**
- Jetson Nano
- USB Camera (`/dev/video0`)
- LiDAR (`/dev/ttyUSB0` or dummy)
- Motor driver (`/dev/ttyACM0`)

**Software:**
- ROS Melodic
- Python motor driver
- XML-RPC server for sensor data access
- ROS nodes: `jetbot_motor_driver`, `lidar_dummy_node`, `xmlrpc_server_nano_realtime`

**Responsibilities:**
- Motor control via `/cmd_vel` topic
- Camera capture and encoding
- LiDAR data collection
- Odometry tracking
- XML-RPC interface for Orin access

### Robot1 Orin (ROS2 Compute)

**Hardware:**
- Jetson Orin
- GPU (optional, can run CPU-only)

**Software:**
- ROS Humble
- MicroViT vision model
- Ollama text generation
- Python AI service

**Responsibilities:**
- Image preprocessing with MicroViT
- Natural language message generation
- MQTT communication
- XML-RPC client to Nano

### Controller (Raspberry Pi)

**Hardware:**
- Raspberry Pi
- Network connection

**Software:**
- MQTT broker (Mosquitto)
- Ollama text generation
- Python controller service

**Responsibilities:**
- Message analysis from robots
- Command routing
- Decision making
- MQTT topic management

### Helper Robot

**Hardware:**
- Jetson Orin (or similar)
- Motors and sensors

**Software:**
- Ollama text generation
- Flask API for task execution
- Python helper service

**Responsibilities:**
- Task execution
- Status reporting
- Command processing

## Communication Architecture

### XML-RPC (Nano ↔ Orin)

Direct communication between Nano and Orin:
- **Nano → Orin**: Sensor data (camera, LiDAR, odometry)
- **Orin → Nano**: Motor commands (via ROS `/cmd_vel` topic)

**Protocol**: XML-RPC over HTTP
**Port**: 8000 (default)
**Format**: Base64-encoded images, JSON sensor data

### MQTT (All Components)

Publish/subscribe messaging:
- **Robots → Controller**: Telemetry, alerts, status
- **Controller → Robots**: Commands, task assignments
- **Helper Robot ↔ Controller**: Task execution, status

**Broker**: Mosquitto (typically on Controller)
**Port**: 1883 (default)
**QoS**: 1 (at least once delivery)

### ROS Topics

**ROS1 (Nano):**
- `/cmd_vel` - Velocity commands (Twist)
- `/scan` - LiDAR scan data (LaserScan)
- `/odom` - Odometry data (Odometry)

**ROS2 (Orin):**
- `/cmd_vel` - Velocity commands (bridged from ROS1)
- `/scan` - LiDAR scan data (bridged from ROS1)
- `/odom` - Odometry data (bridged from ROS1)

## Data Flow

### Image Processing Pipeline

```
Camera (Nano) → XML-RPC → Orin → MicroViT → Feature Extraction → 
Ollama Text Generation → MQTT Alert Message
```

1. Nano captures image from USB camera
2. Image encoded to base64 JPEG
3. Sent via XML-RPC to Orin
4. Orin preprocesses with MicroViT (9ms GPU / 78ms CPU)
5. Features converted to text description
6. Combined with LiDAR data and location
7. Ollama generates natural language message
8. Published to MQTT as alert

### Command Flow

```
Controller Analysis → MQTT Command → Robot Subscription → 
ROS Topic → Motor Driver → Physical Movement
```

1. Controller analyzes robot telemetry
2. Generates command decision
3. Publishes command to MQTT
4. Robot subscribes and receives command
5. Command converted to ROS message
6. Published to `/cmd_vel` topic
7. Motor driver executes movement

## Message Schema

See `common/message_schema/messages.json` for complete schema definitions.

### Key Message Types

**Telemetry:**
- Location (x, y, theta)
- Sensor data (LiDAR, camera)
- Status flags

**Alerts:**
- Obstacle detection
- AI-generated natural language description
- Location and severity

**Commands:**
- Target device
- Command type (move, stop, execute_task)
- Parameters (velocity, target location, task details)

## Security Considerations

- MQTT authentication (username/password)
- Network isolation recommended
- XML-RPC over local network only
- ROS master access control

## Performance Characteristics

- **MicroViT Inference**: 9ms (GPU) / 78ms (CPU)
- **Ollama Text Generation**: 200-500ms (CPU)
- **XML-RPC Latency**: <10ms (local network)
- **MQTT Latency**: <5ms (local network)
- **Total Pipeline**: ~100-600ms (depending on hardware)

## Scalability

- Multiple robots can connect to same MQTT broker
- Controller can manage multiple robots
- Helper robots can be added/removed dynamically
- ROS1-ROS2 bridge allows mixed ROS versions
