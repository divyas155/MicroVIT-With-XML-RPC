## Quick Start – Restart Entire System (Controller + Nano + Orin)

This guide reflects the **current IPs and services** and shows how to bring up the full system (real Nano sensors, MicroViT on Orin, controller AI on Pi) after a reboot.

Machines:
- **Controller Pi**: `10.13.68.48` (user `pi`)
- **Nano**: `10.13.68.184` (user `jetbot`, password `jetbot`)
- **Orin**: `10.13.68.159` (user `jetbot`, password `jetbot`)

**One-time:** To start everything from your laptop with one script (no password each time), run:
`./setup_ssh_keys.sh` and enter password `jetbot` when prompted for Nano and Orin. Then use `./start_system.sh` to start the full system.

---

## 1. Controller Pi – MQTT + Controller AI

### 1.1 Start services

On your laptop / Mac:

```bash
ssh pi@10.13.68.48
```

Start MQTT broker (if not already running):

```bash
sudo systemctl start mosquitto
```

Start controller AI service (this will also start and warm up Ollama on the Pi):

```bash
sudo systemctl start controller-ai.service
systemctl is-active controller-ai.service
```

You should see `active`.

### 1.2 Watch controller decisions (optional)

Still on the Pi:

```bash
sudo journalctl -u controller-ai.service -f
```

Look for:
- `Controller Ollama connected. Available models: [...]`
- `Ollama model ready.`
- Later, for each event:
  - `Obstacle detected by jetson1: ...`
  - `Robot1 AI Message: ...`
  - `Controller AI Message: ...`
  - `Executing AI decision: ...`
  - `Sent ... command to jetson1` (or `robot1_orin|helper_robot`)

Leave this running if you want live logs.

---

## 2. Nano – ROS1 Bringup (camera, LiDAR, odom, XML-RPC)

### 2.1 Start full bringup

Open a new terminal:

```bash
ssh jetbot@10.13.68.184
source /opt/ros/melodic/setup.bash

cd ~/MicroVIT/robot1/nano_ros1_master
[ -d devel ] || catkin_make
source devel/setup.bash
```

Start Nano bringup (use **115200** if your LiDAR only responds at that baud; otherwise omit `lidar_serial_baudrate`):

```bash
# Real LiDAR at 115200 (use 256000 or omit if your unit needs it)
roslaunch jetbot_nano_bringup nano_bringup_full.launch use_lidar:=true lidar_serial_baudrate:=115200
```

You should see:
- `[JetBot Motor Driver] Serial connected to /dev/ttyACM0`
- `✅ Camera initialized on /dev/video0`
- `Nano XML-RPC Server REALTIME] Started on 0.0.0.0:8000`

If the LiDAR keeps giving `RESULT_OPERATION_TIMEOUT` / `Can not start scan: 80008000!` (same issue as before):
- Run the detailed checkup on the Nano: `scp tools/lidar/detailed_lidar_checkup.sh jetbot@10.13.68.184:~/` then on Nano `bash ~/detailed_lidar_checkup.sh`; see **docs/LIDAR_RESOLVE.md** (power, baud, permissions). For a **thorough investigation** (why it fails, what to try first), see **docs/LIDAR_INVESTIGATION_REPORT.md**.
- To skip real LiDAR and use dummy for a clean run:

```bash
roslaunch jetbot_nano_bringup nano_bringup_full.launch use_lidar:=false
```

That uses the **dummy LiDAR** (`nano_lidar_dummy`) so `/scan` is always available and Orin sees LiDAR as simulated/fallback.

### 2.2 Verify Nano ROS graph

In a **second Nano terminal**:

```bash
ssh jetbot@10.13.68.184
source /opt/ros/melodic/setup.bash

rosnode list
```

You should see at least:
- `/jetbot_motor_driver`
- `/nano_xmlrpc_server`
- `/nano_motor_service_server`
- `/base_to_laser`
- `/rplidarNode` **or** `/nano_lidar_dummy`
- `/rosout`

And:

```bash
rostopic info /scan
```

Should show:
- **Publisher**: `/rplidarNode` (real) or `/nano_lidar_dummy` (dummy)
- **Subscriber**: `/nano_xmlrpc_server`

Leave the main `roslaunch` terminal running.

---

## 3. Orin – Ollama Server (LLM for MicroViT)

### 3.1 Start Ollama (Terminal A)

Open a terminal:

```bash
ssh jetbot@10.13.68.159
```

Run Ollama in CPU mode:

```bash
OLLAMA_NO_GPU=1 ollama serve
```

Leave this terminal running. This provides the model (e.g. `qwen2.5:0.5b`) that MicroViT and the controller use.

Optionally, in another Orin terminal:

```bash
curl -s http://localhost:11434/api/tags
```

You should see JSON with the model list.

---

## 4. Orin – Robot1 Realtime AI + MicroViT

### 4.1 Start Robot1 AI service (Terminal B)

Open another terminal:

```bash
ssh jetbot@10.13.68.159

cd ~/MicroVIT/robot1/orin_ros2_compute
[ -d venv ] && source venv/bin/activate
set -a; [ -f config/.env ] && . config/.env; set +a

# Optional: force MicroViT usage if installed
export USE_MICROVIT=true
```

Start the realtime AI service (publishes events every 5 seconds):

```bash
python src/robot1_ai_service_realtime.py --interval 5
```

You should see logs like:
- `Position (REAL): (...)`
- `✅ Captured REAL image from Nano (640x480)`
- `LiDAR (LiDAR (REAL RPLIDAR) or (FALLBACK DUMMY)): ...m`
- `Published REALTIME obstacle event (FULL): Robot jetson1 at (...): LiDAR (...) shows ...m to nearest obstacle.`

If you still see:
- `Failed to establish a new connection: [Errno 111] Connection refused`

then Ollama on Orin is not running or not reachable (recheck step 3.1).

Leave this running.

---

## 5. Quick Verification Checklist

### 5.1 Controller Pi

On the Pi:

```bash
sudo journalctl -u controller-ai.service -n 40 --no-pager
```

You should see:
- `Controller Ollama connected. Available models: ['qwen2.5:0.5b']`
- `Ollama model ready.`
- `Starting Controller AI Service...`
- `Connected to MQTT broker`

When the system is running:
- `Obstacle detected by jetson1: ...`
- `Robot1 AI Message: ...`
- `Controller AI Message: ...`
- `Executing AI decision: ...`
- `Sent ... command to jetson2` or `jetson1` or `robot1_orin|helper_robot`

### 5.2 Nano

```bash
rosnode list
rostopic info /scan
```

Check that `/scan` has a publisher (`rplidarNode` or `nano_lidar_dummy`) and that `/nano_xmlrpc_server` subscribes.

### 5.3 Orin

In the Robot1 AI terminal, ensure there are no repeated connection‑refused errors for Ollama and that obstacle events are being published regularly.

---

## 6. Minimal “One‑Line per Machine” Restart Summary

- **Controller Pi**  
  ```bash
  ssh pi@10.13.68.48
  sudo systemctl start mosquitto
  sudo systemctl start controller-ai.service
  ```

- **Nano**  
  ```bash
  ssh jetbot@10.13.68.184
  source /opt/ros/melodic/setup.bash
  cd ~/MicroVIT/robot1/nano_ros1_master
  [ -d devel ] || catkin_make
  source devel/setup.bash
  roslaunch jetbot_nano_bringup nano_bringup_full.launch use_lidar:=true   # or use_lidar:=false
  ```

- **Orin**  
  ```bash
  # Terminal A
  ssh jetbot@10.13.68.159
  OLLAMA_NO_GPU=1 ollama serve

  # Terminal B
  ssh jetbot@10.13.68.159
  cd ~/MicroVIT/robot1/orin_ros2_compute
  [ -d venv ] && source venv/bin/activate
  set -a; [ -f config/.env ] && . config/.env; set +a
  python src/robot1_ai_service_realtime.py --interval 5
  ```

If all three are running and you see **Controller AI Message** lines on the Pi, the full system (Nano sensors + MicroViT on Orin + controller decisions) is up and working.
