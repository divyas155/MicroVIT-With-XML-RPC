# Troubleshooting Guide

## Resolving Terminal 22 (Nano), 23 (LiDAR), 24 (Orin)

### Terminal 24 – Orin: MQTT timeout, Ollama refused, systemd unit not found

**1. Install the systemd unit on the Orin** (run on Orin; no scp from Mac needed):

```bash
sudo tee /etc/systemd/system/robot1-orin-ai.service << 'ENDOFFILE'
[Unit]
Description=Robot1 Orin AI Service
After=network.target

[Service]
Type=simple
User=jetbot
WorkingDirectory=/home/jetbot/MicroVIT/robot1/orin_ros2_compute/src
Environment="NANO_IP=10.13.68.184"
Environment="NANO_PORT=8000"
Environment="MQTT_BROKER_HOST=10.13.68.48"
Environment="OLLAMA_NO_GPU=1"
ExecStartPre=/bin/bash -c 'curl -s http://localhost:11434/api/tags > /dev/null || (OLLAMA_NO_GPU=1 ollama serve & sleep 3)'
ExecStart=/bin/bash -c 'source /home/jetbot/MicroVIT/robot1/orin_ros2_compute/venv/bin/activate && python3 robot1_ai_service_realtime.py --simulation'
Restart=always
RestartSec=10

[Install]
WantedBy=multi-user.target
ENDOFFILE
sudo systemctl daemon-reload
```

**2. Fix MQTT (connection timed out):** Controller must run Mosquitto and be reachable from Orin.

- On **Controller**: `ssh pi@10.13.68.48` → `sudo systemctl start mosquitto`
- On **Orin**: `ping 10.13.68.48` (must succeed). If it fails, fix network/firewall.

**3. Start the Orin AI service:**  
On Orin: `sudo systemctl start robot1-orin-ai.service`  
(Ollama is started by the unit if not running.)  
Check: `systemctl is-active robot1-orin-ai.service` → `active`.

---

### Terminal 23 – Nano: LiDAR RESULT_OPERATION_TIMEOUT

- **Baud:** The repo default is now **256000**. On Nano, with roscore running, try:
  ```bash
  rosrun rplidar_ros rplidarNode _serial_port:=/dev/ttyUSB0 _serial_baudrate:=256000
  ```
  If that still times out, try **115200** (some units use it).

- **Power:** If the LiDAR disc is **not spinning**, the RPLidar needs more current. Use an **external 5V supply** (e.g. 2A) on the LiDAR power connector; keep USB for data.

- **Full bringup:** Use the updated launch (default 256000):
  ```bash
  roslaunch jetbot_nano_bringup nano_bringup_full.launch use_lidar:=true
  ```
  If LiDAR still fails, run with dummy LiDAR: `use_lidar:=false`.
- **Full step-by-step:** See **docs/LIDAR_RESOLVE.md** (power, baud, permissions, bringup).

---

### Terminal 22 – Nano: Camera VIDIOC / device index change

- The realtime server already falls back to other `/dev/video*` indices; you saw it recover on `/dev/video1`.
- To **force** a device (e.g. if `/dev/video0` is bad), copy the updated launch to the Nano and run:
  ```bash
  roslaunch jetbot_nano_bringup nano_bringup_full.launch camera_device:=/dev/video1
  ```
- Ensure only one process uses the camera (no other roslaunch or test using the same device).

---

## Common Issues

### LiDAR Not Working

**Symptoms:**
- `RESULT_OPERATION_TIMEOUT` error
- No scan data on `/scan` topic
- LiDAR node keeps restarting

**Diagnosis Steps:**

1. **Physical Checks:**
   ```bash
   # Check if LiDAR motor is spinning
   # Check LED status
   # Verify USB connection
   ```

2. **Check Device:**
   ```bash
   ls -l /dev/ttyUSB*
   # Should show: /dev/ttyUSB0 with permissions
   ```

3. **Check Permissions:**
   ```bash
   groups | grep dialout
   # If not in dialout group:
   sudo usermod -aG dialout $USER
   # Log out and back in
   ```

4. **Test UART Connection:**
   ```bash
   cd MicroVIT/tools/lidar
   python3 uart_read_test.py /dev/ttyUSB0 115200
   ```

5. **Test Different Baud Rates:**
   - 115200 (most common)
   - 230400
   - 256000

6. **Check Kernel Messages:**
   ```bash
   dmesg | tail -20 | grep -i 'tty\|uart\|serial'
   ```

**Solutions:**
- Try different baud rate in `config/device_config.yaml`
- Check LiDAR power supply
- Verify TX/RX wiring (if direct UART connection)
- Disable `nvgetty` if using UART: `sudo systemctl disable nvgetty`
- Use dummy LiDAR for testing: set `lidar.type: "dummy"` in config

### Camera Not Initializing

**Symptoms:**
- `Camera available: False`
- `OSError: [Errno 98] Address already in use`
- `method 'get_camera_image' is not supported`

**Diagnosis Steps:**

1. **Check Camera Device:**
   ```bash
   ls -l /dev/video*
   # Should show: /dev/video0 or /dev/video1
   ```

2. **Test Camera Directly:**
   ```bash
   v4l2-ctl --list-devices
   # Or
   ffplay /dev/video0
   ```

3. **Check if Port is in Use:**
   ```bash
   sudo fuser -k 8000/tcp
   ```

4. **Verify XML-RPC Server:**
   ```bash
   # Check if xmlrpc_server_nano_realtime.py is running
   ps aux | grep xmlrpc_server
   ```

**Solutions:**
- Update `camera_device` in `config/device_config.yaml`
- Use V4L2 backend: `cv2.CAP_V4L2`
- Request MJPEG format to avoid unsupported pixel formats
- Kill existing XML-RPC server processes
- Check camera permissions: `sudo chmod 666 /dev/video0`

### Ollama GPU Memory Error

**Symptoms:**
- `cudaMalloc failed: out of memory`
- Ollama crashes when loading model
- System becomes unresponsive

**Solutions:**

1. **Run Ollama in CPU Mode:**
   ```bash
   export OLLAMA_NO_GPU=1
   ollama serve
   ```

2. **Use Smaller Model:**
   ```bash
   ollama pull qwen2.5:0.5b  # Instead of phi3:mini
   ```

3. **Reduce Context Length:**
   ```yaml
   # In config/.env
   OLLAMA_MAX_TOKENS=200
   OLLAMA_CONTEXT_LENGTH=512
   ```

4. **Close Other GPU Processes:**
   ```bash
   nvidia-smi
   # Kill unnecessary GPU processes
   ```

### ROS Master Connection Issues

**Symptoms:**
- `roscore cannot run as another roscore/master is already running`
- `Unable to communicate with master`
- Topics not appearing

**Solutions:**

1. **Kill Existing ROS Master:**
   ```bash
   killall -9 rosmaster roscore
   pkill -9 -f roscore
   ```

2. **Check ROS_MASTER_URI:**
   ```bash
   echo $ROS_MASTER_URI
   # Should be: http://localhost:11311 (for Nano)
   ```

3. **Restart ROS Master:**
   ```bash
   roscore
   ```

### Controller AI: timeouts and no decision

**Symptoms:**
- `Error getting AI decision: Read timed out. (read timeout=30)`
- `WARNING - No AI decision received`
- Ollama log: `invalid option provided" option=max_tokens` or 500 on `/api/generate`

**Cause:** On the Raspberry Pi, qwen2.5:0.5b (or similar) is slow; long prompts and high token limit make Ollama miss the request timeout. Older Ollama rejects `max_tokens` in options (use `num_predict`).

**Fixes applied in code:**
- Controller uses `num_predict` (not `max_tokens`) for Ollama `/api/generate`.
- Config is loaded from `controller_rpi/config/.env` (so `AI_TIMEOUT`, `AI_MAX_TOKENS` apply when run by systemd).
- Prompt to Ollama is shortened (robot message truncated, one-line instruction + JSON).
- Defaults: `AI_TIMEOUT=15`, `AI_MAX_TOKENS=60` if `.env` is missing.

**What you should do on the Pi:**
1. In `~/MicroVIT/controller_rpi/config/.env` set:
   ```bash
   AI_TIMEOUT=10
   AI_MAX_TOKENS=40
   OLLAMA_MODEL=qwen2.5:0.5b
   ```
2. Restart: `sudo systemctl restart controller-ai.service`
3. Check logs (see “How to check controller logs” below).

---

### How to check controller logs

On the **Controller (Raspberry Pi)**, SSH in and use one of these:

**1. Follow logs live (recommended while testing)**  
```bash
ssh pi@10.13.68.48
sudo journalctl -u controller-ai.service -f
```
Press `Ctrl+C` to stop.

**2. Last N lines**  
```bash
sudo journalctl -u controller-ai.service -n 50 --no-pager
```
Change `50` to any number.

**3. Logs since a time**  
```bash
sudo journalctl -u controller-ai.service --since "10 min ago" --no-pager
# or
sudo journalctl -u controller-ai.service --since "2026-02-12 11:00:00" --no-pager
```

**4. Only controller Python messages (filter out Ollama noise)**  
```bash
sudo journalctl -u controller-ai.service -n 200 --no-pager | grep __main__
```

**5. Check if controller is running**  
```bash
systemctl is-active controller-ai.service
# active = running
```

**What to look for in logs:**
- `Connected to MQTT broker` – controller connected.
- `Using model: qwen2.5:0.5b` – which model is used.
- `Obstacle detected by jetson1` – received Robot1 event.
- `Robot1 AI Message: ...` – the message from Orin (not the controller’s reply).
- `AI Response raw: ...` and `Controller AI Message: ...` and `Executing AI decision: ...` – controller **did** generate an AI decision.
- `Error getting AI decision: ... Read timed out` and `No AI decision received` – controller **did not** get a reply from Ollama (timeout or error).

---

### MQTT Connection Issues

**Symptoms:**
- `Connection refused`
- Messages not being received
- Timeout errors

**Diagnosis Steps:**

1. **Check MQTT Broker:**
   ```bash
   systemctl status mosquitto
   # Or
   ps aux | grep mosquitto
   ```

2. **Test Connection:**
   ```bash
   cd MicroVIT/tools/networking
   python3 check_mqtt.py <broker_ip> <port>
   ```

3. **Check Network:**
   ```bash
   cd MicroVIT/tools/diagnostics
   ./ping_all.sh
   ```

**Solutions:**
- Start MQTT broker: `sudo systemctl start mosquitto`
- Check firewall: `sudo ufw allow 1883`
- Verify broker IP in `common/config/mqtt_config.yaml`
- Check username/password if authentication enabled

### Robot Not Moving

**Symptoms:**
- Commands published but robot doesn't move
- Motor driver not responding
- `/cmd_vel` topic has no subscribers

**Diagnosis Steps:**

1. **Check Motor Driver Node:**
   ```bash
   rosnode list | grep motor
   # Should show: /jetbot_motor_driver
   ```

2. **Check Topic Subscribers:**
   ```bash
   rostopic info /cmd_vel
   # Should show subscribers
   ```

3. **Test Motor Directly:**
   ```bash
   rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}"
   ```

4. **Check Motor Port:**
   ```bash
   ls -l /dev/ttyACM*
   # Should show: /dev/ttyACM0
   ```

**Solutions:**
- Ensure motor driver node is running
- Check motor port in `config/device_config.yaml`
- Verify motor hardware connection
- Check motor power supply

### AI Messages Not Generating

**Symptoms:**
- Only LiDAR fallback messages
- No camera analysis in messages
- Ollama errors in logs

**Diagnosis Steps:**

1. **Check Ollama:**
   ```bash
   curl http://localhost:11434/api/tags
   # Should show available models
   ```

2. **Check Environment Variables:**
   ```bash
   cat config/.env | grep USE_MICROVIT
   # Should be: USE_MICROVIT=true
   ```

3. **Check Logs:**
   ```bash
   tail -f logs/robot1_ai.log
   # Look for errors
   ```

**Solutions:**
- Ensure `.env` file exists and is configured
- Start Ollama: `OLLAMA_NO_GPU=1 ollama serve`
- Pull required model: `ollama pull qwen2.5:0.5b`
- Check MicroViT model is available: `USE_MICROVIT=true` in `.env`

## Debug Commands

### Check System Status

```bash
# Network connectivity
cd MicroVIT/tools/diagnostics
./ping_all.sh

# ROS topics
./check_topics.sh ros1  # For Nano
./check_topics.sh ros2  # For Orin

# MQTT
cd ../networking
python3 check_mqtt.py localhost 1883
```

### View Logs

```bash
# Nano logs
journalctl -u robot1-nano.service -f
journalctl -u robot1-nano-bringup.service -f

# Orin logs
journalctl -u robot1-orin-ai.service -f

# Controller logs
journalctl -u controller-ai.service -f

# Helper Robot logs
journalctl -u helper-robot-ai.service -f
```

### Manual Testing

```bash
# Test XML-RPC connection
python3 -c "import xmlrpc.client; n=xmlrpc.client.ServerProxy('http://10.13.68.184:8000'); print(n.get_robot_status())"

# Test ROS topics
rostopic echo /scan -n 1
rostopic echo /cmd_vel -n 1

# Test MQTT
mosquitto_sub -t "robots/+/alerts" -v
```

## Getting Help

1. Check logs in `MicroVIT/logs/` directory
2. Review configuration files in `config/` directories
3. Run diagnostic tools in `tools/` directory
4. Check systemd service status: `systemctl status <service-name>`
