# Step-by-step system start

One ordered sequence to start the **real system** (Controller + Nano + Orin), then the **Streamlit UI** and optionally the **Grafana** stack. Use this order every time.

**IPs:** Controller Pi `10.13.68.48`, Nano `10.13.68.184`, Orin `10.13.68.159`. User `pi` on Pi, `jetbot` on Nano/Orin (password `jetbot`). **LiDAR baud:** 115200.

---

## Part 1 — Real system (Controller + Nano + Orin)

### Step 1.1 — Controller Pi (MQTT + Controller AI)

**From your Mac (or any machine with SSH):**

1. SSH to the Pi:
   ```bash
   ssh pi@10.13.68.48
   ```
2. Start MQTT broker and Controller AI service:
   ```bash
   sudo systemctl start mosquitto
   sudo systemctl start controller-ai.service
   ```
3. Confirm the service is running:
   ```bash
   systemctl is-active controller-ai.service
   ```
   You should see: **active**
4. You can exit the SSH session; both services keep running.

---

### Step 1.2 — Nano (ROS1 bringup: motor, camera, LiDAR, XML-RPC)

**Open a new terminal.**

1. SSH to the Nano:
   ```bash
   ssh jetbot@10.13.68.184
   ```
2. Source ROS and go to the workspace:
   ```bash
   source /opt/ros/melodic/setup.bash
   cd ~/MicroVIT/robot1/nano_ros1_master
   ```
3. Build if needed (first time or after code changes):
   ```bash
   [ -d devel ] || catkin_make
   source devel/setup.bash
   ```
4. Start full bringup **with LiDAR at 115200**:
   ```bash
   roslaunch jetbot_nano_bringup nano_bringup_full.launch use_lidar:=true lidar_serial_baudrate:=115200
   ```
   **Important:** Space between `nano_bringup_full.launch` and `use_lidar:=true` (not `.launcuse_lidar`).

5. **Leave this terminal open.** You should see:
   - `[JetBot Motor Driver] Serial connected to /dev/ttyACM0`
   - `✅ Camera initialized on /dev/video0`
   - `RPLIDAR S/N: ...` and `current scan mode: ...`
   - `Nano XML-RPC Server REALTIME] Started on 0.0.0.0:8000`

**If the camera does not start:** run `sudo chmod 666 /dev/video0` on the Nano, or try `camera_device:=/dev/video1` in the launch command. See COMPLETE_SYSTEM_START.md → Camera not started.

---

### Step 1.3 — Orin — Ollama (Terminal A)

**Open another terminal.**

1. SSH to the Orin:
   ```bash
   ssh jetbot@10.13.68.159
   ```
2. Start Ollama (LLM server):
   ```bash
   OLLAMA_NO_GPU=1 ollama serve
   ```
3. **Leave this terminal open.** This provides the model used by the Controller and Robot1 AI.

---

### Step 1.4 — Orin — Robot1 AI service (Terminal B)

**Open a second SSH session to the Orin** (so Ollama keeps running in the first).

1. SSH to the Orin again:
   ```bash
   ssh jetbot@10.13.68.159
   ```
2. Go to the Robot1 AI service directory and load env:
   ```bash
   cd ~/MicroVIT/robot1/orin_ros2_compute
   [ -d venv ] && source venv/bin/activate
   set -a; [ -f config/.env ] && . config/.env; set +a
   export USE_MICROVIT=true
   ```
3. Start the realtime AI service (publishes obstacle events to MQTT every 5 seconds):
   ```bash
   python src/robot1_ai_service_realtime.py --interval 5
   ```
4. **Leave this terminal open.** You should see logs like “Published REALTIME obstacle event” and no “Connection refused” errors.

---

### Step 1.5 — Quick check (real system)

- **Nano:** In a new terminal: `ssh jetbot@10.13.68.184` → `source /opt/ros/melodic/setup.bash` → `rosnode list` (expect `rplidarNode`, `nano_xmlrpc_server`, etc.) and `rostopic hz /scan` (~5–10 Hz).
- **Controller:** `ssh pi@10.13.68.48` → `sudo journalctl -u controller-ai.service -n 20` (expect “Ollama connected”, “MQTT”).
- **Orin:** Robot1 AI terminal should show periodic “Published REALTIME obstacle event” and no Ollama connection errors.

At this point the **real system** is running: Controller subscribes to Robot1 obstacle events and publishes AI decisions; Nano serves camera, LiDAR, and odometry; Orin runs the Robot1 AI and Ollama.

---

## Part 2 — Streamlit UI (see real-time messages)

**On your Mac (or laptop that can reach the Controller Pi):**

1. Go to the Streamlit app directory:
   ```bash
   cd "/Users/divya/Documents/Documents/Thesis/independent system/RPC-XML sys 20 Jan/MicroVIT_model change/RobotOps_Streamlit_UI"
   ```
2. Install dependencies once (if not already):
   ```bash
   pip install -r requirements.txt
   ```
3. Start the app:
   ```bash
   streamlit run app.py
   ```
4. Open the URL shown (e.g. **http://localhost:8501**).
5. In the **sidebar**, under **“Live system”**:
   - Check **“Use Live MQTT”**.
   - Set **MQTT broker host** to **10.13.68.48** (Controller Pi).
   - Set **MQTT broker port** to **1883**.
6. After a moment the sidebar should show **“Connected”**. Then open **Dashboard** or **Live Feed** to see real-time messages from Robot1, Controller, and Helper.

**Without the real system:** Leave “Use Live MQTT” unchecked, go to **Simulator**, click **Start simulator**, then view **Dashboard** / **Live Feed** (simulated messages only).

---

## Part 3 — Grafana stack (optional, ops-room dashboards)

**On your Mac (or any machine with Docker):**

1. Go to the Grafana stack directory:
   ```bash
   cd "/Users/divya/Documents/Documents/Thesis/independent system/RPC-XML sys 20 Jan/MicroVIT_model change/robotops-grafana"
   ```
2. Start the stack:
   ```bash
   docker compose up -d
   ```
3. Open **Grafana:** http://localhost:3000 — login **admin** / **admin**.
4. Go to **Dashboards → RobotOps** and open:
   - **RobotOps — Command Overview**
   - **RobotOps — Robot1 Deep Dive (Unified)**
   - **RobotOps — Alerts & Runbooks**

**To see live data in Grafana:** The stack’s gateway must receive MQTT. Either:

- Use the **sample publisher** (no real system needed):
  ```bash
  pip install paho-mqtt
  python scripts/publish_sample_messages.py --host localhost --port 1883
  ```
  (Stack includes Mosquitto on 1883.)

- Or point the gateway at your **Controller Pi** MQTT: before `docker compose up -d`, set `export MQTT_BROKER_HOST=10.13.68.48`, then start only the services that don’t need local Mosquitto (see robotops-grafana/README.md). The gateway expects topics like `robots/robot1/status`; your real system uses `robots/jetson1/obstacle` and `controller/ai_decision` — the current Grafana stack is set up for the sample schema. To use it with your real MQTT you’d extend the gateway to subscribe to those topics and map them into the same schema.

---

## Summary table

| Step | Where        | Action |
|------|--------------|--------|
| 1.1  | Controller Pi | `sudo systemctl start mosquitto controller-ai.service` |
| 1.2  | Nano         | SSH → `cd ~/MicroVIT/robot1/nano_ros1_master` → `source devel/setup.bash` → `roslaunch ... use_lidar:=true lidar_serial_baudrate:=115200` (leave running) |
| 1.3  | Orin (term A) | `OLLAMA_NO_GPU=1 ollama serve` (leave running) |
| 1.4  | Orin (term B) | `cd ~/MicroVIT/robot1/orin_ros2_compute` → env + `python src/robot1_ai_service_realtime.py --interval 5` (leave running) |
| 2    | Your Mac     | `cd RobotOps_Streamlit_UI` → `streamlit run app.py` → open URL → enable **Use Live MQTT**, host **10.13.68.48**, port **1883** |
| 3    | Your Mac (opt.) | `cd robotops-grafana` → `docker compose up -d` → open http://localhost:3000 (admin/admin) → RobotOps dashboards |

---

## Shutdown (reverse order)

1. Stop Streamlit (Ctrl+C in its terminal).
2. Stop Grafana stack (if used): `cd robotops-grafana` → `docker compose down`.
3. On Orin: Ctrl+C in both terminals (Robot1 AI, then Ollama).
4. On Nano: Ctrl+C in the roslaunch terminal.
5. On Controller Pi (optional): `ssh pi@10.13.68.48` → `sudo systemctl stop controller-ai.service mosquitto`.
