# Complete System Start (Controller + Nano + Orin)

Use this order every time. **REAL SENSORS ONLY** — no dummy/fallback. Camera, LiDAR, and odometry must all be real for Orin to generate AI messages.

- **Camera:** Real USB camera on `/dev/video0` or `/dev/video1` (no placeholder)
- **LiDAR:** Real RPLIDAR with `use_lidar:=true` (no dummy LiDAR)
- **Odometry:** Motor driver publishes real `/odom` (use_odom_dummy:=false by default)

**LiDAR baud:** 115200 — use `lidar_serial_baudrate:=115200` on the Nano.

---

## Step-by-step startup (all components)

Open 5–6 terminals. Follow in order:

| Step | Machine | Action |
|------|---------|--------|
| 1 | Controller Pi | MQTT broker + Controller AI |
| 2 | Nano | ROS bringup (camera, LiDAR, motor, XML-RPC) |
| 3 | Orin (Terminal A) | Ollama server |
| 4 | Orin (Terminal B) | Robot1 AI service |
| 5 | Helper Robot *(optional)* | Helper AI service |

---

### Step 1: Controller Pi

**SSH to Controller:**

```bash
ssh pi@10.13.68.48
```

**Start MQTT broker and Controller AI:**

```bash
sudo systemctl start mosquitto
sudo systemctl start controller-ai.service
```

**Verify:**

```bash
systemctl is-active mosquitto       # should print: active
systemctl is-active controller-ai.service   # should print: active
```

You can exit SSH; the services keep running.

---

### Step 2: Nano (Robot1)

**SSH to Nano:**

```bash
ssh jetbot@10.13.68.184
```

**Start ROS bringup (real sensors only):**

```bash
source /opt/ros/melodic/setup.bash
cd ~/MicroVIT/robot1/nano_ros1_master
[ -d devel ] || catkin_make
source devel/setup.bash
roslaunch jetbot_nano_bringup nano_bringup_full.launch use_lidar:=true lidar_serial_baudrate:=115200 camera_device:=/dev/video0 use_odom_dummy:=false
```

Use `camera_device:=/dev/video1` if your camera is on video1.

**Optional (if camera drops mid-session):** Disable USB autosuspend before starting:
```bash
~/disable_usb_autosuspend.sh   # after copying tools/camera/disable_usb_autosuspend.sh
```

**Note:** Controller AI uses **Orin's Ollama** (`OLLAMA_HOST=http://10.13.68.159:11434`) — Ollama runs on Orin only, not on the Pi. Ensure Ollama is running on Orin (step 3) before the Controller processes obstacles.

**Keep this terminal open.** Expect:

- `[JetBot Motor Driver] Serial connected to /dev/ttyACM0`
- `[Nano Camera] Opened /dev/video0 via V4L2 (320x240)` and `Publishing to /nano/camera/image_compressed`
- `RPLIDAR S/N: ...`
- `Nano XML-RPC Server REALTIME Started on 0.0.0.0:8000`

---

### Step 3: Orin – Ollama (Terminal A)

**Open a new terminal, SSH to Orin:**

```bash
ssh jetbot@10.13.68.159
OLLAMA_NO_GPU=1 ollama serve
```

**Keep this terminal open.** Ollama must be running before the Robot1 AI service.

---

### Step 4: Orin – Robot1 AI service (Terminal B)

**Open another terminal, SSH to Orin:**

```bash
ssh jetbot@10.13.68.159
cd ~/MicroVIT/robot1/orin_ros2_compute
[ -d venv ] && source venv/bin/activate
set -a; [ -f config/.env ] && . config/.env; set +a
export USE_MICROVIT=true
python src/robot1_ai_service_realtime.py --interval 5
```

**Keep this terminal open.** Expect:

- `✅ XML-RPC connected to Nano at http://10.13.68.184:8000`
- `Camera available: True`
- `✅ MQTT Connected successfully`
- `Starting continuous REALTIME detection every 5 seconds`

---

### Step 5: Helper Robot *(optional)*

**If you have a Helper Robot, SSH to it:**

```bash
ssh jetbot@<helper_robot_ip>
```

**Start the Helper AI service:**

```bash
cd ~/MicroVIT/helper_robot
[ -d venv ] && source venv/bin/activate
set -a; [ -f config/.env ] && . config/.env; set +a
# Ensure MQTT broker points to Controller: MQTT_BROKER_HOST=10.13.68.48
python src/helper_robot_ai_service.py
```

Or use systemd if installed:

```bash
sudo systemctl start helper-robot-ai.service
```

---

### Quick verification

- **Controller:** `ssh pi@10.13.68.48` → `sudo journalctl -u controller-ai.service -n 20` (MQTT, Ollama)
- **Nano:** `ssh jetbot@10.13.68.184` → `source /opt/ros/melodic/setup.bash` → `rosnode list` (includes `rplidarNode`, `nano_xmlrpc_server`) and `rostopic hz /scan` (~5–10 Hz)
- **Orin:** Robot1 AI terminal shows `Published REALTIME obstacle event` periodically

---

## Option A: One script from your Mac (after `./setup_ssh_keys.sh`)

```bash
./start_system.sh
```

Then start **Ollama on Orin** manually (see step 3 below) if the Orin AI service needs it.

---

## Option B: Manual start (each machine)

### 1. Controller Pi

```bash
ssh pi@10.13.68.48
sudo systemctl start mosquitto
sudo systemctl start controller-ai.service
systemctl is-active controller-ai.service   # should print: active
# Leave or exit; service keeps running.
```

---

### 2. Nano – ROS1 bringup (camera, LiDAR at 115200, motor, XML-RPC)

**Important:** You must `cd` to the workspace and `source devel/setup.bash` so `roslaunch` finds `jetbot_nano_bringup`. **REAL sensors only:** `use_lidar:=true` (real RPLIDAR), `use_camera:=true` (real camera), `use_odom_dummy:=false` (motor driver odom). Use **camera_device:=/dev/video1** if your camera is on video1.

```bash
ssh jetbot@10.13.68.184
source /opt/ros/melodic/setup.bash
cd ~/MicroVIT/robot1/nano_ros1_master
[ -d devel ] || catkin_make
source devel/setup.bash
roslaunch jetbot_nano_bringup nano_bringup_full.launch use_lidar:=true lidar_serial_baudrate:=115200 camera_device:=/dev/video0 use_odom_dummy:=false
```

**Note:** There is a **space** between `nano_bringup_full.launch` and `use_lidar:=true` (typo to avoid: `.launcuse_lidar`).

Leave this terminal running. You should see:
- `[JetBot Motor Driver] Serial connected to /dev/ttyACM0` (real odom from motor)
- `[Nano Camera] REAL camera publishing to /nano/camera/image_compressed` (real camera only; if no camera, node exits)
- `RPLIDAR S/N: ...` and `current scan mode: Sensitivity...` (real LiDAR)
- `[Autonomous Driver] Move forward at 0.20 m/s, stop when obstacle < 0.50 m`
- `Nano XML-RPC Server REALTIME Started on 0.0.0.0:8000`

**To disable autonomous drive** (manual/remote control only): add `use_autonomous:=false` to the roslaunch command.

**Robot won't resume after obstacle removed?** Only obstacles in the **front sector** (default ±45°) stop the robot; side/rear are ignored. If it still won't move, the LiDAR's "forward" may be misaligned — try `forward_angle_offset_deg:=180` in the launch, or widen the sector: `forward_sector_deg:=120`.

**If camera fails:** The camera node exits when no `/dev/video*` is available (REAL ONLY). Fix camera hardware (plug in, chmod 666 /dev/video0), then restart. For LiDAR ensure RPLIDAR connected at 115200. use_odom_dummy:=true only if motor has no encoders. See Camera not started below. If LiDAR driver isn’t failing. **If the camera does not start** (you see `⚠️ Camera ... not found/open`), see **Camera not started** below.

---

### 3. Orin – Ollama (Terminal A)

```bash
ssh jetbot@10.13.68.159
OLLAMA_NO_GPU=1 ollama serve
```

Leave this running (LLM for MicroViT and controller).

---

### 4. Orin – Robot1 AI service (Terminal B)

Open a **second** SSH to Orin:

```bash
ssh jetbot@10.13.68.159
cd ~/MicroVIT/robot1/orin_ros2_compute
[ -d venv ] && source venv/bin/activate
set -a; [ -f config/.env ] && . config/.env; set +a
export USE_MICROVIT=true
python src/robot1_ai_service_realtime.py --interval 5
```

Leave this running. You should see obstacle events and image/LiDAR logs.

---

## Quick check

- **Nano:** In another terminal: `ssh jetbot@10.13.68.184` → `source /opt/ros/melodic/setup.bash` → `rosnode list` (should include `rplidarNode`, `nano_xmlrpc_server`) and `rostopic hz /scan` (~5–10 Hz).
- **Controller:** `ssh pi@10.13.68.48` → `sudo journalctl -u controller-ai.service -n 20` (should show Ollama connected and MQTT).
- **Orin:** Robot1 AI terminal should show “Published REALTIME obstacle event” and no connection-refused errors.

---

## If the Nano workspace is missing

If `~/MicroVIT/robot1/nano_ros1_master` does not exist on the Nano, copy the repo from your Mac and build once:

```bash
# From your Mac (in the repo root)
scp -r "robot1/nano_ros1_master" jetbot@10.13.68.184:~/MicroVIT/robot1/
# On Nano
ssh jetbot@10.13.68.184
source /opt/ros/melodic/setup.bash
cd ~/MicroVIT/robot1/nano_ros1_master
catkin_make
source devel/setup.bash
# Then run the roslaunch from step 2.
```

---

## "Multiple files named nano_bringup_full.launch"

If `roslaunch` fails with:

```
RLException: multiple files named [nano_bringup_full.launch] in package [jetbot_nano_bringup]
```

Remove the duplicate (keep only the one in `launch/`):

```bash
rm -f ~/MicroVIT/robot1/nano_ros1_master/src/jetbot_nano_bringup/nano_bringup_full.launch
```

Then run `catkin_make` and `roslaunch` again.

---

## Camera not started (Nano)

If the Nano bringup shows **`⚠️ Camera /dev/video0 not found/open`** (and no `✅ Camera initialized`), do this **on the Nano**:

**1. List video devices and permissions**

```bash
ls -la /dev/video*
```

You should see at least `/dev/video0` (and sometimes `/dev/video1` for metadata). If there is no `/dev/video*`, the camera is not detected (unplug/replug USB camera, try another port).

**2. Fix permissions**

```bash
sudo chmod 666 /dev/video0
# If your camera is on video1:
sudo chmod 666 /dev/video1
```

To make this persistent, add yourself to the `video` group and re-login:

```bash
sudo usermod -a -G video $USER
# Then log out and SSH back in.
```

**3. Ensure nothing else is using the camera**

Only one process can open the camera. Stop any other roslaunch or camera test, then start bringup again.

**4. Try the other video device**

If the camera appears as `/dev/video1` (common when multiple USB devices exist), launch with:

```bash
roslaunch jetbot_nano_bringup nano_bringup_full.launch use_lidar:=true lidar_serial_baudrate:=115200 camera_device:=/dev/video1
```

**5. Run camera diagnostic (no ROS, no v4l2-ctl needed)**

From your Mac, copy the script to the Nano, then on the Nano:

```bash
# On Mac (from repo root):
scp tools/camera/test_camera_nano.py jetbot@10.13.68.184:~/

# On Nano:
python3 ~/test_camera_nano.py
```

The script lists `/dev/video*` and tries OpenCV on each index with V4L2 and default backend. It prints which device works (e.g. `video1 default: OK`). Use that in launch: `camera_device:=/dev/video1`. If all fail, you’ll see “open failed” or “read() failed”—fix permissions with `sudo chmod 666 /dev/video0` (and video1 if present), and ensure no other app is using the camera.

**6. Optional: install v4l-utils for more info**

```bash
sudo apt install v4l-utils
v4l2-ctl --list-devices
```

After fixing, restart the Nano bringup (step 2 in Option B). If you changed the camera device (e.g. to `/dev/video1`), add `camera_device:=/dev/video1` to the `roslaunch` command.

---

## Camera disconnects or "Cannot enable. Maybe the USB cable is bad?"

**dmesg shows:** `usb 1-2-port2: Cannot enable`, `USB disconnect`, `usb_suspend_both` — the camera works briefly then drops. This is usually **USB power** or **cable**, not a software bug.

**Software changes (already applied):**
- **320×240 resolution** — reduces USB bandwidth by ~4× vs 640×480; helps stability. Override: `camera_width:=640 camera_height:=480`.
- **USB autosuspend off** — run before bringup on Nano:
  ```bash
  scp tools/camera/disable_usb_autosuspend.sh jetbot@10.13.68.184:~/
  ssh jetbot@10.13.68.184 'chmod +x ~/disable_usb_autosuspend.sh && ~/disable_usb_autosuspend.sh'
  ```
  Then start `roslaunch` as usual.

**Hardware fixes (most important):**
1. **Powered USB hub** — the Nano often can’t supply enough power for camera + LiDAR + motors.
2. **Short, good-quality USB cable**
3. **Logitech C270/C920** — use a known UVC camera.
4. **Different USB port** — try another port on the Nano or hub.
5. **Node retries every 30 s** — replug the camera and wait; no need to restart roslaunch.

---

## Camera switches off or camera node crashes

**Note:** Camera runs in a separate node (`nano_camera_publisher`). When camera is lost, the node stays alive and retries every 30s; XML-RPC keeps running.

If the camera keeps failing:

1. **USB power**: The Nano’s USB ports may not deliver enough power for camera + LiDAR + motors. Use a **powered USB hub** or a camera with its own power.
2. **Copy full package** from Mac: `scp -r robot1/nano_ros1_master jetbot@10.13.68.184:~/MicroVIT/robot1/` then on Nano: `cd ~/MicroVIT/robot1/nano_ros1_master && catkin_make`
3. **Try a different USB port** on the Nano.
4. **Reduce load**: Run with `use_lidar:=false` to test if the camera stays on; if it does, USB power is likely the limit.

---

## AI message uses test image instead of real camera

If Robot1’s AI messages are generated from a **gray “FALLBACK TEST IMAGE”** instead of the real camera:

1. **Check Orin logs** (where `robot1_ai_service_realtime.py` runs). You should see one of:
   - `Using test image (no XML-RPC connection to Nano)` → Orin cannot reach the Nano (check `NANO_IP` in `.env`, network, and that Nano bringup is running).
   - `Nano camera returned failure: Camera not available on Nano` → Camera did not initialize on the Nano; follow **Camera not started (Nano)** above.
   - `Nano camera returned failure: Failed to capture frame from camera` → Camera opened but read failed; try permissions and ensuring no other process uses the camera.
   - `Image capture failed (Nano unreachable or RPC error): ...` → Connection/timeout to Nano; check Nano IP and that the XML-RPC server is running.

2. **On Orin startup** look for: `Camera available: True` (from Nano’s `get_robot_status`). If it says `False`, the Nano camera is not available; fix camera on Nano first, then restart Nano bringup and Orin AI service.

3. **Nano**: Ensure bringup shows `✅ Camera initialized on /dev/video0` (or your device). If you see `⚠️ Camera ... not found/open`, the log now includes **Last error:** (e.g. `isOpened()=False`, `read() failed`, or `Permission denied`). Use that to fix as in **Camera not started (Nano)**.

4. **Orin startup**: If camera is not available, the Orin now logs `Nano camera not available: <reason>` (the Nano’s `camera_message`). That reason is the same as **Last error** on the Nano and tells you why the camera node is not taking real images.

---

## Nano not passing data to Orin (no real camera/LiDAR)

The Orin **pulls** data from the Nano via XML-RPC (Orin calls Nano’s `get_camera_image()`, `get_lidar_data()`, etc.). If the Orin cannot connect, you get test image and fallback LiDAR and the log explains why.

**1. Start order**  
Start **Nano bringup first**, then start the Orin AI service. The Nano must be listening on `0.0.0.0:8000` before the Orin runs.

**2. Check Orin startup log**  
Look for these lines in order:

- `Attempting XML-RPC connection to Nano at 10.13.68.184:8000 (timeout 10s)...`
- Then either:
  - `✅ XML-RPC connected to Nano at http://10.13.68.184:8000` and `Camera available: True/False` → connection OK; if Camera is False, fix camera on Nano.
  - `❌ XML-RPC connection to Nano failed: ...` → see step 3.

**3. If connection failed**  
The log will show the error (e.g. `Connection refused`, `Timed out`, `No route to host`). From the **Orin** run:

```bash
# Replace with your Nano IP if different
export NANO_IP=10.13.68.184
ping -c 2 $NANO_IP
nc -zv $NANO_IP 8000
```

- If **ping** fails: Nano is off, wrong IP, or different network/VLAN. Fix IP (e.g. in Orin’s `.env` as `NANO_IP=...`) or network.
- If **nc** fails (connection refused): Nano bringup is not running, or port 8000 is blocked. On Nano, start `roslaunch jetbot_nano_bringup nano_bringup_full.launch ...` and ensure you see `Nano XML-RPC Server REALTIME Started on 0.0.0.0:8000`.
- If **nc** succeeds but Orin still fails: firewall on Nano may be blocking 8000. On Nano run `sudo ufw allow 8000` (if using ufw) or temporarily `sudo ufw disable` to test.

**4. Test script (on Orin)**  
From the repo (or copy the script to the Orin):

```bash
cd /path/to/MicroVIT_model_change   # or where you have tools/
export NANO_IP=10.13.68.184
export NANO_PORT=8000
python3 tools/test_nano_orin_connection.py
```

You should see `✅ SUCCESS: Orin can reach Nano.` If not, the script prints the same checks (ping, nc) and confirms whether the problem is network or Nano not running.

---

## Robot1 not generating AI messages (MQTT / Ollama / MicroViT)

If the Orin AI service runs but **no AI messages appear** on the Controller or MQTT monitor:

**1. MQTT broker is on the Controller, not on Orin**

The Robot1 AI service must connect to the **Controller’s** MQTT broker (10.13.68.48). If `MQTT_BROKER_HOST` is unset or `localhost`, messages are never published.

- **Check startup log** for: `MQTT broker 10.13.68.48:1883` (correct) vs `localhost:1883` (wrong).
- **Fix:** On Orin, either copy `config/.env.template` to `config/.env` with `MQTT_BROKER_HOST=10.13.68.48`, or ensure `config/device_config.yaml` has `mqtt.broker_host: "10.13.68.48"`.
- **Verify** Controller: `ssh pi@10.13.68.48` → `sudo systemctl status mosquitto` must be active.
- **Test from Orin:** `mosquitto_pub -h 10.13.68.48 -t test -m hello` (install `mosquitto-clients` if needed).

**2. Ollama model mismatch**

The service uses `qwen2.5:0.5b` (from `device_config.yaml` or `.env`). If it falls back to `phi3:mini` and that model is not installed, Ollama fails and you get fallback text instead of AI generation.

- **Check startup log** for: `Ollama ... model=qwen2.5:0.5b` (correct).
- **On Orin:** `ollama list` — ensure `qwen2.5:0.5b` is present. If not: `ollama pull qwen2.5:0.5b`.
- **Config:** `config/device_config.yaml` should have `ai.ollama_model: "qwen2.5:0.5b"` (default). Or set `OLLAMA_MODEL=qwen2.5:0.5b` in `config/.env`.

**3. MicroViT not enabled**

If `USE_MICROVIT=false` (or unset) and no VIT model, the service uses a simple fallback message (no real AI). MicroViT gives fast image preprocessing and proper AI messages.

- **Check startup log** for: `🚀 Using MicroViT model` → MicroViT enabled.
- **Fix:** Set `USE_MICROVIT=true` when starting, or ensure `config/device_config.yaml` has `ai.use_microvit: true`.

**4. MQTT not connected — explicit error**

Every cycle the service checks MQTT connection. If you see:
```text
❌ MQTT not connected — AI message NOT published
```
then the broker is unreachable. Ensure:
- Controller mosquitto is running.
- Orin can reach 10.13.68.48:1883 (`nc -zv 10.13.68.48 1883`).
- `MQTT_BROKER_HOST` or `device_config` points to 10.13.68.48.

**5. Watch messages from your Mac**

```bash
mosquitto_sub -h 10.13.68.48 -t "robots/#" -t "controller/#" -v
```

You should see `robots/jetson1/obstacle` with JSON payloads when Robot1 publishes.

---

## Controller: "Error getting AI decision ... Read timed out" / "No AI decision received"

The Controller uses **Orin's Ollama** for AI decisions (not local on Pi). If you see `HTTPConnectionPool(host='localhost', port=11434): Read timed out`:

1. **Controller was using localhost** — it should use Orin's Ollama. Update Controller config:
   ```bash
   # On Controller, create/update config/.env
   echo 'OLLAMA_HOST=http://10.13.68.159:11434' >> ~/MicroVIT/controller_rpi/config/.env
   echo 'OLLAMA_MODEL=qwen2.5:0.5b' >> ~/MicroVIT/controller_rpi/config/.env
   echo 'AI_TIMEOUT=60' >> ~/MicroVIT/controller_rpi/config/.env
   ```

2. **Update systemd** and restart:
   ```bash
   sudo cp ~/MicroVIT/controller_rpi/systemd/controller-ai.service /etc/systemd/system/
   sudo systemctl daemon-reload
   sudo systemctl restart controller-ai.service
   ```

3. **Ensure Ollama is running on Orin** (step 3 in startup) — Controller needs it for AI decisions.

4. **Network:** From Controller, `curl -s http://10.13.68.159:11434/api/tags` should return a JSON list of models.

---

## System recovery: nothing works (Nano nodes shutting, no AI messages)

Use this **minimal, stable** sequence when things are broken.

**1. Nano – start with LiDAR off (dummy LiDAR) so rplidar failures don’t spam**

On the Nano, use **dummy LiDAR** until the real LiDAR is fixed. The XML-RPC and motor service nodes now **respawn** if they crash.

```bash
ssh jetbot@10.13.68.184
source /opt/ros/melodic/setup.bash
cd ~/MicroVIT/robot1/nano_ros1_master
source devel/setup.bash
roslaunch jetbot_nano_bringup nano_bringup_full.launch use_lidar:=false camera_device:=/dev/video1
```

- Leave this running. You should see `✅ Camera initialized on /dev/video1` and `Nano XML-RPC Server REALTIME Started on 0.0.0.0:8000`.
- If a node exits, it will respawn (delay 2–3 s). If the whole launch exits, check the last error in the terminal.
- When your real LiDAR works, switch to: `use_lidar:=true lidar_serial_baudrate:=115200`.

**2. Orin – Ollama first, then AI service**

Terminal A (Ollama):

```bash
ssh jetbot@10.13.68.159
OLLAMA_NO_GPU=1 ollama serve
```

Terminal B (Robot1 AI):

```bash
ssh jetbot@10.13.68.159
cd ~/MicroVIT/robot1/orin_ros2_compute
source venv/bin/activate
export USE_MICROVIT=true
# Optional: copy .env from config if you use it
[ -f config/.env ] && set -a && . config/.env && set +a
python src/robot1_ai_service_realtime.py --interval 5
```

- You must see: `Attempting XML-RPC connection to Nano...` then either `✅ XML-RPC connected` or a clear error.
- Then: `Starting continuous REALTIME detection every 5 seconds`. After that, every 5 seconds you should see position, LiDAR, and either `✅ Generated AI message` / `Published REALTIME obstacle event` or `Detection cycle failed (will retry): ...`. If you never see “Starting continuous”, the process is failing during init (model load or Nano connection).

**3. If Nano and Orin are on different networks**

On the Orin, set the Nano IP before starting the AI service:

```bash
export NANO_IP=10.13.68.184
python src/robot1_ai_service_realtime.py --interval 5
```

**4. Quick checks**

- **Nano:** `rosnode list` should include `nano_xmlrpc_server`. If it disappears, check the Nano terminal for Python tracebacks; the node will respawn after a few seconds.
- **Orin:** From Orin run `ping 10.13.68.184` and `nc -zv 10.13.68.184 8000`. Both must succeed for real camera/LiDAR.
- **AI messages:** If you see “Using test image” and “LiDAR (FALLBACK DUMMY)” in the Orin log, the AI service is running but not talking to the Nano; fix Nano reachability and restart the Orin service.
- **Wrong NANO_IP on Orin:** If on the Orin `echo $NANO_IP` shows something other than `10.13.68.184` (e.g. `172.27.25.184`), the service now uses `config/device_config.yaml` (which has `nano_host: "10.13.68.184"`) when present. To fix the shell, run `export NANO_IP=10.13.68.184` or copy `config/.env.template` to `config/.env` with the correct Nano IP.
