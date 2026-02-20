# Complete System Start (Controller + Nano + Orin)

Use this order every time. **Your LiDAR needs 115200 baud** — use `lidar_serial_baudrate:=115200` on the Nano.

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

**Important:** You must `cd` to the workspace and `source devel/setup.bash` so `roslaunch` finds `jetbot_nano_bringup`. Use **115200** for LiDAR. Use **camera_device:=/dev/video1** if your USB camera is on video1 (avoids VIDEOIO errors). If LiDAR keeps failing (80008000), use **use_lidar:=false** so dummy LiDAR runs and nodes stay up (see **System recovery** below).

```bash
ssh jetbot@10.13.68.184
source /opt/ros/melodic/setup.bash
cd ~/MicroVIT/robot1/nano_ros1_master
[ -d devel ] || catkin_make
source devel/setup.bash
roslaunch jetbot_nano_bringup nano_bringup_full.launch use_lidar:=true lidar_serial_baudrate:=115200 camera_device:=/dev/video1
```

**Note:** There is a **space** between `nano_bringup_full.launch` and `use_lidar:=true` (typo to avoid: `.launcuse_lidar`).

Leave this terminal running. You should see:
- `[JetBot Motor Driver] Serial connected to /dev/ttyACM0`
- `✅ Camera initialized on /dev/video1` (or `/dev/video0`)
- `RPLIDAR S/N: ...` and `current scan mode: Sensitivity...` (or, if use_lidar:=false, dummy LiDAR)
- `Nano XML-RPC Server REALTIME] Started on 0.0.0.0:8000`

**If nodes keep exiting:** XML-RPC and motor service nodes now **respawn** after 2–3 s. If the whole launch still shuts down, try **use_lidar:=false** so the LiDAR driver isn’t failing. **If the camera does not start** (you see `⚠️ Camera ... not found/open`), see **Camera not started** below.

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
