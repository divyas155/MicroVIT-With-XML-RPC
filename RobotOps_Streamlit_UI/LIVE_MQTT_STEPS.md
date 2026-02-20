# Step-by-step: See real-time messages on the UI

Follow these steps to see **live** AI messages from your real system (Controller Pi, Nano, Orin) in the RobotOps Streamlit UI.

---

## Prerequisites

- Your **Controller Pi**, **Nano**, and **Orin** are on the same network as the machine where you run the Streamlit app (e.g. your Mac or laptop).
- **MQTT broker** runs on the Controller Pi (Mosquitto).
- Default IPs used below: Controller Pi **10.13.68.48**, Nano **10.13.68.184**, Orin **10.13.68.159**.

---

## Step 1: Start the real system (in order)

Start the system so robots and controller publish messages over MQTT.

### 1.1 Controller Pi – MQTT + Controller AI

On your Mac (or any terminal):

```bash
ssh pi@10.13.68.48
```

On the Pi:

```bash
sudo systemctl start mosquitto
sudo systemctl start controller-ai.service
systemctl is-active controller-ai.service   # should print: active
```

Leave the SSH session or exit; services keep running.

### 1.2 Nano – ROS1 bringup (camera, LiDAR, motor, XML-RPC)

Open a **new** terminal:

```bash
ssh jetbot@10.13.68.184
source /opt/ros/melodic/setup.bash
cd ~/MicroVIT/robot1/nano_ros1_master
source devel/setup.bash
roslaunch jetbot_nano_bringup nano_bringup_full.launch use_lidar:=true lidar_serial_baudrate:=115200
```

Leave this terminal running. You should see the motor driver, camera, LiDAR (or dummy), and XML-RPC server start.

### 1.3 Orin – Ollama (terminal A)

Open another terminal:

```bash
ssh jetbot@10.13.68.159
OLLAMA_NO_GPU=1 ollama serve
```

Leave it running.

### 1.4 Orin – Robot1 AI service (terminal B)

Open one more terminal:

```bash
ssh jetbot@10.13.68.159
cd ~/MicroVIT/robot1/orin_ros2_compute
[ -d venv ] && source venv/bin/activate
set -a; [ -f config/.env ] && . config/.env; set +a
export USE_MICROVIT=true
python src/robot1_ai_service_realtime.py --interval 5
```

Leave it running. This service publishes **obstacle events** to MQTT every 5 seconds (or your `--interval`). The Controller subscribes to those and publishes **AI decisions** back.

---

## Step 2: Run the Streamlit app (on your Mac/laptop)

On your **Mac** (or the machine that can reach the Controller Pi):

```bash
cd "/Users/divya/Documents/Documents/Thesis/independent system/RPC-XML sys 20 Jan/MicroVIT_model change/RobotOps_Streamlit_UI"
pip install -r requirements.txt   # once
streamlit run app.py
```

Open the URL shown in the terminal (e.g. **http://localhost:8501**).

---

## Step 3: Enable Live MQTT in the UI

1. In the **sidebar** (left), find **“Live system”**.
2. Check **“Use Live MQTT”**.
3. Set **MQTT broker host** to your Controller Pi IP (e.g. **10.13.68.48**).
4. Set **MQTT broker port** to **1883** (default).
5. If the broker is reachable, the sidebar will show **“Connected”** after a moment.

The app subscribes to:

- **robots/+/obstacle** – Robot1 (and Helper) obstacle events (natural message, LiDAR, etc.)
- **controller/ai_decision** – Controller AI decisions and messages

Incoming MQTT messages are converted to the UI message format and appended to the feed.

---

## Step 4: View real-time messages

- **Dashboard** – Tiles for Robot1, Controller, Helper show status and last message from **live** MQTT.
- **Live Feed** – Table updates as new messages arrive (page refreshes every 2 seconds when you’re on Live Feed). You see Robot1 obstacle reports and Controller AI messages.
- **Robot1 Detail** – Latest Robot1 message (from Orin + Nano) with AI perception, LiDAR, and suggested action.

You can leave **Simulator** off when using Live MQTT. If you turn the Simulator on as well, both simulated and live messages will appear in the same feed.

---

## Step 5: Optional – verify MQTT from the Pi

To confirm the Controller is publishing and the broker is reachable:

```bash
# On your Mac (install mosquitto clients if needed: brew install mosquitto)
mosquitto_sub -h 10.13.68.48 -t "robots/#" -t "controller/#" -v
```

You should see messages when the Orin Robot1 AI service and Controller are running. Stop with Ctrl+C.

---

## Troubleshooting

| Issue | What to do |
|-------|------------|
| Sidebar never shows “Connected” | Check Controller Pi IP and port (1883). From your Mac run `ping 10.13.68.48` and ensure no firewall blocks port 1883. |
| No messages in UI | Ensure Orin Robot1 AI service is running (`python src/robot1_ai_service_realtime.py --interval 5`). It publishes every 5 s. Check Controller logs: `ssh pi@10.13.68.48` then `sudo journalctl -u controller-ai.service -f`. |
| “Connection refused” | Start Mosquitto on the Pi: `sudo systemctl start mosquitto`. |
| UI shows old messages only | Live Feed auto-refreshes every 2 s. Go to another page and back, or refresh the browser. |

---

## Summary

1. Start **Controller** (MQTT + controller-ai).
2. Start **Nano** (full bringup with LiDAR at 115200).
3. Start **Orin** Ollama, then **Orin Robot1 AI** service.
4. Run **Streamlit** on your Mac: `streamlit run app.py`.
5. In the UI sidebar, enable **Use Live MQTT** and set broker to the Controller Pi IP.
6. Open **Dashboard** or **Live Feed** to see real-time messages.

No changes are required to your existing project code; the UI connects to the same MQTT broker the Controller and robots already use.
