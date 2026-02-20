# Launch Complete System - Step by Step

## ✅ Prerequisites Verified
- ✅ LiDAR working at 115200 baud
- ✅ Launch file configured with real LiDAR
- ✅ `/scan` topic publishing at ~7 Hz

## Launch Steps

### Step 1: Stop Any Running Processes (Nano)

**On Nano, run:**
```bash
pkill -9 roscore; pkill -9 rosmaster; pkill -9 rplidarNode; pkill -9 roslaunch; killall -9 rosmaster 2>/dev/null; echo "✅ Cleaned up"
```

### Step 2: Start ROS Master (Nano - Terminal 1)

**SSH to Nano:**
```bash
ssh jetbot@10.13.68.184
```

**Start roscore:**
```bash
cd ~/MicroVIT/robot1/nano_ros1_master
source /opt/ros/melodic/setup.bash
source devel/setup.bash
roscore
```

**Keep this terminal open!** You should see:
```
started roslaunch server http://nano-4gb-jp45:11311/
```

### Step 3: Start Complete Nano Bringup (Nano - Terminal 2)

**Open a NEW terminal, SSH to Nano:**
```bash
ssh jetbot@10.13.68.184
```

**Launch the complete system:**
```bash
cd ~/MicroVIT/robot1/nano_ros1_master
source /opt/ros/melodic/setup.bash
source devel/setup.bash
roslaunch jetbot_nano_bringup nano_bringup_full.launch
```

**Expected Output:**
```
[jetbot_motor_driver-1] process started with pid [xxxxx]
[rplidarNode-2] process started with pid [xxxxx]
[nano_xmlrpc_server-3] process started with pid [xxxxx]
[nano_motor_service_server-4] process started with pid [xxxxx]
[base_to_laser-5] process started with pid [xxxxx]

[Nano REALTIME] ✅ Camera initialized on /dev/video0 (MJPEG 640x480)
[Nano XML-RPC Server REALTIME] Started on 0.0.0.0:8000
RPLIDAR S/N: C19CEDF9C7E29BD1A7E39EF2514F431B
[ INFO] Firmware Ver: 1.29
[ INFO] Hardware Rev: 7
[ INFO] RPLidar health status : 0
[ INFO] current scan mode: Sensitivity, max_distance: 12.0 m
```

**✅ Verify these are running:**
- ✅ Motor driver
- ✅ **Real LiDAR (rplidarNode)** - should show serial number and firmware
- ✅ XML-RPC server on port 8000
- ✅ Camera initialized

**Keep this terminal open!**

### Step 4: Verify LiDAR is Working (Nano - Terminal 3, Optional)

**Open another terminal to verify:**
```bash
ssh jetbot@10.13.68.184
source /opt/ros/melodic/setup.bash

# Check /scan topic
rostopic hz /scan
# Should show ~7 Hz

# View scan data
rostopic echo /scan -n 1
# Should show LaserScan message with ranges array
```

### Step 5: Start Ollama (Orin - Terminal 4)

**SSH to Orin:**
```bash
ssh jetbot@10.13.68.159
```

**Start Ollama:**
```bash
OLLAMA_NO_GPU=1 ollama serve
```

**Keep this terminal open!** You should see:
```
2024/... INFO server config env="map[OLLAMA_NO_GPU:1]"
```

### Step 6: Start Orin AI Service (Orin - Terminal 5)

**Open a NEW terminal, SSH to Orin:**
```bash
ssh jetbot@10.13.68.159
```

**Start AI Service:**
```bash
cd ~/MicroVIT/robot1/orin_ros2_compute
source venv/bin/activate 2>/dev/null || (python3 -m venv venv && source venv/bin/activate && pip install -r requirements.txt)
export NANO_IP=10.13.68.184
export NANO_PORT=8000
export OLLAMA_NO_GPU=1
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
   LiDAR available: True  ← Should show True now!
✅ MQTT Connected successfully
✅ Ollama connected. Available models: ['qwen2.5:0.5b']
🚀 Using MicroViT model for fast image preprocessing
✅ MicroViT model loaded successfully!
🤖 Robot1 AI Service started in REALTIME MODE
Starting continuous REALTIME detection every 30 seconds
✅ Captured REAL image from Nano (640x480)
✅ Generated AI message with MicroViT+Ollama (FULL):
[AI analysis appears here]
```

**✅ Key indicators:**
- ✅ `LiDAR available: True` (should now show True!)
- ✅ `Captured REAL image from Nano`
- ✅ AI messages being generated

**Keep this terminal open to monitor AI messages!**

## Verification Checklist

### Nano Services
- [ ] roscore running
- [ ] Motor driver running
- [ ] **Real LiDAR (rplidarNode) running** - check for serial number in logs
- [ ] XML-RPC server running on port 8000
- [ ] Camera initialized
- [ ] `/scan` topic publishing at ~7 Hz

### Orin Services
- [ ] Ollama running
- [ ] AI service connected to Nano
- [ ] LiDAR data available (should show `True`)
- [ ] AI messages being generated

## Troubleshooting

### If LiDAR not showing in AI service:
1. Check Nano logs - should see rplidarNode with serial number
2. Verify `/scan` topic: `rostopic hz /scan`
3. Check XML-RPC server can read LiDAR: Look for "LiDAR available: True"

### If rplidarNode not starting:
- Check device: `ls -l /dev/ttyUSB0`
- Verify baud rate is 115200 in launch file
- Check motor is spinning (physical check)

## Quick Commands Reference

**Kill all ROS processes:**
```bash
pkill -9 roscore; pkill -9 rosmaster; pkill -9 rplidarNode; pkill -9 roslaunch; killall -9 rosmaster 2>/dev/null
```

**Check if services are running:**
```bash
# On Nano
pgrep rosmaster && echo "roscore running" || echo "roscore not running"
pgrep rplidarNode && echo "LiDAR running" || echo "LiDAR not running"
pgrep -f xmlrpc && echo "XML-RPC running" || echo "XML-RPC not running"

# Check /scan topic
rostopic hz /scan
```

## Summary

Your complete system should now be running with:
- ✅ Real LiDAR (115200 baud) publishing `/scan` at ~7 Hz
- ✅ Motor driver ready
- ✅ Camera streaming
- ✅ XML-RPC server providing data to Orin
- ✅ AI service processing real sensor data

The system is now using **REAL LiDAR data** instead of dummy data!
