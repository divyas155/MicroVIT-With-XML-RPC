# LiDAR Troubleshooting Guide

**Same issue (80008000 / RESULT_OPERATION_TIMEOUT)?** See **docs/LIDAR_RESOLVE.md** for the full flow (power, baud, permissions, detailed checkup script).

## Quick Diagnosis

If your LiDAR (`/scan` topic) is not publishing data, follow these steps:

### Step 1: Run Diagnostic

**Recommended – detailed checkup (on Nano):**
```bash
# From Mac: scp tools/lidar/detailed_lidar_checkup.sh jetbot@10.13.68.184:~/
# On Nano:
bash ~/detailed_lidar_checkup.sh
cat /tmp/lidar_checkup_report.txt
```
This checks device, USB, raw read, and rplidarNode at 256000/115200/… and reports power vs baud.

**Or – baud-only test:**  
On Nano: `./tools/lidar/diagnose_lidar_nano.sh` or `./tools/lidar/test_all_bauds.sh` (tests 115200, 256000, etc.).

### Step 2: Physical Check

**Before/during diagnostics, verify:**

1. **LiDAR motor spinning** – The disc must rotate. If not:
   - Many RPLidars need **external 5V** (USB is not enough). See **docs/lidar_power_fix.md**.
   - Try another USB port/cable; power cycle (unplug/replug).
2. **LED** – LiDAR LED should be ON when powered.
3. **USB** – Cable firmly connected.

### Step 3: Enable Real LiDAR

Default baud in this repo is **256000**. If your unit needs **115200**, use that.

**Option A: Helper script**
```bash
./tools/lidar/enable_real_lidar.sh enable 256000   # or 115200
```

**Option B: Launch file**  
Use `nano_bringup_full.launch` with `use_lidar:=true` and `lidar_serial_baudrate:=256000` (or 115200).

**Option C: Command line**
```bash
roslaunch jetbot_nano_bringup nano_bringup_full.launch use_lidar:=true lidar_serial_baudrate:=256000
# or lidar_serial_baudrate:=115200
```

### Step 4: Verify It Works

After launching, check:

```bash
# Check if /scan topic exists
rostopic list | grep scan

# Check if data is publishing
rostopic hz /scan

# View scan data
rostopic echo /scan -n 1
```

## Common Issues and Solutions

### Issue 1: "No new messages" on `/scan`

**Symptoms:**
- `rostopic hz /scan` shows "no new messages"
- `rplidarNode` exits with code 255

**Causes:**
1. Wrong baud rate (most common)
2. LiDAR motor not spinning
3. USB connection issue

**Solution:**
1. Run diagnostic script to find correct baud rate
2. Check physical hardware (motor spinning, LED on)
3. Try different USB port/cable

### Issue 2: Permission Denied

**Symptoms:**
- Cannot open `/dev/ttyUSB0`
- Error: "Permission denied"

**Solution:**
```bash
# Add user to dialout group (if not already)
sudo usermod -a -G dialout $USER
# Log out and back in, or:
newgrp dialout

# Or temporarily fix permissions
sudo chmod 666 /dev/ttyUSB0
```

### Issue 3: Device Not Found

**Symptoms:**
- `/dev/ttyUSB0` doesn't exist
- `ls /dev/ttyUSB*` shows nothing

**Solution:**
```bash
# Check USB devices
lsusb
dmesg | tail -20

# Try unplugging and replugging USB cable
# Check if device appears as /dev/ttyUSB1 or /dev/ttyACM0
```

### Issue 4: Wrong LiDAR Model

**Symptoms:**
- All baud rates fail
- Motor is spinning but no data

**Solution:**
- Verify you have an RPLidar (A1, A2, A3, S1, etc.)
- Check LiDAR manual for correct baud rate
- Some LiDARs need different ROS drivers (not `rplidar_ros`)

### Issue 5: rplidarNode Crashes Immediately

**Symptoms:**
- Node starts then exits with code 255
- Error: "SDK RESULT_OPERATION_TIMEOUT"

**Solution:**
1. Wrong baud rate → Run diagnostic script
2. Hardware issue → Check motor, power, cable
3. USB adapter issue → Try different USB port

## Manual Testing

If the diagnostic script doesn't work, test manually:

```bash
# 1. Start roscore
roscore

# 2. In another terminal, test each baud rate:
rosrun rplidar_ros rplidarNode _serial_port:=/dev/ttyUSB0 _serial_baudrate:=115200

# 3. In a third terminal, check for data:
rostopic echo /scan -n 1

# 4. If no data, stop (Ctrl+C) and try next baud rate:
rosrun rplidar_ros rplidarNode _serial_port:=/dev/ttyUSB0 _serial_baudrate:=256000
```

## Common RPLidar Baud Rates

| Model | Typical Baud Rate |
|-------|------------------|
| RPLidar A1 | 115200 |
| RPLidar A2 | 256000 |
| RPLidar A3 | 256000 |
| RPLidar S1 | 256000 |

**Note:** Some clones or variants may use different rates. Always check your specific model's documentation.

## Configuration Files

### Launch File Location
- Main: `robot1/nano_ros1_master/launch/nano_bringup_full.launch`
- Package: `robot1/nano_ros1_master/src/jetbot_nano_bringup/launch/nano_bringup_full.launch`

### Key Parameters
```xml
<arg name="use_lidar" default="true" />  <!-- false = dummy LiDAR -->
<arg name="lidar_serial_port" default="/dev/ttyUSB0" />
<arg name="lidar_serial_baudrate" default="256000" />  <!-- try 115200 if timeout/80008000 -->
```

## Still Having Issues?

1. **Check logs:**
   ```bash
   # View rplidarNode output
   rosnode info /rplidarNode
   
   # Check system logs
   dmesg | grep -i usb
   ```

2. **Verify ROS installation:**
   ```bash
   rospack find rplidar_ros
   ```

3. **Test USB-to-serial adapter:**
   ```bash
   # Check adapter info
   dmesg | grep -i "cp210\|ch340\|ftdi"
   ```

4. **Try different USB port** (some ports may have power issues)

5. **Check LiDAR power requirements** (some need external 5V supply)
