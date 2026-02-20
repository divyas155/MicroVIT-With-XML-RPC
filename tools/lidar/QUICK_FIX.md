# Quick LiDAR Fix - Run This Now!

## Option 1: Copy Script to Nano (Recommended)

From your **development machine**, run:

```bash
# Copy script to Nano
scp tools/lidar/diagnose_lidar_nano.sh jetbot@10.13.68.184:~/

# SSH to Nano
ssh jetbot@10.13.68.184

# Run diagnostic
chmod +x diagnose_lidar_nano.sh
./diagnose_lidar_nano.sh
```

## Option 2: Copy-Paste Directly (If SSH copy doesn't work)

If you're **already SSH'd into the Nano**, you can create the script directly:

```bash
# On Nano, create the script
cat > ~/diagnose_lidar.sh << 'SCRIPT_END'
[paste the entire content of tools/lidar/diagnose_lidar_nano.sh here]
SCRIPT_END

chmod +x ~/diagnose_lidar.sh
~/diagnose_lidar.sh
```

## Option 3: Quick Manual Test (Fastest)

If you want to test quickly without the full script, run these commands **on the Nano**:

```bash
# 1. Make sure roscore is running (in one terminal)
roscore

# 2. Test baud rates one by one (in another terminal)
# Try 115200 first:
rosrun rplidar_ros rplidarNode _serial_port:=/dev/ttyUSB0 _serial_baudrate:=115200

# In a third terminal, check if it works:
rostopic echo /scan -n 1

# If no data, stop (Ctrl+C) and try 256000:
rosrun rplidar_ros rplidarNode _serial_port:=/dev/ttyUSB0 _serial_baudrate:=256000
```

## After Finding Correct Baud Rate

Once you know the working baud rate (e.g., 115200), **from your development machine**:

```bash
# Enable real LiDAR with correct baud rate
./tools/lidar/enable_real_lidar.sh enable 115200

# Then on Nano, launch:
roslaunch jetbot_nano_bringup nano_bringup_full.launch
```

## What the Script Does

1. ✅ Checks if `/dev/ttyUSB0` exists
2. ✅ Verifies permissions
3. ✅ Tests baud rates: 115200, 230400, 256000, 128000, 460800
4. ✅ Identifies which baud rate works
5. ✅ Verifies `/scan` topic publishes data
6. ✅ Provides next steps

## Expected Output

If successful, you'll see:
```
✅✅✅ SUCCESS! ✅✅✅

Working baud rate: 115200

Next steps:
1. Update your launch file to use this baud rate
2. Enable real LiDAR in launch file
3. Launch with the correct parameters
```
