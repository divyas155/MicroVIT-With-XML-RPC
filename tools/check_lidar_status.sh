#!/bin/bash
# Check LiDAR status on Nano
# Run this on Nano

echo "=========================================="
echo "LiDAR Status Check"
echo "=========================================="
echo ""

# 1. Check if roscore is running
echo "1. Checking ROS master..."
if rostopic list > /dev/null 2>&1; then
    echo "   ✅ roscore is running"
else
    echo "   ❌ roscore is not running"
    echo "   Start with: roscore"
    exit 1
fi
echo ""

# 2. Check what nodes are running
echo "2. Checking running nodes..."
NODES=$(rosnode list 2>/dev/null || echo "")
if echo "$NODES" | grep -q "rplidarNode"; then
    echo "   ✅ rplidarNode is running"
    RPLIDAR_RUNNING="yes"
elif echo "$NODES" | grep -q "nano_lidar_dummy"; then
    echo "   ⚠️  nano_lidar_dummy is running (dummy LiDAR)"
    RPLIDAR_RUNNING="no"
else
    echo "   ❌ No LiDAR node found"
    RPLIDAR_RUNNING="no"
fi
echo ""

# 3. Check /scan topic
echo "3. Checking /scan topic..."
if rostopic list | grep -q "^/scan$"; then
    echo "   ✅ /scan topic exists"
    
    # Check if publishing
    RATE=$(timeout 3 rostopic hz /scan 2>&1 | grep "average rate" | tail -1 | awk '{print $3}')
    if [ -n "$RATE" ]; then
        echo "   ✅ Publishing at ~$RATE Hz"
    else
        echo "   ❌ Not publishing (no new messages)"
    fi
else
    echo "   ❌ /scan topic not found"
fi
echo ""

# 4. Check rplidarNode process
echo "4. Checking rplidarNode process..."
if pgrep rplidarNode > /dev/null; then
    echo "   ✅ rplidarNode process is running (PID: $(pgrep rplidarNode))"
else
    echo "   ❌ rplidarNode process not running"
fi
echo ""

# 5. Summary and recommendations
echo "=========================================="
echo "Summary"
echo "=========================================="
echo ""

if [ "$RPLIDAR_RUNNING" == "yes" ] && [ -n "$RATE" ]; then
    echo "✅ LiDAR is working correctly!"
    echo "   Publishing at ~$RATE Hz"
elif [ "$RPLIDAR_RUNNING" == "no" ]; then
    echo "⚠️  Real LiDAR node not running"
    echo ""
    echo "Solution: Restart launch with real LiDAR enabled"
    echo ""
    echo "1. Stop current launch (Ctrl+C)"
    echo "2. Restart with:"
    echo "   roslaunch jetbot_nano_bringup nano_bringup_full.launch use_dummy_lidar:=false lidar_serial_baudrate:=115200"
    echo ""
    echo "Or copy updated launch file from development machine first"
else
    echo "❌ LiDAR node running but not publishing"
    echo ""
    echo "Possible issues:"
    echo "  1. USB port changed (check /dev/ttyUSB0)"
    echo "  2. LiDAR motor stopped spinning"
    echo "  3. Wrong baud rate"
    echo ""
    echo "Check:"
    echo "  ls -l /dev/ttyUSB0"
    echo "  pgrep rplidarNode"
    echo "  Check launch file logs for errors"
fi
echo ""
