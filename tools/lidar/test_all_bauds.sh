#!/bin/bash
# Quick baud rate tester - run this on Nano
# Tests all common baud rates and shows which one works

source /opt/ros/melodic/setup.bash 2>/dev/null || {
    echo "❌ ROS not found. Source ROS first: source /opt/ros/melodic/setup.bash"
    exit 1
}

LIDAR_DEVICE="/dev/ttyUSB0"
BAUD_RATES=(115200 256000 230400 128000 460800 9600 38400 57600)

echo "=========================================="
echo "Testing All Baud Rates"
echo "=========================================="
echo "Device: $LIDAR_DEVICE"
echo ""

# Kill any existing nodes
pkill -9 rplidarNode 2>/dev/null || true

# Start roscore if needed
if ! rostopic list > /dev/null 2>&1; then
    echo "Starting roscore..."
    roscore > /tmp/roscore.log 2>&1 &
    sleep 3
fi

SUCCESS=""

for baud in "${BAUD_RATES[@]}"; do
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "Testing: $baud baud"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    
    # Kill any existing
    pkill -9 rplidarNode 2>/dev/null || true
    sleep 1
    
    # Start node
    timeout 8 rosrun rplidar_ros rplidarNode \
        _serial_port:="$LIDAR_DEVICE" \
        _serial_baudrate:=$baud \
        > /tmp/lidar_${baud}.log 2>&1 &
    
    NODE_PID=$!
    sleep 6
    
    # Check if still running
    if ps -p $NODE_PID > /dev/null 2>&1; then
        # Check for /scan data
        if timeout 2 rostopic echo /scan -n 1 > /dev/null 2>&1; then
            echo "   ✅✅✅ SUCCESS! Working baud rate: $baud"
            SUCCESS=$baud
            kill $NODE_PID 2>/dev/null || true
            break
        else
            echo "   ⚠️  Node running but no /scan data"
        fi
        kill $NODE_PID 2>/dev/null || true
    else
        echo "   ❌ Node crashed/timeout"
        # Show error
        if [ -f /tmp/lidar_${baud}.log ]; then
            grep -i "error\|timeout\|fail" /tmp/lidar_${baud}.log | head -2 || echo "   (No specific error)"
        fi
    fi
    
    sleep 1
done

echo ""
echo "=========================================="
if [ -n "$SUCCESS" ]; then
    echo "✅ FOUND WORKING BAUD RATE: $SUCCESS"
    echo ""
    echo "To enable real LiDAR, run (from dev machine):"
    echo "  ./tools/lidar/enable_real_lidar.sh enable $SUCCESS"
else
    echo "❌ No working baud rate found"
    echo ""
    echo "Possible issues:"
    echo "  1. LiDAR motor not spinning (hardware)"
    echo "  2. Wrong LiDAR model (not RPLidar)"
    echo "  3. USB/power issue"
    echo ""
    echo "Check logs: cat /tmp/lidar_*.log"
fi
echo "=========================================="

# Cleanup
pkill -9 rplidarNode 2>/dev/null || true
