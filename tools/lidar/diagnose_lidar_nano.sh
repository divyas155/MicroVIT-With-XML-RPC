#!/bin/bash
# Simple LiDAR Diagnostic Script for Jetson Nano
# Run this directly on the Nano (via SSH or monitor) to find the correct baud rate

set -e

LIDAR_DEVICE="/dev/ttyUSB0"
BAUD_RATES=(115200 230400 256000 128000 460800)

# Source ROS
source /opt/ros/melodic/setup.bash 2>/dev/null || {
    echo "❌ ERROR: ROS Melodic not found. Make sure ROS is installed."
    exit 1
}

echo "=========================================="
echo "LiDAR Diagnostic Test for Jetson Nano"
echo "=========================================="
echo ""

# Check 1: Device exists
echo "1. Checking device..."
if [ ! -e "$LIDAR_DEVICE" ]; then
    echo "   ❌ $LIDAR_DEVICE NOT FOUND"
    echo "   Available USB devices:"
    ls -l /dev/ttyUSB* 2>/dev/null || echo "   No /dev/ttyUSB* devices found"
    echo ""
    echo "   Try:"
    echo "   - Unplug and replug the LiDAR USB cable"
    echo "   - Check: dmesg | tail -20"
    exit 1
fi
echo "   ✅ Device found: $LIDAR_DEVICE"
ls -l "$LIDAR_DEVICE"
echo ""

# Check 2: Permissions
echo "2. Checking permissions..."
if [ -r "$LIDAR_DEVICE" ] && [ -w "$LIDAR_DEVICE" ]; then
    echo "   ✅ Device is readable/writable"
else
    echo "   ⚠️  Permission issue. Current user groups:"
    groups
    echo "   Trying to fix permissions..."
    sudo chmod 666 "$LIDAR_DEVICE" 2>/dev/null || {
        echo "   ❌ Could not fix permissions. Run: sudo chmod 666 $LIDAR_DEVICE"
    }
fi
echo ""

# Check 3: Kill any existing rplidarNode
echo "3. Cleaning up any existing LiDAR nodes..."
pkill -9 rplidarNode 2>/dev/null || true
pkill -9 roslaunch 2>/dev/null || true
sleep 2
echo "   ✅ Cleanup done"
echo ""

# Check 4: Start roscore if not running
echo "4. Checking ROS master..."
if ! rostopic list > /dev/null 2>&1; then
    echo "   ⚠️  roscore not running. Starting in background..."
    roscore > /tmp/roscore.log 2>&1 &
    ROSCORE_PID=$!
    sleep 3
    if ! rostopic list > /dev/null 2>&1; then
        echo "   ❌ Failed to start roscore"
        exit 1
    fi
    echo "   ✅ roscore started (PID: $ROSCORE_PID)"
else
    echo "   ✅ roscore already running"
    ROSCORE_PID=""
fi
echo ""

# Check 5: Physical check reminder
echo "=========================================="
echo "IMPORTANT: Physical Check"
echo "=========================================="
echo "Before testing, please verify:"
echo "  1. LiDAR motor is SPINNING (you should see/feel it rotating)"
echo "  2. LiDAR LED is ON (usually green/red)"
echo "  3. USB cable is securely connected"
echo ""
read -p "Is the LiDAR motor spinning? (y/n): " -n 1 -r
echo ""
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "⚠️  WARNING: If motor is not spinning, this is likely a hardware issue:"
    echo "   - Check power supply to LiDAR"
    echo "   - Try different USB port/cable"
    echo "   - Power cycle the LiDAR"
    echo ""
    read -p "Continue testing anyway? (y/n): " -n 1 -r
    echo ""
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        [ -n "$ROSCORE_PID" ] && kill $ROSCORE_PID 2>/dev/null || true
        exit 0
    fi
fi
echo ""

# Check 6: Test each baud rate
echo "=========================================="
echo "Testing Baud Rates"
echo "=========================================="
echo "Testing each baud rate for 10 seconds..."
echo "Looking for /scan topic to publish data..."
echo ""

SUCCESS_BAUD=""
for baud in "${BAUD_RATES[@]}"; do
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "Testing: $baud baud"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    
    # Kill any existing rplidarNode
    pkill -9 rplidarNode 2>/dev/null || true
    sleep 1
    
    # Start rplidarNode in background
    rosrun rplidar_ros rplidarNode \
        _serial_port:="$LIDAR_DEVICE" \
        _serial_baudrate:=$baud \
        > /tmp/lidar_test_${baud}.log 2>&1 &
    
    NODE_PID=$!
    echo "   Started rplidarNode (PID: $NODE_PID)"
    
    # Wait a bit for node to initialize
    sleep 5
    
    # Check if node is still running
    if ! ps -p $NODE_PID > /dev/null 2>&1; then
        echo "   ❌ Node crashed immediately"
        echo "   Error output:"
        tail -5 /tmp/lidar_test_${baud}.log 2>/dev/null || echo "   (No log available)"
        echo ""
        continue
    fi
    
    echo "   ✅ Node still running, checking for /scan data..."
    
    # Check if /scan topic exists and has data
    sleep 3
    if timeout 3 rostopic echo /scan -n 1 > /dev/null 2>&1; then
        echo "   ✅✅✅ SUCCESS! /scan topic is publishing!"
        echo ""
        echo "   Verifying scan data quality..."
        SCAN_DATA=$(timeout 2 rostopic echo /scan -n 1 2>/dev/null | head -5)
        if [ -n "$SCAN_DATA" ]; then
            echo "   ✅ Scan data looks good!"
            echo ""
            echo "   Sample scan header:"
            echo "$SCAN_DATA" | head -3
            echo ""
            SUCCESS_BAUD=$baud
            kill $NODE_PID 2>/dev/null || true
            break
        fi
    else
        echo "   ⚠️  Node running but /scan not publishing yet"
        echo "   Checking node output..."
        tail -3 /tmp/lidar_test_${baud}.log 2>/dev/null || echo "   (No recent output)"
    fi
    
    # Cleanup
    kill $NODE_PID 2>/dev/null || true
    sleep 1
    echo ""
done

# Final cleanup
pkill -9 rplidarNode 2>/dev/null || true
[ -n "$ROSCORE_PID" ] && kill $ROSCORE_PID 2>/dev/null || true

echo "=========================================="
echo "Diagnostic Results"
echo "=========================================="
echo ""

if [ -n "$SUCCESS_BAUD" ]; then
    echo "✅✅✅ SUCCESS! ✅✅✅"
    echo ""
    echo "Working baud rate: $SUCCESS_BAUD"
    echo ""
    echo "Next steps:"
    echo "1. Update your launch file to use this baud rate:"
    echo "   lidar_serial_baudrate:=$SUCCESS_BAUD"
    echo ""
    echo "2. Enable real LiDAR in launch file:"
    echo "   use_dummy_lidar:=false"
    echo ""
    echo "3. Launch with:"
    echo "   roslaunch jetbot_nano_bringup nano_bringup_full.launch \\"
    echo "       use_dummy_lidar:=false \\"
    echo "       lidar_serial_baudrate:=$SUCCESS_BAUD"
    echo ""
else
    echo "❌ No working baud rate found"
    echo ""
    echo "Possible issues:"
    echo "1. ❌ LiDAR motor not spinning (hardware issue)"
    echo "   → Check power supply, USB cable, try different port"
    echo ""
    echo "2. ❌ Wrong LiDAR model (not RPLidar)"
    echo "   → Check LiDAR brand/model, may need different driver"
    echo ""
    echo "3. ❌ Baud rate not in tested list"
    echo "   → Check LiDAR manual for correct baud rate"
    echo "   → Try: 9600, 38400, 57600, 115200, 128000, 230400, 256000"
    echo ""
    echo "4. ❌ USB-to-serial adapter issue"
    echo "   → Try: dmesg | grep ttyUSB0"
    echo "   → Try different USB port"
    echo ""
    echo "5. ❌ LiDAR needs initialization"
    echo "   → Some LiDARs need a specific startup sequence"
    echo ""
    echo "Check logs:"
    echo "   ls -l /tmp/lidar_test_*.log"
    echo "   cat /tmp/lidar_test_256000.log  # Example"
fi

echo ""
