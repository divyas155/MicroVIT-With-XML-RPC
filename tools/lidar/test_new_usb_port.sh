#!/bin/bash
# Test LiDAR with new USB port
# Run this on Nano after changing USB port

echo "=========================================="
echo "Testing LiDAR with New USB Port"
echo "=========================================="
echo ""

# 1. Check which USB device is now connected
echo "1. CHECKING USB DEVICES"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Available USB serial devices:"
ls -l /dev/ttyUSB* 2>/dev/null || echo "  No /dev/ttyUSB* devices found"
echo ""

# Check dmesg for recent USB connections
echo "Recent USB connections:"
dmesg | tail -20 | grep -i "usb\|tty\|cp210\|ch340" | tail -5
echo ""

# 2. Determine which device to use
if [ -e "/dev/ttyUSB0" ]; then
    LIDAR_DEVICE="/dev/ttyUSB0"
    echo "✅ Using /dev/ttyUSB0"
elif [ -e "/dev/ttyUSB1" ]; then
    LIDAR_DEVICE="/dev/ttyUSB1"
    echo "✅ Using /dev/ttyUSB1 (ttyUSB0 not found)"
else
    echo "❌ No USB serial device found!"
    echo "   Please check:"
    echo "   1. USB cable is connected"
    echo "   2. LiDAR is powered"
    echo "   3. Run: dmesg | tail -20"
    exit 1
fi

echo "Device: $LIDAR_DEVICE"
ls -l "$LIDAR_DEVICE"
echo ""

# 3. Check permissions
echo "2. CHECKING PERMISSIONS"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if [ -r "$LIDAR_DEVICE" ] && [ -w "$LIDAR_DEVICE" ]; then
    echo "✅ Device is readable/writable"
else
    echo "⚠️  Permission issue. Fixing..."
    sudo chmod 666 "$LIDAR_DEVICE" 2>/dev/null || {
        echo "❌ Could not fix. Run: sudo chmod 666 $LIDAR_DEVICE"
    }
fi
echo ""

# 4. Physical check reminder
echo "3. PHYSICAL CHECK"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "After changing USB port, please verify:"
echo "  □ LiDAR motor is SPINNING (disc rotating)"
echo "  □ LiDAR LED is ON"
echo "  □ USB cable is securely connected"
echo ""
read -p "Is the motor spinning NOW? (y/n): " -n 1 -r
MOTOR_SPINNING=$REPLY
echo ""
echo ""

# 5. Test raw data read
echo "4. RAW DATA TEST"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Testing if LiDAR sends data (5 seconds)..."
echo ""

# Test multiple baud rates
BAUD_RATES=(115200 256000 230400)
FOUND_DATA="no"

for baud in "${BAUD_RATES[@]}"; do
    echo -n "Testing $baud baud... "
    stty -F "$LIDAR_DEVICE" $baud cs8 -cstopb -parenb raw -echo 2>/dev/null
    
    RAW_DATA=$(timeout 3 cat "$LIDAR_DEVICE" 2>&1 | head -c 100)
    BYTES=${#RAW_DATA}
    
    if [ "$BYTES" -gt 10 ]; then
        echo "✅ DATA FOUND! ($BYTES bytes)"
        echo "   First 32 bytes:"
        echo "$RAW_DATA" | head -c 32 | hexdump -C | head -2
        FOUND_DATA="yes"
        WORKING_BAUD=$baud
        break
    else
        echo "❌ No data"
    fi
done
echo ""

# 6. Test with rplidarNode
echo "5. RPLIDAR NODE TEST"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
source /opt/ros/melodic/setup.bash 2>/dev/null || {
    echo "❌ ROS not found"
    exit 1
}

pkill -9 rplidarNode 2>/dev/null || true
if ! rostopic list > /dev/null 2>&1; then
    echo "Starting roscore..."
    roscore > /tmp/roscore.log 2>&1 &
    sleep 3
fi

# Use the baud rate that found data, or default to 256000
TEST_BAUD=${WORKING_BAUD:-256000}
echo "Testing rplidarNode at $TEST_BAUD baud (10 seconds)..."
echo ""

rosrun rplidar_ros rplidarNode \
    _serial_port:="$LIDAR_DEVICE" \
    _serial_baudrate:=$TEST_BAUD \
    > /tmp/rplidar_new_port.log 2>&1 &
NODE_PID=$!

sleep 10

if ps -p $NODE_PID > /dev/null 2>&1; then
    echo "✅ Node still running"
    
    # Check for /scan data
    sleep 2
    if timeout 3 rostopic echo /scan -n 1 > /dev/null 2>&1; then
        echo "✅✅✅ SUCCESS! /scan topic has data!"
        echo ""
        echo "Sample scan:"
        timeout 2 rostopic echo /scan -n 1 | head -5
    else
        echo "⚠️  Node running but /scan not publishing"
        echo ""
        echo "Node output:"
        tail -5 /tmp/rplidar_new_port.log
    fi
else
    echo "❌ Node exited/crashed"
    echo ""
    echo "Error log:"
    cat /tmp/rplidar_new_port.log
fi

kill $NODE_PID 2>/dev/null || true
pkill -9 rplidarNode 2>/dev/null || true
echo ""

# 7. Summary
echo "=========================================="
echo "TEST SUMMARY"
echo "=========================================="
echo ""

if [[ ! $MOTOR_SPINNING =~ ^[Yy]$ ]]; then
    echo "🔴 Motor still NOT spinning"
    echo ""
    echo "Even with new USB port, motor not spinning = POWER ISSUE"
    echo ""
    echo "SOLUTION: Connect external 5V power supply"
    echo "  → USB ports typically provide only 500mA"
    echo "  → Most RPLidars need 800mA-1A+"
    echo "  → External power is required"
elif [ "$FOUND_DATA" == "yes" ]; then
    echo "🟢 SUCCESS! Data found at $WORKING_BAUD baud"
    echo ""
    echo "New USB port is working!"
    echo ""
    echo "Next steps:"
    echo "1. Update launch file to use: $LIDAR_DEVICE"
    echo "2. Use baud rate: $WORKING_BAUD"
    echo "3. Enable real LiDAR:"
    echo "   ./tools/lidar/enable_real_lidar.sh enable $WORKING_BAUD"
elif ps -p $NODE_PID > /dev/null 2>&1 || timeout 2 rostopic echo /scan -n 1 > /dev/null 2>&1; then
    echo "🟢 SUCCESS! rplidarNode working!"
    echo ""
    echo "New USB port resolved the issue!"
else
    echo "🟡 Still having issues"
    echo ""
    echo "Possible causes:"
    echo "  1. Motor not spinning (power issue)"
    echo "  2. Wrong baud rate (try: 9600, 38400, 57600)"
    echo "  3. Wrong LiDAR model"
fi

echo ""
echo "Device used: $LIDAR_DEVICE"
echo "Check logs: cat /tmp/rplidar_new_port.log"
echo ""
