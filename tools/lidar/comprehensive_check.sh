#!/bin/bash
# Comprehensive LiDAR Diagnostic - Checks Everything Systematically
# Run this on Nano

set -e

LIDAR_DEVICE="/dev/ttyUSB0"

echo "=========================================="
echo "COMPREHENSIVE LiDAR DIAGNOSTIC"
echo "=========================================="
echo ""

# 1. System Information
echo "1. SYSTEM INFORMATION"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "OS: $(cat /etc/os-release | grep PRETTY_NAME | cut -d'"' -f2)"
echo "Kernel: $(uname -r)"
echo "Architecture: $(uname -m)"
echo "User: $(whoami)"
echo "Groups: $(groups)"
echo ""

# 2. ROS Installation Check
echo "2. ROS INSTALLATION CHECK"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if [ -f /opt/ros/melodic/setup.bash ]; then
    echo "✅ ROS Melodic found"
    source /opt/ros/melodic/setup.bash
    echo "ROS_DISTRO: $ROS_DISTRO"
    echo "ROS_VERSION: $ROS_VERSION"
else
    echo "❌ ROS Melodic not found!"
    exit 1
fi
echo ""

# 3. rplidar_ros Package Check
echo "3. RPLIDAR_ROS PACKAGE CHECK"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if rospack find rplidar_ros > /dev/null 2>&1; then
    RPLIDAR_PATH=$(rospack find rplidar_ros)
    echo "✅ rplidar_ros package found"
    echo "   Path: $RPLIDAR_PATH"
    
    # Check SDK version in source
    if [ -f "$RPLIDAR_PATH/src/sdk/include/rplidar.h" ]; then
        SDK_VERSION=$(grep -i "version\|SDK" "$RPLIDAR_PATH/src/sdk/include/rplidar.h" | head -3 || echo "Not found")
        echo "   SDK info: $SDK_VERSION"
    fi
    
    # Check if node exists
    if [ -f "$RPLIDAR_PATH/rplidarNode" ] || command -v rplidarNode > /dev/null 2>&1; then
        echo "✅ rplidarNode executable found"
    else
        echo "⚠️  rplidarNode executable not found in expected location"
    fi
else
    echo "❌ rplidar_ros package NOT found!"
    echo "   Install with: sudo apt-get install ros-melodic-rplidar-ros"
    exit 1
fi
echo ""

# 4. USB Device Check
echo "4. USB DEVICE CHECK"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if [ -e "$LIDAR_DEVICE" ]; then
    echo "✅ Device exists: $LIDAR_DEVICE"
    ls -l "$LIDAR_DEVICE"
    
    # Check permissions
    if [ -r "$LIDAR_DEVICE" ] && [ -w "$LIDAR_DEVICE" ]; then
        echo "✅ Device is readable and writable"
    else
        echo "❌ Permission issue!"
        echo "   Current permissions: $(stat -c '%a' $LIDAR_DEVICE)"
        echo "   Fix with: sudo chmod 666 $LIDAR_DEVICE"
    fi
else
    echo "❌ Device NOT FOUND: $LIDAR_DEVICE"
    echo "   Available USB devices:"
    ls -l /dev/ttyUSB* 2>/dev/null || echo "   None found"
    exit 1
fi
echo ""

# 5. USB Hardware Details
echo "5. USB HARDWARE DETAILS"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "USB device info:"
USB_INFO=$(dmesg | grep -i "ttyUSB0\|cp210\|ch340\|ftdi" | tail -10)
if [ -n "$USB_INFO" ]; then
    echo "$USB_INFO"
else
    echo "⚠️  No USB messages found in dmesg"
fi
echo ""
echo "lsusb output:"
lsusb | grep -i "serial\|cp210\|ch340\|ftdi" || lsusb | tail -5
echo ""

# 6. Process Lock Check
echo "6. PROCESS LOCK CHECK"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
pkill -9 rplidarNode 2>/dev/null || true
sleep 1

# Check using fuser (more reliable than lsof)
if command -v fuser > /dev/null 2>&1; then
    if fuser "$LIDAR_DEVICE" 2>/dev/null; then
        echo "⚠️  Device is locked by process:"
        fuser -v "$LIDAR_DEVICE" 2>/dev/null || echo "   (Could not get details)"
    else
        echo "✅ Device is free (not locked)"
    fi
else
    echo "⚠️  fuser not available, checking manually..."
    if ls -l /proc/*/fd 2>/dev/null | grep -q "$LIDAR_DEVICE"; then
        echo "⚠️  Device appears to be in use"
    else
        echo "✅ Device appears free"
    fi
fi
echo ""

# 7. Serial Port Configuration Test
echo "7. SERIAL PORT CONFIGURATION TEST"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
for baud in 115200 256000; do
    echo "Testing configuration at $baud baud..."
    if stty -F "$LIDAR_DEVICE" $baud cs8 -cstopb -parenb raw -echo 2>&1 | grep -q "error\|invalid"; then
        echo "   ❌ Failed to configure at $baud"
    else
        echo "   ✅ Configuration successful at $baud"
        # Check current settings
        CURRENT_BAUD=$(stty -F "$LIDAR_DEVICE" 2>&1 | grep -o "speed [0-9]*" | cut -d' ' -f2 || echo "unknown")
        echo "      Current speed: $CURRENT_BAUD"
    fi
done
echo ""

# 8. Raw Data Read Test (Multiple Baud Rates)
echo "8. RAW DATA READ TEST (Multiple Baud Rates)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
BAUD_RATES=(115200 256000 230400 128000)
HAS_DATA="no"

for baud in "${BAUD_RATES[@]}"; do
    echo "Testing $baud baud (3 seconds)..."
    stty -F "$LIDAR_DEVICE" $baud cs8 -cstopb -parenb raw -echo 2>/dev/null
    
    RAW_DATA=$(timeout 3 cat "$LIDAR_DEVICE" 2>&1 | head -c 100)
    BYTES=${#RAW_DATA}
    
    if [ "$BYTES" -gt 0 ]; then
        echo "   ✅ Received $BYTES bytes at $baud baud!"
        echo "   First 32 bytes (hex):"
        echo "$RAW_DATA" | head -c 32 | hexdump -C | head -2
        HAS_DATA="yes"
        WORKING_BAUD=$baud
        break
    else
        echo "   ❌ No data at $baud baud"
    fi
    sleep 1
done

if [ "$HAS_DATA" == "no" ]; then
    echo ""
    echo "⚠️⚠️⚠️ CRITICAL: NO DATA from LiDAR at any baud rate!"
    echo "   This indicates:"
    echo "   1. Motor not spinning (power issue) - MOST LIKELY"
    echo "   2. LiDAR not powered"
    echo "   3. Wrong LiDAR model"
    echo "   4. Hardware failure"
fi
echo ""

# 9. ROS Node Test with Detailed Output
echo "9. ROS NODE TEST (Detailed)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
pkill -9 rplidarNode 2>/dev/null || true
pkill -9 roslaunch 2>/dev/null || true
sleep 2

# Start roscore if needed
if ! rostopic list > /dev/null 2>&1; then
    echo "Starting roscore..."
    roscore > /tmp/roscore_diag.log 2>&1 &
    ROSCORE_PID=$!
    sleep 4
    if ! rostopic list > /dev/null 2>&1; then
        echo "❌ Failed to start roscore"
        exit 1
    fi
    echo "✅ roscore started"
else
    echo "✅ roscore already running"
    ROSCORE_PID=""
fi

# Test with 256000 (most common)
echo ""
echo "Testing rplidarNode at 256000 baud (10 seconds)..."
rosrun rplidar_ros rplidarNode \
    _serial_port:="$LIDAR_DEVICE" \
    _serial_baudrate:=256000 \
    > /tmp/rplidar_detailed.log 2>&1 &
NODE_PID=$!

sleep 10

if ps -p $NODE_PID > /dev/null 2>&1; then
    echo "✅ Node still running after 10 seconds"
    
    # Check for /scan topic
    sleep 2
    if timeout 3 rostopic echo /scan -n 1 > /dev/null 2>&1; then
        echo "✅✅✅ SUCCESS! /scan topic has data!"
        echo ""
        echo "Sample scan data:"
        timeout 2 rostopic echo /scan -n 1 | head -10
    else
        echo "⚠️  Node running but /scan not publishing"
        echo ""
        echo "Node output:"
        tail -10 /tmp/rplidar_detailed.log
    fi
else
    echo "❌ Node exited/crashed"
    echo ""
    echo "Full error log:"
    cat /tmp/rplidar_detailed.log
fi

kill $NODE_PID 2>/dev/null || true
pkill -9 rplidarNode 2>/dev/null || true
[ -n "$ROSCORE_PID" ] && kill $ROSCORE_PID 2>/dev/null || true
echo ""

# 10. Physical Check Reminder
echo "10. PHYSICAL CHECK REMINDER"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Please verify physically:"
echo "  □ LiDAR motor/disc is SPINNING (rotating)"
echo "  □ LiDAR LED is ON"
echo "  □ USB cable is securely connected"
echo "  □ LiDAR has power (check if needs external 5V)"
echo ""
read -p "Is the motor spinning? (y/n): " -n 1 -r
echo ""
MOTOR_STATUS=$REPLY
echo ""

# 11. Summary and Recommendations
echo "=========================================="
echo "DIAGNOSTIC SUMMARY"
echo "=========================================="
echo ""

if [[ ! $MOTOR_STATUS =~ ^[Yy]$ ]]; then
    echo "⚠️⚠️⚠️ MOTOR NOT SPINNING = POWER ISSUE"
    echo ""
    echo "SOLUTION: Connect external 5V power supply"
    echo "  → Most RPLidars (A2/A3/S1) need external 5V"
    echo "  → USB provides data but not enough power"
    echo "  → Connect 5V to LiDAR power connector"
    echo ""
elif [ "$HAS_DATA" == "no" ]; then
    echo "⚠️⚠️⚠️ NO DATA despite motor spinning"
    echo ""
    echo "Possible causes:"
    echo "  1. Wrong LiDAR model (not RPLidar)"
    echo "  2. Wrong baud rate (try: 9600, 38400, 57600)"
    echo "  3. LiDAR needs initialization"
    echo "  4. Hardware failure"
    echo ""
else
    echo "✅ Data received at $WORKING_BAUD baud"
    echo "   Use this baud rate in your launch file"
fi

echo ""
echo "Check detailed logs:"
echo "  cat /tmp/rplidar_detailed.log"
echo ""
