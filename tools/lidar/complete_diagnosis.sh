#!/bin/bash
# Complete LiDAR Diagnosis - Checks Everything
# Run this on Nano for thorough analysis

LIDAR_DEVICE="/dev/ttyUSB0"

echo "=========================================="
echo "COMPLETE LiDAR DIAGNOSIS"
echo "=========================================="
echo "Date: $(date)"
echo ""

# 1. Check SDK Version Mismatch (Important!)
echo "1. SDK VERSION CHECK"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
source /opt/ros/melodic/setup.bash 2>/dev/null || true

# Test rplidarNode to see SDK version
pkill -9 rplidarNode 2>/dev/null || true
if ! rostopic list > /dev/null 2>&1; then
    roscore > /tmp/roscore.log 2>&1 &
    sleep 3
fi

timeout 3 rosrun rplidar_ros rplidarNode \
    _serial_port:="$LIDAR_DEVICE" \
    _serial_baudrate:=256000 \
    2>&1 | grep -i "SDK\|version" | head -3 > /tmp/sdk_info.txt || true

if [ -s /tmp/sdk_info.txt ]; then
    echo "SDK Version Info:"
    cat /tmp/sdk_info.txt
    echo ""
    echo "⚠️  NOTE: SDK 1.7.0 vs 2.0.0 compatibility differences exist"
    echo "   Some LiDARs work better with specific SDK versions"
else
    echo "Could not determine SDK version"
fi
pkill -9 rplidarNode 2>/dev/null || true
echo ""

# 2. Check rplidar_ros package details
echo "2. RPLIDAR_ROS PACKAGE DETAILS"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if rospack find rplidar_ros > /dev/null 2>&1; then
    RPLIDAR_PATH=$(rospack find rplidar_ros)
    echo "Package path: $RPLIDAR_PATH"
    
    # Check for node executable
    if [ -f "$RPLIDAR_PATH/rplidarNode" ]; then
        echo "✅ Node executable found"
        ls -lh "$RPLIDAR_PATH/rplidarNode"
    elif command -v rplidarNode > /dev/null 2>&1; then
        echo "✅ Node found in PATH"
        which rplidarNode
    else
        echo "⚠️  Node executable location unclear"
    fi
    
    # Check package version if available
    if [ -f "$RPLIDAR_PATH/package.xml" ]; then
        VERSION=$(grep -i "version" "$RPLIDAR_PATH/package.xml" | head -1 | sed 's/.*<version>\(.*\)<\/version>.*/\1/' || echo "Unknown")
        echo "Package version: $VERSION"
    fi
else
    echo "❌ rplidar_ros package not found!"
fi
echo ""

# 3. USB Adapter Specific Checks
echo "3. USB ADAPTER (CP2102) SPECIFIC CHECKS"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "CP2102 is detected. Checking adapter-specific issues..."

# Check if adapter supports high baud rates
echo "Testing adapter capability..."
for baud in 115200 256000 460800; do
    if stty -F "$LIDAR_DEVICE" $baud cs8 -cstopb -parenb raw -echo 2>&1 | grep -q "error\|invalid"; then
        echo "   ❌ Cannot set $baud baud"
    else
        echo "   ✅ Adapter supports $baud baud"
    fi
done
echo ""

# 4. Serial Port State Check
echo "4. SERIAL PORT STATE ANALYSIS"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Current port settings:"
stty -F "$LIDAR_DEVICE" -a 2>&1 | head -5
echo ""

# 5. Comprehensive Data Read Test
echo "5. COMPREHENSIVE DATA READ TEST"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Testing multiple baud rates with longer timeout..."
BAUD_RATES=(9600 38400 57600 115200 128000 230400 256000 460800)
FOUND_DATA="no"

for baud in "${BAUD_RATES[@]}"; do
    echo -n "Testing $baud... "
    stty -F "$LIDAR_DEVICE" $baud cs8 -cstopb -parenb raw -echo 2>/dev/null
    
    # Longer timeout for slower baud rates
    if [ $baud -lt 115200 ]; then
        TIMEOUT=5
    else
        TIMEOUT=3
    fi
    
    RAW_DATA=$(timeout $TIMEOUT cat "$LIDAR_DEVICE" 2>&1 | head -c 200)
    BYTES=${#RAW_DATA}
    
    if [ "$BYTES" -gt 10 ]; then
        echo "✅ FOUND DATA! ($BYTES bytes)"
        echo "   First 64 bytes:"
        echo "$RAW_DATA" | head -c 64 | hexdump -C | head -3
        FOUND_DATA="yes"
        WORKING_BAUD=$baud
        break
    else
        echo "❌ No data"
    fi
done

if [ "$FOUND_DATA" == "no" ]; then
    echo ""
    echo "⚠️⚠️⚠️ NO DATA at ANY baud rate (including low rates)"
    echo "   This STRONGLY indicates:"
    echo "   1. Motor not spinning (power issue) - 95% likely"
    echo "   2. LiDAR completely dead"
    echo "   3. Wrong device (not LiDAR)"
fi
echo ""

# 6. Process and Lock Check (Detailed)
echo "6. PROCESS AND LOCK CHECK (DETAILED)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
pkill -9 rplidarNode 2>/dev/null || true
sleep 2

# Check all possible ways device could be locked
echo "Checking for processes using device..."

# Method 1: fuser
if command -v fuser > /dev/null 2>&1; then
    FUSER_OUT=$(fuser "$LIDAR_DEVICE" 2>&1)
    if echo "$FUSER_OUT" | grep -q "[0-9]"; then
        echo "⚠️  Device locked by process(es):"
        fuser -v "$LIDAR_DEVICE" 2>/dev/null
    else
        echo "✅ Device free (fuser check)"
    fi
fi

# Method 2: lsof (if available)
if command -v lsof > /dev/null 2>&1; then
    LSOF_OUT=$(sudo lsof "$LIDAR_DEVICE" 2>&1 | grep -v COMMAND)
    if [ -n "$LSOF_OUT" ]; then
        echo "⚠️  Device locked (lsof check):"
        echo "$LSOF_OUT"
    else
        echo "✅ Device free (lsof check)"
    fi
fi

# Method 3: Check /proc
PROC_LOCKS=$(ls -l /proc/*/fd 2>/dev/null | grep "$LIDAR_DEVICE" | wc -l)
if [ "$PROC_LOCKS" -gt 0 ]; then
    echo "⚠️  Device appears in use ($PROC_LOCKS file descriptors)"
else
    echo "✅ Device free (proc check)"
fi
echo ""

# 7. Hardware Power Check
echo "7. HARDWARE POWER CHECK"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "CRITICAL: Physical verification required"
echo ""
echo "Please check:"
echo "  □ LiDAR motor/disc is SPINNING (rotating)"
echo "  □ LiDAR LED is ON"
echo "  □ USB cable is securely connected"
echo "  □ LiDAR has external power (if required)"
echo ""
read -p "Is the motor spinning? (y/n): " -n 1 -r
MOTOR_SPINNING=$REPLY
echo ""
echo ""

# 8. Alternative Driver Check
echo "8. ALTERNATIVE DRIVER CHECK"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Checking if alternative drivers are available..."
if rospack list | grep -i "lidar\|rplidar" | grep -v rplidar_ros; then
    echo "Alternative LiDAR drivers found:"
    rospack list | grep -i "lidar\|rplidar" | grep -v rplidar_ros
else
    echo "No alternative drivers found"
fi
echo ""

# 9. Final Summary
echo "=========================================="
echo "DIAGNOSIS SUMMARY"
echo "=========================================="
echo ""

if [[ ! $MOTOR_SPINNING =~ ^[Yy]$ ]]; then
    echo "🔴 PRIMARY ISSUE: Motor NOT spinning"
    echo ""
    echo "ROOT CAUSE: Power issue (99% certain)"
    echo ""
    echo "SOLUTION:"
    echo "  1. Connect external 5V power supply to LiDAR"
    echo "  2. Most RPLidars (A2/A3/S1) need external 5V"
    echo "  3. USB provides data but not enough power for motor"
    echo "  4. Check LiDAR manual for power requirements"
    echo ""
    echo "After connecting power:"
    echo "  - Motor should spin immediately"
    echo "  - LED should turn on"
    echo "  - Then retest with rplidarNode"
    echo ""
elif [ "$FOUND_DATA" == "no" ]; then
    echo "🟡 ISSUE: Motor spinning but NO DATA"
    echo ""
    echo "Possible causes:"
    echo "  1. Wrong LiDAR model (not RPLidar)"
    echo "  2. Wrong driver (needs different ROS package)"
    echo "  3. Hardware failure"
    echo "  4. Baud rate not tested (try: 9600, 38400, 57600)"
    echo ""
    echo "Next steps:"
    echo "  - Verify LiDAR model/brand"
    echo "  - Check if it's actually an RPLidar"
    echo "  - Try different ROS driver if available"
else
    echo "🟢 DATA FOUND at $WORKING_BAUD baud"
    echo ""
    echo "Use this baud rate in your launch file:"
    echo "  lidar_serial_baudrate:=$WORKING_BAUD"
fi

echo ""
echo "Detailed logs available:"
echo "  /tmp/sdk_info.txt"
echo "  /tmp/rplidar_detailed.log (if created)"
echo ""
