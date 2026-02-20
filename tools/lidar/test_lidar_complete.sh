#!/bin/bash
# Complete LiDAR Test with ROS Environment

LIDAR_DEVICE="/dev/ttyUSB0"
BAUD_RATES=(115200 230400 256000)

# Source ROS
source /opt/ros/melodic/setup.bash 2>/dev/null || true

echo "=========================================="
echo "Complete LiDAR Diagnostic Test"
echo "=========================================="
echo ""

# Check 1: Device and permissions
echo "1. Device Check:"
if [ -e "$LIDAR_DEVICE" ]; then
    echo "   ✅ $LIDAR_DEVICE exists"
    ls -l "$LIDAR_DEVICE"
    
    # Check if readable/writable
    if [ ! -r "$LIDAR_DEVICE" ] || [ ! -w "$LIDAR_DEVICE" ]; then
        echo "   ⚠️  Permission issue. Fixing..."
        sudo chmod 666 "$LIDAR_DEVICE" 2>/dev/null || echo "   Need sudo to fix permissions"
    fi
else
    echo "   ❌ Device not found!"
    exit 1
fi
echo ""

# Check 2: Process lock
echo "2. Process Lock Check:"
LOCKED=$(lsof "$LIDAR_DEVICE" 2>/dev/null | grep -v COMMAND)
if [ -z "$LOCKED" ]; then
    echo "   ✅ Device not locked"
else
    echo "   ⚠️  Locked by:"
    echo "$LOCKED"
fi
echo ""

# Check 3: USB connection stability
echo "3. USB Connection Check:"
dmesg | grep -i "ttyUSB0\|cp210x" | tail -3
echo ""

# Check 4: Test each baud rate with RPLidar
echo "4. Testing RPLidar at different baud rates:"
echo "   (Each test runs for 8 seconds)"
echo ""

for baud in "${BAUD_RATES[@]}"; do
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    echo "Testing: $baud baud"
    echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
    
    # Kill any existing rplidarNode
    pkill -9 rplidarNode 2>/dev/null || true
    sleep 1
    
    # Test with timeout
    timeout 8 rosrun rplidar_ros rplidarNode \
        _serial_port:="$LIDAR_DEVICE" \
        _serial_baudrate:=$baud \
        2>&1 | tee /tmp/lidar_test_$baud.log &
    
    TEST_PID=$!
    sleep 8
    
    # Check if process is still running (means it didn't crash)
    if ps -p $TEST_PID > /dev/null 2>&1; then
        echo "   ✅ Node still running - checking for scan data..."
        sleep 2
        
        # Check if /scan topic has data
        if timeout 2 rostopic echo /scan -n 1 > /dev/null 2>&1; then
            echo "   ✅✅ SUCCESS! LiDAR working at $baud baud!"
            echo "   ⭐ RECOMMENDED BAUD RATE: $baud"
            kill $TEST_PID 2>/dev/null || true
            break
        else
            echo "   ⚠️  Node running but no /scan data yet"
        fi
        kill $TEST_PID 2>/dev/null || true
    else
        echo "   ❌ Node crashed or timed out"
        if [ -f /tmp/lidar_test_$baud.log ]; then
            echo "   Error log:"
            grep -i "error\|timeout\|fail" /tmp/lidar_test_$baud.log | head -3 || echo "   (No specific errors found)"
        fi
    fi
    
    pkill -9 rplidarNode 2>/dev/null || true
    sleep 1
    echo ""
done

# Cleanup
pkill -9 rplidarNode 2>/dev/null || true
rm -f /tmp/lidar_test_*.log

echo "=========================================="
echo "Diagnostic Summary"
echo "=========================================="
echo ""
echo "If all tests failed, possible issues:"
echo "  1. LiDAR motor not spinning (check physically)"
echo "  2. LiDAR not powered (check LED)"
echo "  3. Wrong LiDAR model (not RPLidar)"
echo "  4. USB cable loose or damaged"
echo "  5. LiDAR needs initialization command"
echo ""
echo "Try:"
echo "  - Unplug and replug USB cable"
echo "  - Power cycle the LiDAR"
echo "  - Check LiDAR model/manual for correct baud rate"
echo ""
