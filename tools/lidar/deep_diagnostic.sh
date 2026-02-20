#!/bin/bash
# Deep hardware diagnostic for LiDAR issues
# Run this on Nano to identify the root cause

echo "=========================================="
echo "Deep LiDAR Hardware Diagnostic"
echo "=========================================="
echo ""

LIDAR_DEVICE="/dev/ttyUSB0"

# 1. Check device exists
echo "1. DEVICE CHECK"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if [ -e "$LIDAR_DEVICE" ]; then
    echo "✅ Device exists: $LIDAR_DEVICE"
    ls -l "$LIDAR_DEVICE"
else
    echo "❌ Device NOT FOUND!"
    echo "Available USB devices:"
    ls -l /dev/ttyUSB* 2>/dev/null || echo "  None found"
    exit 1
fi
echo ""

# 2. USB device details
echo "2. USB DEVICE INFORMATION"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "USB device details:"
dmesg | grep -i "ttyUSB0\|cp210\|ch340\|ftdi" | tail -5
echo ""
echo "lsusb output:"
lsusb | grep -i "serial\|cp210\|ch340\|ftdi" || lsusb | tail -3
echo ""

# 3. Serial port configuration test
echo "3. SERIAL PORT CONFIGURATION TEST"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Testing if we can configure serial port..."
if stty -F "$LIDAR_DEVICE" 115200 cs8 -cstopb -parenb raw -echo 2>&1; then
    echo "✅ Serial port configuration successful"
else
    echo "❌ Cannot configure serial port"
    echo "Error: $(stty -F "$LIDAR_DEVICE" 115200 2>&1)"
fi
echo ""

# 4. Try to read raw data
echo "4. RAW DATA READ TEST"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Attempting to read raw data from device (5 seconds)..."
echo "If LiDAR is working, you should see some data bytes"
timeout 5 cat "$LIDAR_DEVICE" | head -c 100 | hexdump -C | head -5
if [ ${PIPESTATUS[0]} -eq 0 ]; then
    echo "✅ Read some data (check if it looks like LiDAR data)"
else
    echo "❌ No data received or read failed"
fi
echo ""

# 5. Check for processes using the device
echo "5. PROCESS CHECK"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
pkill -9 rplidarNode 2>/dev/null || true
sleep 1
if fuser "$LIDAR_DEVICE" 2>/dev/null; then
    echo "⚠️  Device is locked by another process"
    fuser -v "$LIDAR_DEVICE"
else
    echo "✅ Device is free (not locked)"
fi
echo ""

# 6. Test with rplidarNode and capture detailed output
echo "6. DETAILED RPLIDAR NODE TEST"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
source /opt/ros/melodic/setup.bash 2>/dev/null || {
    echo "⚠️  ROS not sourced, but continuing..."
}

# Test with 256000 (most common)
echo "Testing with 256000 baud (most common for A2/A3)..."
pkill -9 rplidarNode 2>/dev/null || true
if ! rostopic list > /dev/null 2>&1; then
    roscore > /tmp/roscore.log 2>&1 &
    sleep 3
fi

rosrun rplidar_ros rplidarNode \
    _serial_port:="$LIDAR_DEVICE" \
    _serial_baudrate:=256000 \
    > /tmp/lidar_detailed.log 2>&1 &
NODE_PID=$!

sleep 8

if ps -p $NODE_PID > /dev/null 2>&1; then
    echo "✅ Node still running after 8 seconds"
    echo "Checking for /scan data..."
    if timeout 2 rostopic echo /scan -n 1 > /dev/null 2>&1; then
        echo "✅✅✅ /scan topic has data!"
    else
        echo "⚠️  Node running but no /scan data"
    fi
else
    echo "❌ Node crashed or exited"
    echo ""
    echo "Full error log:"
    cat /tmp/lidar_detailed.log
fi

kill $NODE_PID 2>/dev/null || true
pkill -9 rplidarNode 2>/dev/null || true
echo ""

# 7. Hardware checklist
echo "7. HARDWARE CHECKLIST"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Please verify the following PHYSICALLY:"
echo ""
echo "□ LiDAR motor is SPINNING (disc rotating)"
echo "□ LiDAR LED is ON (usually green/red)"
echo "□ USB cable is securely connected"
echo "□ USB cable is not damaged"
echo "□ LiDAR is powered (check if it needs external 5V)"
echo "□ Try a different USB port on Nano"
echo "□ Try a different USB cable"
echo ""
read -p "Is the LiDAR motor spinning? (y/n): " -n 1 -r
echo ""
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo ""
    echo "⚠️⚠️⚠️ CRITICAL: Motor not spinning = Hardware issue!"
    echo ""
    echo "Possible causes:"
    echo "  1. LiDAR not powered (needs external 5V power supply)"
    echo "  2. Faulty USB cable"
    echo "  3. Faulty LiDAR unit"
    echo "  4. USB port not providing enough power"
    echo ""
    echo "Solutions to try:"
    echo "  - Connect LiDAR to external 5V power supply"
    echo "  - Try different USB port (some ports provide more power)"
    echo "  - Try different USB cable"
    echo "  - Check LiDAR manual for power requirements"
fi
echo ""

# 8. Summary and recommendations
echo "=========================================="
echo "DIAGNOSTIC SUMMARY"
echo "=========================================="
echo ""
echo "If all baud rates failed AND motor is spinning:"
echo "  → May be wrong LiDAR model (not RPLidar)"
echo "  → Check LiDAR brand/model/manual"
echo "  → May need different ROS driver"
echo ""
echo "If motor is NOT spinning:"
echo "  → Hardware/power issue (most likely)"
echo "  → Check power supply, USB cable, USB port"
echo ""
echo "Check detailed logs:"
echo "  cat /tmp/lidar_detailed.log"
echo ""
