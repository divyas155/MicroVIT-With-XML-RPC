#!/bin/bash
# Comprehensive LiDAR Connection Test Script
# Tests multiple baud rates and checks for common issues

LIDAR_DEVICE="/dev/ttyUSB0"
BAUD_RATES=(115200 230400 256000 460800 921600)

echo "=========================================="
echo "LiDAR Connection Diagnostic Test"
echo "=========================================="
echo ""

# Check 1: Device exists
echo "1. Checking if device exists..."
if [ -e "$LIDAR_DEVICE" ]; then
    echo "   ✅ Device $LIDAR_DEVICE exists"
    ls -l "$LIDAR_DEVICE"
else
    echo "   ❌ Device $LIDAR_DEVICE NOT FOUND"
    echo "   Available USB devices:"
    ls -l /dev/ttyUSB* 2>/dev/null || echo "   No /dev/ttyUSB* devices found"
    exit 1
fi
echo ""

# Check 2: Permissions
echo "2. Checking permissions..."
if [ -r "$LIDAR_DEVICE" ] && [ -w "$LIDAR_DEVICE" ]; then
    echo "   ✅ Device is readable and writable"
else
    echo "   ❌ Permission denied. User groups:"
    groups
    echo "   Device permissions:"
    ls -l "$LIDAR_DEVICE"
    echo "   Try: sudo chmod 666 $LIDAR_DEVICE"
fi
echo ""

# Check 3: Device not locked
echo "3. Checking if device is locked by another process..."
LOCKED_BY=$(lsof "$LIDAR_DEVICE" 2>/dev/null | grep -v COMMAND)
if [ -z "$LOCKED_BY" ]; then
    echo "   ✅ Device is not locked"
else
    echo "   ⚠️  Device is locked by:"
    echo "$LOCKED_BY"
    echo "   Kill process: kill -9 <PID>"
fi
echo ""

# Check 4: USB device info
echo "4. USB device information..."
USB_INFO=$(dmesg | grep -i "ttyUSB0\|cp210x" | tail -3)
if [ -n "$USB_INFO" ]; then
    echo "   USB messages:"
    echo "$USB_INFO"
else
    echo "   ⚠️  No recent USB messages found"
fi
echo ""

# Check 5: Test each baud rate
echo "5. Testing different baud rates..."
echo "   (This will take ~30 seconds)"
echo ""

for baud in "${BAUD_RATES[@]}"; do
    echo "   Testing baud rate: $baud"
    
    # Configure serial port
    stty -F "$LIDAR_DEVICE" $baud cs8 -cstopb -parenb raw -echo 2>/dev/null
    
    # Try to read something (timeout after 2 seconds)
    timeout 2 cat "$LIDAR_DEVICE" > /tmp/lidar_test_$baud.bin 2>&1
    BYTES_READ=$(wc -c < /tmp/lidar_test_$baud.bin 2>/dev/null || echo "0")
    
    if [ "$BYTES_READ" -gt 0 ]; then
        echo "      ✅ Received $BYTES_READ bytes at $baud baud"
        hexdump -C /tmp/lidar_test_$baud.bin | head -3
    else
        echo "      ❌ No data received at $baud baud"
    fi
    
    rm -f /tmp/lidar_test_$baud.bin
    sleep 0.5
done
echo ""

# Check 6: Test with RPLidar node (each baud rate)
echo "6. Testing with RPLidar ROS node..."
echo "   (Press Ctrl+C after each test if it times out)"
echo ""

for baud in "${BAUD_RATES[@]}"; do
    echo "   Testing RPLidar node at $baud baud..."
    echo "   Command: rosrun rplidar_ros rplidarNode _serial_port:=$LIDAR_DEVICE _serial_baudrate:=$baud"
    echo ""
    echo "   Starting test (will timeout after 10 seconds if no response)..."
    
    timeout 10 rosrun rplidar_ros rplidarNode _serial_port:="$LIDAR_DEVICE" _serial_baudrate:=$baud 2>&1 | head -20
    
    EXIT_CODE=$?
    if [ $EXIT_CODE -eq 0 ]; then
        echo "   ✅ SUCCESS! RPLidar responded at $baud baud"
        echo "   ⭐ RECOMMENDED BAUD RATE: $baud"
        break
    elif [ $EXIT_CODE -eq 124 ]; then
        echo "   ❌ Timeout at $baud baud (no response)"
    else
        echo "   ❌ Error at $baud baud (exit code: $EXIT_CODE)"
    fi
    echo ""
    sleep 1
done

echo ""
echo "=========================================="
echo "Test Complete"
echo "=========================================="
echo ""
echo "If all tests failed, check:"
echo "  1. LiDAR motor is spinning (physical check)"
echo "  2. USB cable is securely connected"
echo "  3. LiDAR is powered (LED should be on)"
echo "  4. Try unplugging and replugging USB"
echo "  5. Check if it's a different LiDAR brand (not RPLidar)"
echo ""
