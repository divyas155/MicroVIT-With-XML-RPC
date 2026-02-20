#!/bin/bash
echo "=========================================="
echo "Testing LiDAR with New USB Port"
echo "=========================================="

# Find which device
if [ -e "/dev/ttyUSB0" ]; then
    DEV="/dev/ttyUSB0"
elif [ -e "/dev/ttyUSB1" ]; then
    DEV="/dev/ttyUSB1"
else
    echo "❌ No USB device found!"
    ls -l /dev/ttyUSB* 2>/dev/null
    exit 1
fi

echo "Using device: $DEV"
ls -l "$DEV"
echo ""

# Check motor
echo "CRITICAL: Is the LiDAR motor spinning NOW?"
read -p "Motor spinning? (y/n): " -n 1 -r
echo ""
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo ""
    echo "🔴 Motor still NOT spinning = POWER ISSUE"
    echo "   Solution: Connect external 5V power supply"
    exit 1
fi

echo ""
echo "✅ Motor spinning! Testing data..."
echo ""

# Test
source /opt/ros/melodic/setup.bash
pkill -9 rplidarNode 2>/dev/null || true
if ! rostopic list > /dev/null 2>&1; then 
    roscore > /tmp/roscore.log 2>&1 & 
    sleep 3
fi

# Test baud rates
for baud in 115200 256000 230400; do
    echo -n "Testing $baud... "
    stty -F "$DEV" $baud cs8 -cstopb -parenb raw -echo 2>/dev/null
    DATA=$(timeout 3 cat "$DEV" 2>&1 | head -c 50)
    if [ ${#DATA} -gt 10 ]; then
        echo "✅ DATA FOUND!"
        echo "Testing rplidarNode..."
        rosrun rplidar_ros rplidarNode _serial_port:="$DEV" _serial_baudrate:=$baud > /tmp/test.log 2>&1 &
        sleep 8
        if timeout 2 rostopic echo /scan -n 1 > /dev/null 2>&1; then
            echo ""
            echo "✅✅✅ SUCCESS! Working at $baud baud!"
            echo "Device: $DEV"
            echo "Baud rate: $baud"
            pkill -9 rplidarNode 2>/dev/null || true
            exit 0
        fi
        pkill -9 rplidarNode 2>/dev/null || true
    else
        echo "❌ No data"
    fi
done

echo ""
echo "⚠️  Still having issues - check: cat /tmp/test.log"
