#!/bin/bash
# Comprehensive test when motor is spinning
# Tests all baud rates and checks for any data

echo "=========================================="
echo "Comprehensive LiDAR Test (Motor Spinning)"
echo "=========================================="
echo ""

DEV="/dev/ttyUSB0"
source /opt/ros/melodic/setup.bash

# Kill any existing nodes
pkill -9 rplidarNode 2>/dev/null || true
if ! rostopic list > /dev/null 2>&1; then 
    roscore > /tmp/roscore.log 2>&1 & 
    sleep 3
fi

echo "1. TESTING RAW DATA READ (All Baud Rates)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
BAUD_RATES=(9600 38400 57600 115200 128000 230400 256000 460800)
FOUND_DATA="no"

for baud in "${BAUD_RATES[@]}"; do
    echo -n "Testing $baud baud (raw read)... "
    stty -F "$DEV" $baud cs8 -cstopb -parenb raw -echo 2>/dev/null
    
    # Longer timeout for slower baud rates
    TIMEOUT=$([ $baud -lt 115200 ] && echo 5 || echo 3)
    RAW_DATA=$(timeout $TIMEOUT cat "$DEV" 2>&1 | head -c 200)
    BYTES=${#RAW_DATA}
    
    if [ "$BYTES" -gt 10 ]; then
        echo "✅ DATA FOUND! ($BYTES bytes)"
        echo "   First 64 bytes (hex):"
        echo "$RAW_DATA" | head -c 64 | hexdump -C | head -3
        FOUND_DATA="yes"
        WORKING_BAUD=$baud
        break
    else
        echo "❌ No data"
    fi
done

echo ""

if [ "$FOUND_DATA" == "no" ]; then
    echo "⚠️  NO DATA at any baud rate despite motor spinning"
    echo "   This suggests:"
    echo "   1. Wrong LiDAR model (not RPLidar)"
    echo "   2. LiDAR needs initialization sequence"
    echo "   3. Communication protocol issue"
    echo ""
fi

echo "2. TESTING RPLIDAR NODE (All Baud Rates)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Test rplidarNode with all baud rates
for baud in "${BAUD_RATES[@]}"; do
    echo -n "Testing rplidarNode at $baud... "
    
    pkill -9 rplidarNode 2>/dev/null || true
    sleep 1
    
    # Run node with timeout
    timeout 10 rosrun rplidar_ros rplidarNode \
        _serial_port:="$DEV" \
        _serial_baudrate:=$baud \
        > /tmp/rplidar_${baud}.log 2>&1 &
    
    NODE_PID=$!
    sleep 8
    
    # Check if node is still running
    if ps -p $NODE_PID > /dev/null 2>&1; then
        echo "✅ Node running"
        
        # Check for /scan data
        sleep 2
        if timeout 3 rostopic echo /scan -n 1 > /dev/null 2>&1; then
            echo "   ✅✅✅ SUCCESS! /scan topic has data!"
            echo ""
            echo "   Working configuration:"
            echo "   Device: $DEV"
            echo "   Baud rate: $baud"
            echo ""
            echo "   Sample scan data:"
            timeout 2 rostopic echo /scan -n 1 | head -10
            echo ""
            kill $NODE_PID 2>/dev/null || true
            pkill -9 rplidarNode 2>/dev/null || true
            exit 0
        else
            echo "   ⚠️  Node running but no /scan data"
        fi
        kill $NODE_PID 2>/dev/null || true
    else
        echo "❌ Node crashed/timeout"
        # Show error
        if [ -f /tmp/rplidar_${baud}.log ]; then
            ERROR=$(grep -i "error\|timeout\|fail" /tmp/rplidar_${baud}.log | head -1)
            if [ -n "$ERROR" ]; then
                echo "   Error: $ERROR"
            fi
        fi
    fi
    
    pkill -9 rplidarNode 2>/dev/null || true
    sleep 1
done

echo ""
echo "=========================================="
echo "SUMMARY"
echo "=========================================="

if [ "$FOUND_DATA" == "yes" ]; then
    echo "✅ Raw data found at $WORKING_BAUD baud"
    echo "   But rplidarNode still failing"
    echo "   Possible: SDK version issue or initialization needed"
elif [ -f /tmp/rplidar_256000.log ]; then
    echo "Checking error details..."
    echo ""
    echo "Error log (256000 baud):"
    cat /tmp/rplidar_256000.log
else
    echo "❌ No data found and rplidarNode failing at all baud rates"
    echo ""
    echo "Possible issues:"
    echo "  1. Wrong LiDAR model (not RPLidar)"
    echo "  2. LiDAR needs specific initialization"
    echo "  3. SDK version incompatibility"
    echo "  4. Hardware communication issue"
fi

echo ""
echo "Check logs: ls -l /tmp/rplidar_*.log"
