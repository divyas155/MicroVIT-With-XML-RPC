#!/bin/bash
# Check LiDAR test logs for errors
# Run this on Nano after testing

echo "Checking LiDAR test logs..."
echo ""

if [ -f /tmp/lidar_detailed.log ]; then
    echo "=== Detailed Test Log ==="
    cat /tmp/lidar_detailed.log
    echo ""
fi

for log in /tmp/lidar_*.log; do
    if [ -f "$log" ]; then
        echo "=== $(basename $log) ==="
        cat "$log"
        echo ""
    fi
done

echo "=== System Messages (dmesg) ==="
dmesg | grep -i "ttyUSB0\|usb\|serial" | tail -10
