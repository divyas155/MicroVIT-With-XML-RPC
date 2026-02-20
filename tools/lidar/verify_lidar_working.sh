#!/bin/bash
# Verify LiDAR is working correctly

echo "=========================================="
echo "LiDAR Verification Check"
echo "=========================================="
echo ""

source /opt/ros/melodic/setup.bash 2>/dev/null || {
    echo "❌ ROS not sourced"
    exit 1
}

# 1. Check /scan topic exists
echo "1. Checking /scan topic..."
if rostopic list | grep -q "^/scan$"; then
    echo "   ✅ /scan topic exists"
else
    echo "   ❌ /scan topic not found"
    exit 1
fi

# 2. Check publishing rate
echo ""
echo "2. Checking publishing rate..."
RATE=$(timeout 3 rostopic hz /scan 2>&1 | grep "average rate" | tail -1 | awk '{print $3}')
if [ -n "$RATE" ]; then
    echo "   ✅ Publishing at ~$RATE Hz"
else
    echo "   ⚠️  Could not determine rate"
fi

# 3. Check scan data
echo ""
echo "3. Checking scan data..."
SCAN_DATA=$(timeout 2 rostopic echo /scan -n 1 2>/dev/null)
if [ -n "$SCAN_DATA" ]; then
    echo "   ✅ Scan data available"
    
    # Extract key fields
    RANGES_COUNT=$(echo "$SCAN_DATA" | grep -A 1000 "ranges:" | grep -c "^-" || echo "0")
    ANGLE_MIN=$(echo "$SCAN_DATA" | grep "angle_min:" | awk '{print $2}')
    ANGLE_MAX=$(echo "$SCAN_DATA" | grep "angle_max:" | awk '{print $2}')
    RANGE_MIN=$(echo "$SCAN_DATA" | grep "range_min:" | awk '{print $2}')
    RANGE_MAX=$(echo "$SCAN_DATA" | grep "range_max:" | awk '{print $2}')
    
    echo ""
    echo "   Scan parameters:"
    echo "   - Number of ranges: $RANGES_COUNT"
    echo "   - Angle range: $ANGLE_MIN to $ANGLE_MAX rad"
    echo "   - Distance range: $RANGE_MIN to $RANGE_MAX m"
else
    echo "   ❌ No scan data"
    exit 1
fi

# 4. Check rplidarNode status
echo ""
echo "4. Checking rplidarNode status..."
if pgrep rplidarNode > /dev/null; then
    echo "   ✅ rplidarNode is running (PID: $(pgrep rplidarNode))"
else
    echo "   ⚠️  rplidarNode not running (but /scan is publishing)"
fi

echo ""
echo "=========================================="
echo "✅ LiDAR is working correctly!"
echo "=========================================="
echo ""
echo "Configuration:"
echo "  Device: /dev/ttyUSB0"
echo "  Baud rate: 115200"
echo "  Publishing rate: ~$RATE Hz"
echo ""
echo "Next steps:"
echo "  1. Launch full system: roslaunch jetbot_nano_bringup nano_bringup_full.launch"
echo "  2. Verify LiDAR data via XML-RPC: get_lidar_data()"
echo ""
