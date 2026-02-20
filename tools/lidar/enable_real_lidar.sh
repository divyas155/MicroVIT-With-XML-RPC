#!/bin/bash
# Helper script to enable/disable real LiDAR in launch files (use_lidar + baud)
# Usage: ./enable_real_lidar.sh [enable|disable] [baud_rate]
# Default baud: 256000 (try 115200 if RESULT_OPERATION_TIMEOUT / 80008000)

ACTION="${1:-enable}"
BAUD_RATE="${2:-256000}"

LAUNCH_FILE_1="robot1/nano_ros1_master/launch/nano_bringup_full.launch"
LAUNCH_FILE_2="robot1/nano_ros1_master/src/jetbot_nano_bringup/launch/nano_bringup_full.launch"

if [ "$ACTION" = "enable" ]; then
    echo "Enabling real LiDAR with baud rate: $BAUD_RATE"
    
    for f in "$LAUNCH_FILE_1" "$LAUNCH_FILE_2"; do
        if [ -f "$f" ]; then
            sed -i.bak "s/<arg name=\"use_lidar\" default=\"false\" \/>/<arg name=\"use_lidar\" default=\"true\" \/>/" "$f" 2>/dev/null || true
            sed -i.bak "s/<arg name=\"lidar_serial_baudrate\" default=\"[0-9]*\" \/>/<arg name=\"lidar_serial_baudrate\" default=\"$BAUD_RATE\" \/>/" "$f"
            echo "✅ Updated: $f"
        fi
    done
    
    echo ""
    echo "Real LiDAR enabled! Launch with:"
    echo "  roslaunch jetbot_nano_bringup nano_bringup_full.launch use_lidar:=true"
    echo "Or override baud: lidar_serial_baudrate:=$BAUD_RATE"
    
elif [ "$ACTION" = "disable" ]; then
    echo "Disabling real LiDAR (use dummy LiDAR)"
    
    for f in "$LAUNCH_FILE_1" "$LAUNCH_FILE_2"; do
        if [ -f "$f" ]; then
            sed -i.bak "s/<arg name=\"use_lidar\" default=\"true\" \/>/<arg name=\"use_lidar\" default=\"false\" \/>/" "$f" 2>/dev/null || true
            echo "✅ Updated: $f"
        fi
    done
    
    echo ""
    echo "Dummy LiDAR enabled. Launch with: use_lidar:=false"
    
else
    echo "Usage: $0 [enable|disable] [baud_rate]"
    echo "  $0 enable 256000   # default"
    echo "  $0 enable 115200  # if 256000 gives timeout"
    echo "  $0 disable"
    exit 1
fi
