#!/bin/bash
# Copy updated launch file to Nano and provide restart instructions

NANO_IP="10.13.68.184"
NANO_USER="jetbot"
LAUNCH_FILE="robot1/nano_ros1_master/src/jetbot_nano_bringup/launch/nano_bringup_full.launch"
NANO_LAUNCH_PATH="~/MicroVIT/robot1/nano_ros1_master/src/jetbot_nano_bringup/launch/nano_bringup_full.launch"

echo "=========================================="
echo "Copy Launch File to Nano and Restart"
echo "=========================================="
echo ""

# Copy launch file
echo "1. Copying updated launch file to Nano..."
scp "$LAUNCH_FILE" ${NANO_USER}@${NANO_IP}:${NANO_LAUNCH_PATH}

if [ $? -eq 0 ]; then
    echo "   ✅ Launch file copied successfully"
else
    echo "   ❌ Failed to copy. Manual copy needed:"
    echo "   scp $LAUNCH_FILE ${NANO_USER}@${NANO_IP}:${NANO_LAUNCH_PATH}"
    exit 1
fi

echo ""
echo "2. On Nano, run these commands:"
echo ""
echo "   # Stop current launch (Ctrl+C in terminal running roslaunch)"
echo "   pkill -9 roslaunch; pkill -9 rplidarNode; pkill -9 nano_lidar_dummy"
echo ""
echo "   # Restart with real LiDAR"
echo "   cd ~/MicroVIT/robot1/nano_ros1_master"
echo "   source /opt/ros/melodic/setup.bash"
echo "   source devel/setup.bash"
echo "   roslaunch jetbot_nano_bringup nano_bringup_full.launch"
echo ""
echo "3. Verify real LiDAR is running:"
echo "   rosnode list | grep rplidarNode"
echo "   # Should show: /rplidarNode"
echo "   # Should NOT show: /nano_lidar_dummy"
echo ""
echo "   rostopic hz /scan"
echo "   # Should show ~7 Hz"
echo ""
