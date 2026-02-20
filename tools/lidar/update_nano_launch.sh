#!/bin/bash
# Copy updated launch file to Nano

NANO_IP="10.13.68.184"
NANO_USER="jetbot"
LAUNCH_FILE="robot1/nano_ros1_master/src/jetbot_nano_bringup/launch/nano_bringup_full.launch"
NANO_PATH="~/MicroVIT/robot1/nano_ros1_master/src/jetbot_nano_bringup/launch/"

echo "Copying updated launch file to Nano..."
echo ""

# Copy launch file
scp "$LAUNCH_FILE" ${NANO_USER}@${NANO_IP}:${NANO_PATH}

if [ $? -eq 0 ]; then
    echo "✅ Launch file copied successfully!"
    echo ""
    echo "On Nano, restart the launch:"
    echo "  1. Stop current launch (Ctrl+C)"
    echo "  2. Run: roslaunch jetbot_nano_bringup nano_bringup_full.launch"
    echo ""
    echo "You should now see rplidarNode instead of nano_lidar_dummy"
else
    echo "❌ Failed to copy. Try manually:"
    echo "  scp $LAUNCH_FILE ${NANO_USER}@${NANO_IP}:${NANO_PATH}"
fi
