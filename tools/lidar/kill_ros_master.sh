#!/bin/bash
# Kill ROS master and all ROS nodes

echo "Killing ROS master and nodes..."

# Kill roscore/rosmaster
pkill -9 roscore
pkill -9 rosmaster
pkill -9 rosout

# Kill all ROS nodes
pkill -9 rplidarNode
pkill -9 roslaunch
pkill -9 rosrun

# Wait a moment
sleep 1

# Verify
if pgrep -x rosmaster > /dev/null; then
    echo "⚠️  rosmaster still running, forcing kill..."
    killall -9 rosmaster
else
    echo "✅ ROS master killed"
fi

echo "Done. You can now start roscore fresh."
