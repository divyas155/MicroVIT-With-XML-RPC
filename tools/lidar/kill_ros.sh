#!/bin/bash
# Kill ROS master and all ROS nodes

echo "Killing ROS master and nodes..."

# Kill all ROS processes
pkill -9 roscore
pkill -9 rosmaster
pkill -9 rosout
pkill -9 rplidarNode
pkill -9 roslaunch
pkill -9 rosrun

# Wait a moment
sleep 1

# Force kill rosmaster if still running
if pgrep rosmaster > /dev/null 2>&1; then
    echo "Force killing rosmaster..."
    killall -9 rosmaster 2>/dev/null
fi

# Verify
if pgrep rosmaster > /dev/null 2>&1; then
    echo "⚠️  rosmaster still running"
    echo "Run manually: killall -9 rosmaster"
else
    echo "✅ ROS master killed successfully"
fi

echo ""
echo "You can now start roscore fresh:"
echo "  roscore"
