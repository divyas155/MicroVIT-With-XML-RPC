#!/bin/bash
# Kill ROS master and all ROS nodes - CORRECT VERSION

echo "Killing ROS master and nodes..."

# Kill each process separately (pkill only accepts one pattern at a time)
pkill -9 roscore
pkill -9 rosmaster
pkill -9 rosout
pkill -9 rplidarNode
pkill -9 roslaunch
pkill -9 rosrun

# Wait a moment
sleep 1

# Force kill rosmaster if still running
killall -9 rosmaster 2>/dev/null

# Verify
if pgrep rosmaster > /dev/null 2>&1; then
    echo "⚠️  rosmaster still running (PID: $(pgrep rosmaster))"
    echo "Force killing..."
    kill -9 $(pgrep rosmaster) 2>/dev/null
else
    echo "✅ ROS master killed successfully"
fi

echo ""
echo "You can now start roscore fresh:"
echo "  roscore"
