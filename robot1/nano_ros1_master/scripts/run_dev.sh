#!/bin/bash
# Development mode: Run Nano ROS1 master in foreground

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
NANO_DIR="$(dirname "$SCRIPT_DIR")"
WS_DIR="$NANO_DIR/../.."

# Source ROS1 environment
source /opt/ros/melodic/setup.bash

# Build workspace if needed
if [ -d "$NANO_DIR/src" ]; then
    cd "$NANO_DIR"
    if [ ! -d "devel" ]; then
        echo "Building ROS1 workspace..."
        catkin_make
    fi
    source devel/setup.bash
fi

# Start ROS master
echo "Starting ROS master..."
roscore &
ROS_MASTER_PID=$!

sleep 2

# Start bringup
echo "Starting Nano bringup..."
roslaunch jetbot_nano_bringup nano_bringup_full.launch

# Cleanup on exit
trap "kill $ROS_MASTER_PID 2>/dev/null || true" EXIT
