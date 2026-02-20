#!/bin/bash
# Check ROS topics (ROS1 or ROS2)

set -e

ROS_VERSION="${1:-ros1}"  # ros1 or ros2

echo "=========================================="
echo "ROS $ROS_VERSION Topic Check"
echo "=========================================="
echo ""

if [ "$ROS_VERSION" = "ros1" ]; then
    source /opt/ros/melodic/setup.bash
    
    echo "ROS Master URI: $ROS_MASTER_URI"
    echo ""
    
    echo "Active Nodes:"
    rosnode list || echo "⚠️  No ROS master running"
    echo ""
    
    echo "Active Topics:"
    rostopic list || echo "⚠️  No topics available"
    echo ""
    
    echo "Key Topics Status:"
    for topic in "/cmd_vel" "/scan" "/odom"; do
        if rostopic list | grep -q "^$topic$"; then
            echo "✅ $topic: Active"
            rostopic info "$topic" | head -3
        else
            echo "❌ $topic: Not found"
        fi
        echo ""
    done
    
elif [ "$ROS_VERSION" = "ros2" ]; then
    source /opt/ros/humble/setup.bash
    
    echo "ROS Domain ID: $ROS_DOMAIN_ID"
    echo ""
    
    echo "Active Nodes:"
    ros2 node list || echo "⚠️  No ROS2 nodes running"
    echo ""
    
    echo "Active Topics:"
    ros2 topic list || echo "⚠️  No topics available"
    echo ""
    
    echo "Key Topics Status:"
    for topic in "/cmd_vel" "/scan" "/odom"; do
        if ros2 topic list | grep -q "^$topic$"; then
            echo "✅ $topic: Active"
            ros2 topic info "$topic" | head -3
        else
            echo "❌ $topic: Not found"
        fi
        echo ""
    done
else
    echo "Usage: $0 [ros1|ros2]"
    exit 1
fi

echo "=========================================="
