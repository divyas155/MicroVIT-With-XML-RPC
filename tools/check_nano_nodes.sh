#!/bin/bash
# Check which Nano nodes are running: Camera, LiDAR, Odom (real vs dummy)
# Run from Mac: ./tools/check_nano_nodes.sh
# Or on Nano: bash check_nano_nodes.sh

set -e
NANO_IP="${NANO_IP:-10.13.68.184}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

echo "=========================================="
echo "Nano Node Diagnostic (Camera, LiDAR, Odom)"
echo "=========================================="
echo ""
echo "Target: jetbot@${NANO_IP}"
echo ""

# Run checks on Nano via SSH
ssh -o ConnectTimeout=5 jetbot@${NANO_IP} bash -s << 'REMOTE'
# Source ROS (required for rostopic, rosnode)
source /opt/ros/melodic/setup.bash 2>/dev/null
[ -f /home/jetbot/MicroVIT/robot1/nano_ros1_master/devel/setup.bash ] && source /home/jetbot/MicroVIT/robot1/nano_ros1_master/devel/setup.bash 2>/dev/null

echo "1. ROS Master & Nodes"
echo "   ------------------"
if ! rostopic list > /dev/null 2>&1; then
    echo "   ❌ ROS master not running. Start Nano bringup first."
    exit 1
fi
echo "   ✅ ROS master running"
echo ""
echo "   Running nodes:"
rosnode list 2>/dev/null | while read n; do echo "      - $n"; done
echo ""

echo "2. LiDAR (/scan)"
echo "   ------------------"
SCAN_INFO=$(rostopic info /scan 2>/dev/null || true)
if echo "$SCAN_INFO" | grep -q "rplidarNode"; then
    SCAN_PUB="rplidarNode"
elif echo "$SCAN_INFO" | grep -q "nano_lidar_dummy"; then
    SCAN_PUB="nano_lidar_dummy"
else
    SCAN_PUB=""
fi
if [ -z "$SCAN_PUB" ]; then
    echo "   ❌ No publisher for /scan"
else
    if [ "$SCAN_PUB" = "rplidarNode" ]; then
        echo "   ✅ REAL LiDAR (rplidarNode)"
        RATE=$(timeout 2 rostopic hz /scan 2>&1 | grep "average rate" | awk '{print $3}')
        [ -n "$RATE" ] && echo "      Publishing at ~${RATE} Hz"
    elif [ "$SCAN_PUB" = "nano_lidar_dummy" ]; then
        echo "   ⚠️  DUMMY LiDAR (nano_lidar_dummy)"
    else
        echo "   ? LiDAR source: $SCAN_PUB"
    fi
fi
echo ""

echo "3. Odometry (/odom)"
echo "   ------------------"
ODOM_INFO=$(rostopic info /odom 2>/dev/null || true)
if echo "$ODOM_INFO" | grep -q "jetbot_motor_driver"; then
    ODOM_PUB="jetbot_motor_driver"
elif echo "$ODOM_INFO" | grep -q "nano_odom_dummy"; then
    ODOM_PUB="nano_odom_dummy"
else
    ODOM_PUB=""
fi
if [ -z "$ODOM_PUB" ]; then
    echo "   ❌ No publisher for /odom (motor driver may not send odom packets)"
else
    if [ "$ODOM_PUB" = "jetbot_motor_driver" ]; then
        echo "   ✅ REAL Odom (jetbot_motor_driver)"
        RATE=$(timeout 2 rostopic hz /odom 2>&1 | grep "average rate" | awk '{print $3}')
        [ -n "$RATE" ] && echo "      Publishing at ~${RATE} Hz"
    elif [ "$ODOM_PUB" = "nano_odom_dummy" ]; then
        echo "   ⚠️  DUMMY Odom (nano_odom_dummy)"
    else
        echo "   ? Odom source: $ODOM_PUB"
    fi
fi
echo ""

echo "4. Camera (via XML-RPC get_robot_status)"
echo "   ------------------"
# Call get_robot_status via Python on Nano
python3 -c "
import xmlrpc.client
import sys
try:
    p = xmlrpc.client.ServerProxy('http://127.0.0.1:8000', allow_none=True)
    st = p.get_robot_status()
    if st.get('success'):
        cam = st.get('camera_available', False)
        msg = st.get('status_message', '')
        if cam:
            print('   ✅ Camera available')
        else:
            print('   ❌ Camera NOT available:', msg)
    else:
        print('   ❌ get_robot_status failed:', st.get('status_message', 'unknown'))
except Exception as e:
    print('   ❌ XML-RPC error:', e)
" 2>/dev/null || echo "   ❌ Could not query XML-RPC (port 8000)"
echo ""

echo "5. XML-RPC Server"
echo "   ------------------"
if rosnode list 2>/dev/null | grep -q "nano_xmlrpc"; then
    echo "   ✅ nano_xmlrpc_server running"
else
    echo "   ❌ nano_xmlrpc_server not found"
fi
echo ""

echo "6. Motor Driver"
echo "   ------------------"
if rosnode list 2>/dev/null | grep -q "jetbot_motor_driver"; then
    echo "   ✅ jetbot_motor_driver running"
else
    echo "   ❌ jetbot_motor_driver not found"
fi
echo ""

echo "=========================================="
echo "Summary"
echo "=========================================="
REMOTE
