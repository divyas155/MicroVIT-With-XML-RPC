#!/bin/bash
# Detailed real LiDAR node checkup - run on Nano (jetbot@10.13.68.184)
# Usage: bash detailed_lidar_checkup.sh
# Report saved to: /tmp/lidar_checkup_report.txt

set -e
REPORT="/tmp/lidar_checkup_report.txt"
LIDAR_DEVICE="${LIDAR_DEVICE:-/dev/ttyUSB0}"
BAUD_RATES=(256000 115200 230400 128000 460800 9600)

exec 3>&1
exec 1> >(tee -a "$REPORT")
exec 2>&1

echo "=============================================="
echo "  DETAILED REAL LIDAR NODE CHECKUP"
echo "  $(date -Iseconds)"
echo "=============================================="
echo ""

# ---------------------------------------------------------------------------
# 1. System and user
# ---------------------------------------------------------------------------
echo "1. SYSTEM & USER"
echo "──────────────────────────────────────────────"
echo "Hostname: $(hostname)"
echo "User: $(whoami) (groups: $(groups))"
echo "Kernel: $(uname -r)"
echo ""

# ---------------------------------------------------------------------------
# 2. ROS and rplidar_ros
# ---------------------------------------------------------------------------
echo "2. ROS & RPLIDAR_ROS PACKAGE"
echo "──────────────────────────────────────────────"
if [ ! -f /opt/ros/melodic/setup.bash ]; then
    echo "❌ ROS Melodic not found."
else
    source /opt/ros/melodic/setup.bash
    echo "ROS_DISTRO=$ROS_DISTRO"
    if rospack find rplidar_ros >/dev/null 2>&1; then
        RPLIDAR_PATH=$(rospack find rplidar_ros)
        echo "rplidar_ros: $RPLIDAR_PATH"
        if [ -f "$RPLIDAR_PATH/rplidarNode" ]; then
            echo "rplidarNode: present"
        else
            echo "rplidarNode: not found in package (may be in PATH)"
        fi
    else
        echo "❌ rplidar_ros package not found."
    fi
fi
echo ""

# ---------------------------------------------------------------------------
# 3. Serial device
# ---------------------------------------------------------------------------
echo "3. SERIAL DEVICE ($LIDAR_DEVICE)"
echo "──────────────────────────────────────────────"
if [ ! -e "$LIDAR_DEVICE" ]; then
    echo "❌ Device does not exist."
    echo "   Available: $(ls /dev/ttyUSB* 2>/dev/null || echo 'none')"
else
    ls -l "$LIDAR_DEVICE"
    if [ -r "$LIDAR_DEVICE" ] && [ -w "$LIDAR_DEVICE" ]; then
        echo "✅ Read/write OK"
    else
        echo "❌ Permission denied. Run: sudo chmod 666 $LIDAR_DEVICE"
    fi
fi
echo ""

# ---------------------------------------------------------------------------
# 4. USB hardware (dmesg, lsusb)
# ---------------------------------------------------------------------------
echo "4. USB HARDWARE"
echo "──────────────────────────────────────────────"
echo "dmesg (last ttyUSB/serial/CP210/CH340/FTDI):"
dmesg 2>/dev/null | grep -iE "ttyUSB|serial|cp210|ch340|ftdi|pl2303" | tail -15 || echo "  (none or no permission)"
echo ""
echo "lsusb:"
lsusb 2>/dev/null | head -20
echo ""

# ---------------------------------------------------------------------------
# 5. Process lock
# ---------------------------------------------------------------------------
echo "5. PORT LOCK"
echo "──────────────────────────────────────────────"
pkill -9 rplidarNode 2>/dev/null || true
sleep 1
if command -v fuser >/dev/null 2>&1; then
    if fuser "$LIDAR_DEVICE" 2>/dev/null; then
        echo "⚠️  Device in use:"
        fuser -v "$LIDAR_DEVICE" 2>/dev/null || true
    else
        echo "✅ No process using $LIDAR_DEVICE"
    fi
else
    echo "  (fuser not available)"
fi
echo ""

# ---------------------------------------------------------------------------
# 6. Serial config (stty)
# ---------------------------------------------------------------------------
echo "6. SERIAL CONFIG (stty)"
echo "──────────────────────────────────────────────"
for baud in 256000 115200; do
    if stty -F "$LIDAR_DEVICE" $baud cs8 -cstopb -parenb raw -echo 2>/dev/null; then
        echo "  $baud: OK"
    else
        echo "  $baud: failed"
    fi
done
echo ""

# ---------------------------------------------------------------------------
# 7. Raw read (multiple bauds)
# ---------------------------------------------------------------------------
echo "7. RAW READ (3s per baud)"
echo "──────────────────────────────────────────────"
RAW_ANY="no"
RAW_BAUD=""
for baud in 115200 256000 230400 128000 460800; do
    stty -F "$LIDAR_DEVICE" $baud cs8 -cstopb -parenb raw -echo 2>/dev/null || true
    BYTES=$(timeout 3 cat "$LIDAR_DEVICE" 2>/dev/null | head -c 500 | wc -c)
    if [ "${BYTES:-0}" -gt 0 ]; then
        echo "  $baud: ✅ ${BYTES} bytes"
        RAW_ANY="yes"
        RAW_BAUD="$baud"
    else
        echo "  $baud: ❌ 0 bytes"
    fi
    sleep 0.5
done
if [ "$RAW_ANY" = "no" ]; then
    echo "  → No raw data at any baud. Likely: motor not spinning (power) or wrong port."
fi
echo ""

# ---------------------------------------------------------------------------
# 8. roscore
# ---------------------------------------------------------------------------
echo "8. ROS MASTER"
echo "──────────────────────────────────────────────"
if ! rostopic list >/dev/null 2>&1; then
    echo "Starting roscore in background..."
    roscore >/tmp/roscore_checkup.log 2>&1 &
    ROSCORE_PID=$!
    for i in 1 2 3 4 5; do
        sleep 1
        if rostopic list >/dev/null 2>&1; then break; fi
    done
    if ! rostopic list >/dev/null 2>&1; then
        echo "❌ roscore failed to start"
        ROSCORE_PID=""
    else
        echo "✅ roscore running"
    fi
else
    echo "✅ roscore already running"
    ROSCORE_PID=""
fi
echo ""

# ---------------------------------------------------------------------------
# 9. rplidarNode test per baud (full log + /scan check)
# ---------------------------------------------------------------------------
echo "9. RPLIDARNODE TEST (per baud, ~10s each)"
echo "──────────────────────────────────────────────"
NODE_SUCCESS=""
for baud in "${BAUD_RATES[@]}"; do
    pkill -9 rplidarNode 2>/dev/null || true
    sleep 1
    echo ""
    echo "  --- $baud baud ---"
    timeout 12 rosrun rplidar_ros rplidarNode \
        _serial_port:="$LIDAR_DEVICE" \
        _serial_baudrate:=$baud \
        > /tmp/rplidar_node_${baud}.log 2>&1 &
    NODE_PID=$!
    sleep 8
    if ! kill -0 $NODE_PID 2>/dev/null; then
        echo "    Node exited. Last lines:"
        tail -5 /tmp/rplidar_node_${baud}.log 2>/dev/null | sed 's/^/      /'
    else
        SCAN_OK=""
        if timeout 3 rostopic echo /scan -n 1 >/dev/null 2>&1; then
            SCAN_OK="yes"
            echo "    ✅ Node running, /scan has data"
            NODE_SUCCESS="$baud"
            kill $NODE_PID 2>/dev/null || true
            break
        else
            echo "    ⚠️  Node running but no /scan data"
            tail -3 /tmp/rplidar_node_${baud}.log 2>/dev/null | sed 's/^/      /'
        fi
        kill $NODE_PID 2>/dev/null || true
    fi
done
pkill -9 rplidarNode 2>/dev/null || true
echo ""

# ---------------------------------------------------------------------------
# 10. Full node log sample (last failure if any)
# ---------------------------------------------------------------------------
echo "10. SAMPLE NODE LOG (last failed baud)"
echo "──────────────────────────────────────────────"
for baud in 256000 115200 230400; do
    if [ -f /tmp/rplidar_node_${baud}.log ]; then
        echo "  From /tmp/rplidar_node_${baud}.log:"
        cat /tmp/rplidar_node_${baud}.log | sed 's/^/    /'
        break
    fi
done
echo ""

# ---------------------------------------------------------------------------
# 11. Summary and recommendation
# ---------------------------------------------------------------------------
echo "=============================================="
echo "  SUMMARY & RECOMMENDATION"
echo "=============================================="
echo ""

if [ -n "$NODE_SUCCESS" ]; then
    echo "✅ WORKING BAUD: $NODE_SUCCESS"
    echo "   Use in bringup:"
    echo "   roslaunch jetbot_nano_bringup nano_bringup_full.launch use_lidar:=true lidar_serial_baudrate:=$NODE_SUCCESS"
elif [ "$RAW_ANY" = "yes" ]; then
    echo "⚠️  Raw data at ${RAW_BAUD} baud but rplidarNode did not produce /scan."
    echo "   Try: different USB port, or run bringup with lidar_serial_baudrate:=${RAW_BAUD}"
else
    echo "❌ No raw data from LiDAR at any baud."
    echo ""
    echo "   Most likely: MOTOR NOT SPINNING (power)."
    echo "   → Use external 5V (2A) on LiDAR power connector; keep USB for data."
    echo "   → See docs/LIDAR_RESOLVE.md and docs/lidar_power_fix.md"
    echo ""
    echo "   Other: wrong /dev (try ttyUSB1), or faulty unit."
fi

echo ""
echo "Report saved to: $REPORT"
echo "Node logs: /tmp/rplidar_node_<baud>.log"
echo ""

# Don't leave roscore running if we started it (optional)
[ -n "$ROSCORE_PID" ] && kill $ROSCORE_PID 2>/dev/null || true
