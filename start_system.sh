#!/bin/bash
# Start the full system: Controller (MQTT + AI) → Nano (ROS1 + XML-RPC) → Orin (AI service)
# Run ./setup_ssh_keys.sh once so Nano and Orin accept key-based SSH (password: jetbot).

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

CONTROLLER_IP="10.13.68.48"
NANO_IP="10.13.68.184"
ORIN_IP="10.13.68.159"

echo "=========================================="
echo "System startup"
echo "=========================================="
echo ""

# 1) Connectivity
echo "1. Checking connectivity..."
bash tools/diagnostics/ping_all.sh
echo ""

# 2) Controller
echo "2. Starting Controller (MQTT + controller-ai)..."
if ssh -o ConnectTimeout=8 -o BatchMode=yes pi@${CONTROLLER_IP} "sudo systemctl start mosquitto 2>/dev/null; sudo systemctl start controller-ai.service; sleep 2; systemctl is-active controller-ai.service" 2>/dev/null; then
    echo "   ✅ Controller is active."
else
    echo "   ⚠️  Could not start Controller via SSH (no key?). Start manually:"
    echo "      ssh pi@${CONTROLLER_IP}"
    echo "      sudo systemctl start mosquitto && sudo systemctl start controller-ai.service"
fi
echo ""

# 3) Nano (systemd if available, else instructions)
echo "3. Starting Nano (ROS1 bringup + XML-RPC)..."
if ssh -o ConnectTimeout=8 -o BatchMode=yes jetbot@${NANO_IP} "sudo systemctl start robot1-nano.service 2>/dev/null; sudo systemctl start robot1-nano-bringup.service 2>/dev/null; sleep 3; systemctl is-active robot1-nano-bringup.service 2>/dev/null || systemctl is-active robot1-nano.service 2>/dev/null || true" 2>/dev/null | grep -q active; then
    echo "   ✅ Nano bringup service is active."
elif ssh -o ConnectTimeout=8 -o BatchMode=yes jetbot@${NANO_IP} "sudo systemctl start robot1-nano-bringup.service; sleep 2; systemctl is-active robot1-nano-bringup.service" 2>/dev/null | grep -q active; then
    echo "   ✅ Nano bringup service is active."
else
    echo "   ⚠️  Could not start Nano via SSH (run ./setup_ssh_keys.sh once?) or systemd not installed. Start manually:"
    echo "      ssh jetbot@${NANO_IP}"
    echo "      source /opt/ros/melodic/setup.bash && cd ~/MicroVIT/robot1/nano_ros1_master && source devel/setup.bash"
    echo "      roslaunch jetbot_nano_bringup nano_bringup_full.launch use_lidar:=true lidar_serial_baudrate:=115200"
fi
echo ""

# 4) Orin (start after Nano so XML-RPC is available)
echo "4. Starting Orin (Robot1 AI service)..."
if ssh -o ConnectTimeout=8 -o BatchMode=yes jetbot@${ORIN_IP} "sudo systemctl start robot1-orin-ai.service; sleep 2; systemctl is-active robot1-orin-ai.service" 2>/dev/null | grep -q active; then
    echo "   ✅ Orin AI service is active."
else
    echo "   ⚠️  Could not start Orin via SSH (run ./setup_ssh_keys.sh once?) or systemd not installed. Start manually:"
    echo "      ssh jetbot@${ORIN_IP}"
    echo "      sudo systemctl start robot1-orin-ai.service"
fi
echo ""
echo "=========================================="
