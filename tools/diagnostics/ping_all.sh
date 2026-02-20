#!/bin/bash
# Ping all devices in the system to verify network connectivity

set -e

# Device IPs (update these for your network)
NANO_IP="${NANO_IP:-10.13.68.184}"
ORIN_IP="${ORIN_IP:-10.13.68.159}"
CONTROLLER_IP="${CONTROLLER_IP:-10.13.68.48}"
HELPER_IP="${HELPER_IP:-10.13.68.XXX}"  # Update with actual IP if using helper

echo "=========================================="
echo "Network Connectivity Check"
echo "=========================================="
echo ""

check_device() {
    local name=$1
    local ip=$2
    
    if [ "$ip" = "10.13.68.XXX" ] || [ "$ip" = "172.27.25.XXX" ] || [ -z "$ip" ]; then
        echo "⚠️  $name: IP not configured (skipping)"
        return
    fi
    
    if ping -c 1 -W 2 "$ip" > /dev/null 2>&1; then
        echo "✅ $name ($ip): Reachable"
    else
        echo "❌ $name ($ip): NOT reachable"
    fi
}

check_device "Nano" "$NANO_IP"
check_device "Orin" "$ORIN_IP"
check_device "Controller" "$CONTROLLER_IP"
check_device "Helper Robot" "$HELPER_IP"

echo ""
echo "=========================================="
