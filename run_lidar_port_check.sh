#!/bin/bash
# Run LiDAR port check on the Nano and show report. Execute from your Mac.
# You will be prompted for jetbot password (twice: scp then ssh).

set -e
NANO="jetbot@10.13.68.184"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "Copying check_lidar_port.sh to Nano..."
scp -o ConnectTimeout=15 tools/lidar/check_lidar_port.sh "$NANO":~/

echo ""
echo "Running port check on Nano and fetching report..."
echo "=============================================="
ssh -o ConnectTimeout=15 "$NANO" 'bash ~/check_lidar_port.sh; echo ""; echo "=== REPORT ==="; cat /tmp/lidar_port_report.txt'
echo "=============================================="
echo "Done."
