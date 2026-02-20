#!/bin/bash
# Instructions to copy and run diagnostic on Nano
# 
# OPTION 1: Copy script to Nano (from your development machine)
# scp tools/lidar/diagnose_lidar_nano.sh jetbot@10.13.68.184:~/
# ssh jetbot@10.13.68.184
# chmod +x diagnose_lidar_nano.sh
# ./diagnose_lidar_nano.sh
#
# OPTION 2: Copy-paste the script content directly into Nano SSH session
# (See diagnose_lidar_nano.sh for full script)

cat << 'EOF'
==========================================
QUICK COPY-PASTE VERSION FOR NANO
==========================================

Copy everything below this line and paste into your Nano SSH session:

EOF

cat tools/lidar/diagnose_lidar_nano.sh
