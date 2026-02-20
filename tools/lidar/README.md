# LiDAR Diagnostic Tools

Tools to diagnose and fix LiDAR connection issues (e.g. 80008000 / RESULT_OPERATION_TIMEOUT) on Jetson Nano.

**Full flow:** See **docs/LIDAR_RESOLVE.md** (power, baud, permissions, detailed checkup).

## Quick Start

### 1. Run Detailed Checkup (on Nano) – recommended

```bash
# From Mac: copy then run on Nano
scp tools/lidar/detailed_lidar_checkup.sh jetbot@10.13.68.184:~/
ssh jetbot@10.13.68.184
bash ~/detailed_lidar_checkup.sh
cat /tmp/lidar_checkup_report.txt
```

This checks device, USB, raw read, and rplidarNode at 256000/115200/… and reports whether the issue is power or baud.

**Or** run `./diagnose_lidar_nano.sh` or `./test_all_bauds.sh` on the Nano for baud-only testing.

### 2. Enable Real LiDAR

Default baud in this repo is **256000**. Use **115200** if the checkup shows that works:

```bash
./enable_real_lidar.sh enable 256000   # or 115200
```

### 3. Launch with Real LiDAR

```bash
# On Nano
roslaunch jetbot_nano_bringup nano_bringup_full.launch use_lidar:=true
# Or override baud: lidar_serial_baudrate:=115200
```

## Scripts

### `diagnose_lidar_nano.sh`
Comprehensive diagnostic script that tests multiple baud rates and identifies issues.

**Usage:**
```bash
./diagnose_lidar_nano.sh
```

**Requirements:**
- Run on Jetson Nano (not remote machine)
- ROS Melodic installed
- LiDAR connected to `/dev/ttyUSB0`

### `enable_real_lidar.sh`
Helper script to enable/disable real LiDAR in launch files.

**Usage:**
```bash
./enable_real_lidar.sh enable 256000   # default; use 115200 if timeout
./enable_real_lidar.sh enable 115200
./enable_real_lidar.sh disable         # use dummy LiDAR
```

### `test_lidar_complete.sh`
Complete test suite (alternative to diagnose script).

### `test_lidar_connection.sh`
Basic connection test (checks device, permissions, USB info).

## Manual Testing

If scripts don't work, test manually:

```bash
# Terminal 1: Start roscore
roscore

# Terminal 2: Test baud rate (try 256000 first, then 115200)
rosrun rplidar_ros rplidarNode _serial_port:=/dev/ttyUSB0 _serial_baudrate:=256000

# Terminal 3: Check for data
rostopic echo /scan -n 1
```

## Common Baud Rates

- **RPLidar A1**: 115200
- **RPLidar A2**: 256000
- **RPLidar A3**: 256000
- **RPLidar S1**: 256000

## Troubleshooting

- **Same issue (80008000 / timeout):** [LIDAR_RESOLVE.md](../../docs/LIDAR_RESOLVE.md) (power, baud, detailed checkup).
- [LiDAR Troubleshooting Guide](../../docs/lidar_troubleshooting.md) for more options.

## Copying Scripts to Nano

```bash
# From your development machine
scp tools/lidar/diagnose_lidar_nano.sh jetbot@<nano-ip>:~/
ssh jetbot@<nano-ip>
chmod +x diagnose_lidar_nano.sh
./diagnose_lidar_nano.sh
```
