#!/bin/bash
# Thorough port/device check for LiDAR - run on Nano
# Finds which /dev/tty* is the LiDAR and why raw read might be 0 bytes.
# Usage: bash check_lidar_port.sh
# Run when no roslaunch/rplidarNode is using the port.

set -e
REPORT="/tmp/lidar_port_report.txt"
exec 1> >(tee "$REPORT")
exec 2>&1

echo "=============================================="
echo "  LIDAR PORT & DEVICE CHECK"
echo "  $(date -Iseconds)"
echo "=============================================="
echo ""

pkill -9 rplidarNode 2>/dev/null || true
sleep 1

shopt -s nullglob 2>/dev/null || true

echo "1. ALL SERIAL DEVICES (ttyUSB*, ttyACM*)"
echo "──────────────────────────────────────────────"
for dev in /dev/ttyUSB* /dev/ttyACM*; do
  [ -e "$dev" ] || continue
  echo "  $dev:"
  ls -l "$dev"
  stat --printf="    major:minor=%t:%T  perms=%a\n" "$dev" 2>/dev/null || true
  # Who has it open?
  if command -v fuser >/dev/null 2>&1; then
    if fuser "$dev" 2>/dev/null; then
      echo "    ⚠️  IN USE by:"
      fuser -v "$dev" 2>/dev/null | sed 's/^/      /' || true
    else
      echo "    (not open by any process)"
    fi
  fi
  echo ""
done
echo ""

echo "2. DMESG – which USB device is which port?"
echo "──────────────────────────────────────────────"
echo "  (Look for 'cp210\|ch340\|ftdi\|pl2303\|ttyUSB\|ttyACM' and which bus/port)"
dmesg 2>/dev/null | grep -iE "usb.*tty|ttyUSB|ttyACM|cp210|ch340|ftdi|pl2303|cdc_acm" | tail -30 || echo "  (need sudo for dmesg?)"
echo ""
echo "  Full recent USB attach/detach:"
dmesg 2>/dev/null | grep -i "usb.*attach\|usb.*detach\|pl2303\|ch341\|cp210\|ftdi_sio\|cdc_acm" | tail -20 || true
echo ""

echo "3. LSUSB – USB devices (vendor:product)"
echo "──────────────────────────────────────────────"
lsusb 2>/dev/null
echo ""
echo "  (Common: 10c4:ea60 = CP210x, 1a86:7523 = CH340, 0403:6001 = FTDI)"
echo ""

echo "4. RAW READ TEST ON EACH PORT (3 sec, 115200 then 256000)"
echo "──────────────────────────────────────────────"
echo "  Motor is usually /dev/ttyACM0 – we skip writing to it to avoid moving the robot."
echo "  Testing only /dev/ttyUSB* for LiDAR."
echo ""

for dev in /dev/ttyUSB*; do
  [ -e "$dev" ] || continue
  for baud in 115200 256000 512000 921600 460800; do
    stty -F "$dev" $baud cs8 -cstopb -parenb raw -echo 2>/dev/null || true
    BYTES=$(timeout 3 cat "$dev" 2>/dev/null | wc -c)
    echo "  $dev @ $baud: $BYTES bytes"
  done
  echo ""
done
if ! compgen -G /dev/ttyUSB* >/dev/null 2>&1; then
  echo "  (no /dev/ttyUSB* – trying ttyACM if LiDAR on second ACM device)"
  for dev in /dev/ttyACM*; do
    [ -e "$dev" ] || continue
    [ "$dev" = "/dev/ttyACM0" ] && echo "  Skipping $dev (motor)" && continue
    for baud in 115200 256000; do
      stty -F "$dev" $baud cs8 -cstopb -parenb raw -echo 2>/dev/null || true
      BYTES=$(timeout 3 cat "$dev" 2>/dev/null | wc -c)
      echo "  $dev @ $baud: $BYTES bytes"
    done
  done
fi

echo "5. PERMISSIONS"
echo "──────────────────────────────────────────────"
for dev in /dev/ttyUSB* /dev/ttyACM*; do
  [ -e "$dev" ] || continue
  if [ -r "$dev" ] && [ -w "$dev" ]; then
    echo "  $dev: read/write OK"
  else
    echo "  $dev: ❌ no read/write (run: sudo chmod 666 $dev)"
  fi
done
echo ""

echo "6. UDEV PERSISTENT NAMES (by-id, by-serial)"
echo "──────────────────────────────────────────────"
ls -l /dev/serial/by-id/ 2>/dev/null || echo "  /dev/serial/by-id/ not found or empty"
ls -l /dev/serial/by-path/ 2>/dev/null || echo "  /dev/serial/by-path/ not found or empty"
echo ""

echo "7. RECOMMENDATION"
echo "──────────────────────────────────────────────"
USB_COUNT=$(ls /dev/ttyUSB* 2>/dev/null | wc -l)
if [ "$USB_COUNT" -eq 0 ]; then
  echo "  ❌ No /dev/ttyUSB* found. LiDAR may be unplugged or using ttyACM."
  echo "     Try: ls /dev/ttyACM* (if LiDAR appears as ACM, use that in launch)."
elif [ "$USB_COUNT" -gt 1 ]; then
  echo "  ⚠️  Multiple ttyUSB ports. Use the one that gave non-zero bytes above."
  echo "     If all gave 0: see docs/lidar_motor_spinning_no_data.md"
else
  echo "  Single ttyUSB: /dev/ttyUSB0. If raw read was 0 at all bauds:"
  echo "  • Try a DIFFERENT USB PORT on the Nano (unplug LiDAR, plug into another port), then run this script again."
  echo "  • Reseat the internal cable in the LiDAR (CP2102 board to main PCB)."
  echo "  • Try a different USB cable (data-capable)."
  echo "  • See docs/lidar_motor_spinning_no_data.md"
fi
echo ""
echo "Report saved to: $REPORT"
