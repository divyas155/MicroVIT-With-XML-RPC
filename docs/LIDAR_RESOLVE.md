# Resolving LiDAR: 80008000 / RESULT_OPERATION_TIMEOUT

If you see **Can not start scan: 80008000!** or **Error, operation time out. RESULT_OPERATION_TIMEOUT!**, follow these steps in order.

**For a full investigation (why it fails, what the codes mean, exact action order):** see **docs/LIDAR_INVESTIGATION_REPORT.md**.

---

## 1. Physical check (most important)

**The LiDAR disc must be spinning.** If it is not:

- **Cause:** USB usually can’t supply enough current for the motor (A2/A3/S1 need ~800mA–1A).
- **Fix:** Use an **external 5V power supply** (e.g. 2A wall adapter or power bank):
  - Connect 5V and GND to the LiDAR’s **power connector** (red/black), **or**
  - Use a USB cable that carries only power (no data) from a charger into the LiDAR’s USB power port (if it has one).
  - Keep the **data USB** connected from LiDAR to the Nano.
- **Check:** After powering, the disc should rotate and the LED should be on.

If the motor still doesn’t spin with external 5V, try another cable/port and rule out a faulty unit.

---

## 2. Baud rate (on the Nano)

The repo default is **256000** (typical for A2/A3/S1). Some units use **115200** (e.g. A1, some A2).

**Option A – Quick test (one baud)**  
With **roscore** already running in another terminal:

```bash
# Try 256000 first (default in launch)
rosrun rplidar_ros rplidarNode _serial_port:=/dev/ttyUSB0 _serial_baudrate:=256000
```

In a third terminal: `rostopic hz /scan` — you should see ~5–10 Hz. If you get timeout, stop (Ctrl+C) and try:

```bash
rosrun rplidar_ros rplidarNode _serial_port:=/dev/ttyUSB0 _serial_baudrate:=115200
```

**Option B – Detailed checkup (recommended)**  
Run the full diagnostic on the Nano (device, USB, raw read, rplidarNode at each baud). From your **Mac**:

```bash
scp tools/lidar/detailed_lidar_checkup.sh jetbot@10.13.68.184:~/
ssh jetbot@10.13.68.184
bash ~/detailed_lidar_checkup.sh
```

Report is printed and saved to `/tmp/lidar_checkup_report.txt`. It will tell you if the issue is power (no raw data), baud, or port.

**Option C – Test all baud rates only**  
```bash
scp tools/lidar/test_all_bauds.sh jetbot@10.13.68.184:~/
ssh jetbot@10.13.68.184
source /opt/ros/melodic/setup.bash && bash ~/test_all_bauds.sh
```

---

## 3. Port / device check (if you suspect wrong port)

If raw read is always 0 and you think the port might be wrong, run the port diagnostic on the Nano:

```bash
# From Mac: scp tools/lidar/check_lidar_port.sh jetbot@10.13.68.184:~/
# On Nano:
bash ~/check_lidar_port.sh
cat /tmp/lidar_port_report.txt
```

It lists all `/dev/ttyUSB*` and `/dev/ttyACM*`, which process (if any) has each open, `dmesg`/`lsusb`, and tries a 3 s raw read at 115200 and 256000 on each. Use the device that gives non-zero bytes (or fix power if all give 0). If the LiDAR is on e.g. `/dev/ttyUSB1`, use in launch: `lidar_serial_port:=/dev/ttyUSB1`.

---

## 4. Permissions (on the Nano)

If you see “Permission denied” on `/dev/ttyUSB0`:

```bash
sudo chmod 666 /dev/ttyUSB0
# Or add yourself to dialout (then log out and back in):
sudo usermod -a -G dialout $USER
```

---

## 5. Full bringup with real LiDAR

Once you know the **working baud** (256000 or 115200):

1. **Copy the updated launch** from your Mac to the Nano (so it uses 256000 and optional camera_device):

   ```bash
   scp "robot1/nano_ros1_master/src/jetbot_nano_bringup/launch/nano_bringup_full.launch" \
       jetbot@10.13.68.184:~/MicroVIT/robot1/nano_ros1_master/src/jetbot_nano_bringup/launch/
   ```

2. **On the Nano**, start full bringup:

   ```bash
   cd ~/MicroVIT/robot1/nano_ros1_master && source devel/setup.bash
   roslaunch jetbot_nano_bringup nano_bringup_full.launch use_lidar:=true
   ```

   If you had to use **115200** in step 2, override baud:

   ```bash
   roslaunch jetbot_nano_bringup nano_bringup_full.launch use_lidar:=true lidar_serial_baudrate:=115200
   ```

3. **Confirm:**  
   `rostopic hz /scan` — should show a stable rate; no “Can not start scan: 80008000!” in the bringup terminal.

---

## 6. Verify serial (optional)

- On the Nano, run the minimal command/response test (requires `pip install pyserial`):
  ```bash
  python3 tools/lidar/lidar_serial_ping.py /dev/ttyUSB0 256000
  ```
  If you see "Valid RPLidar response header" and bytes read, the serial path works and the issue is likely **power** (see **docs/LIDAR_INVESTIGATION_REPORT.md**).

## 7. If it still fails

- Use **dummy LiDAR** so the rest of the pipeline runs (Orin will still get camera + odom):

  ```bash
  roslaunch jetbot_nano_bringup nano_bringup_full.launch use_lidar:=false
  ```

- **Motor spinning but still 0 bytes?** See **docs/lidar_motor_spinning_no_data.md** (different USB port, reseat internal cable, try 512000 baud).
- See **docs/lidar_power_fix.md** for power wiring and **docs/lidar_troubleshooting.md** for more options.
- **Full analysis:** **docs/LIDAR_INVESTIGATION_REPORT.md**.

---

## Summary

| Step | Action |
|------|--------|
| 1 | Ensure LiDAR motor is spinning (external 5V if needed). |
| 2 | On Nano: test 256000 then 115200 (or run `test_all_bauds.sh`). |
| 3 | Run `check_lidar_port.sh` on Nano; use the port that gives non-zero bytes (or fix power). |
| 4 | Fix permissions if needed (`sudo chmod 666 /dev/ttyUSBx`). |
| 5 | Copy updated launch to Nano; run full bringup with `use_lidar:=true` and correct `lidar_serial_port` / `lidar_serial_baudrate`. |
| 6 | Optional: run `lidar_serial_ping.py` to confirm serial command/response; see **LIDAR_INVESTIGATION_REPORT.md** for full analysis. |
| 7 | If still failing, use `use_lidar:=false` and rely on dummy LiDAR. |
