# Motor Spinning But No Data (0 bytes)

If the LiDAR **motor is spinning** but you still get **0 bytes** from raw read and **Can not start scan: 80008000!** from rplidarNode, the problem is **data connection**, not power.

Try these in order:

---

## 1. Try a different USB port on the Nano

Unplug the LiDAR USB cable from the Nano and plug it into **another USB port**. Then run the port check again:

```bash
bash ~/check_lidar_port.sh
```

Sometimes one port has a bad or flaky data line.

---

## 2. Reseat the internal cable (RPLidar with CP2102)

Many RPLidar units have a **CP2102 USB–serial board** inside, connected to the main PCB by a small cable. If that connector is loose, the motor can run (power is fine) but **no data** reaches the USB.

- **Carefully** open the LiDAR (often two screws on the bottom).
- Find the cable from the USB/CP2102 board to the main board and **reseat** the connector (unplug and plug back in firmly).
- Close the unit and test again.

---

## 3. Try another USB cable

Use a **data-capable** USB cable (not charge-only). Try a different cable if you have one to rule out a bad data line.

---

## 4. Try more baud rates

Some units use **512000** or **921600**. The port check script now tests 115200, 256000, 512000, 921600, 460800. Run it again and see if any baud gives non-zero bytes:

```bash
bash ~/check_lidar_port.sh
cat /tmp/lidar_port_report.txt
```

If e.g. **512000** gives data, use it in the launch:

```bash
roslaunch jetbot_nano_bringup nano_bringup_full.launch use_lidar:=true lidar_serial_baudrate:=512000
```

---

## 5. Test with rplidarNode at 512000

With the motor spinning, try the driver at 512000:

```bash
rosrun rplidar_ros rplidarNode _serial_port:=/dev/ttyUSB0 _serial_baudrate:=512000
```

In another terminal: `rostopic hz /scan`. If you get a rate, use `lidar_serial_baudrate:=512000` in the full launch.

---

## Summary

| Check | Action |
|-------|--------|
| USB port | Plug LiDAR into another USB port on the Nano. |
| Internal cable | Open LiDAR, reseat the CP2102–to–PCB connector. |
| Cable | Try a different USB cable (data-capable). |
| Baud | Run `check_lidar_port.sh` and try 512000 / 921600 in launch or rplidarNode. |

If all baud rates still give 0 bytes after trying another port and cable, the internal data path (cable or board) is likely faulty.
