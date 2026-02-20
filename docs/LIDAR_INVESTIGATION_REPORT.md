# LiDAR Thorough Investigation Report

This document summarizes the **root cause analysis** for RPLidar failures on the Jetson Nano (get_info and get_health succeed, but **“Can not start scan: 80008000!”** or **80008002**), and gives a clear “do this next” path.

---

## 1. What We Know

### 1.1 Observed behaviour

- **rplidarNode** connects to the device and gets:
  - Serial number, Firmware (e.g. 1.29), Hardware Rev (e.g. 7), Health status **OK**.
- Then it fails with:
  - **“Can not start scan: 80008000!”** or **“Can not start scan: 80008002!”**
  - Sometimes **“Error, operation time out. RESULT_OPERATION_TIMEOUT!”** or **“cannot retrieve rplidar health code: 80008002”**.
- **Raw read** from `/dev/ttyUSB0` (e.g. `cat /dev/ttyUSB0` for a few seconds): **0 bytes** at 115200, 256000, 512000, 921600, 460800.
- **Motor is spinning** (user confirmed) → power to the motor is sufficient.
- **Port check**: ttyUSB0 = CP2102 (LiDAR), ttyACM0/ttyACM1 = TinyUSB (motor). No port mix-up; permissions OK; no process holding the port.

### 1.2 Important conclusion

- **Serial link is working for command/response.**  
  If the node did not receive any data, we would not see S/N, firmware, or health. So:
  - **getDeviceInfo** and **getHealth** succeed → the LiDAR **does** send data back when the host sends the right commands.
- **Raw `cat` giving 0 bytes is expected** when no command is sent: RPLidar is **command–response**; it does not stream data until the host sends a “start scan” (and the device accepts it). So “0 bytes on raw read” does **not** by itself mean “broken data line”; it can mean “no scan started”.
- The **real failure** is: **startScan()** (or the scan-start step) returns **0x80008000** or **0x80008002** instead of success.

---

## 2. Where It Fails (rplidar_ros node)

From the **rplidar_ros** `node.cpp` sequence:

1. Connect to serial port at chosen baud (e.g. 256000).
2. **getDeviceInfo** → we see S/N, firmware, hardware.
3. Optional **reset** if `initial_reset:=true`.
4. **getHealth** → we see “RPLidar health status : OK.”
5. For **A-series** (A1/A2/A3): **setMotorSpeed(600)** to spin the motor.
6. **startScan(false, true, 0, &current_scan_mode)** (or **startScanExpress** if a scan mode is set).
7. If `startScan` / `startScanExpress` returns failure → **“Can not start scan: %08x!”** (80008000 or 80008002).

So the failure is **at step 6**: the device accepts info/health commands but **rejects or times out** on the **scan start** command.

---

## 3. Error Codes (80008000 vs 80008002)

- **0x80008000**  
  - Commonly reported as **power-related** (e.g. insufficient USB power to the LiDAR).
- **0x80008002**  
  - “Can not start scan” / “cannot retrieve rplidar health code” in different contexts.  
  - In practice also treated as **power / power-delivery** related by Slamtec and community (e.g. [Slamtec/rplidar_ros#86](https://github.com/Slamtec/rplidar_ros/issues/86)): e.g. Raspberry Pi powered via GPIO instead of USB-C, or weak power to the host.  
  - So even with **health OK**, the **scan start** can fail due to **power delivery** (e.g. host USB bus sagging when the scan starts).

**Conclusion:** Both codes point to **power / power delivery** as the main suspect, not “no serial data at all”.

---

## 4. Why “Motor Spinning” Is Not Enough

- The motor can spin from:
  - **setMotorSpeed(600)** (PWM from the driver), or  
  - External 5V, or  
  - Residual charge / low load.
- When **scan** starts, the device draws **more current** (laser + processing). If the **host (Nano) USB power** is marginal, the **USB voltage can sag** and the LiDAR may:
  - Refuse the scan start, or  
  - Time out → 80008000 / 80008002 / RESULT_OPERATION_TIMEOUT.

So: **motor spinning ≠ sufficient power for scan**.

---

## 5. Recommended Action Order

Do these in order. Stop when scan works.

### 5.1 Power the Jetson Nano properly (highest impact)

- **Power the Nano via the barrel jack** with a **stable 5V supply** (official 5V/4A or equivalent).  
- **Avoid** powering only via USB from a weak source (e.g. a low-current USB port).  
- If you use a hub, use a **powered** USB hub for the LiDAR.  
- **Goal:** No voltage sag when the LiDAR starts scanning.

### 5.2 External 5V for the LiDAR (if not already)

- Use an **external 5V** (e.g. 2A adapter or power bank) for the LiDAR’s **power pins** (red/black) or its power-only USB port.  
- Keep **data USB** from LiDAR to Nano.  
- This reduces current draw from the Nano’s USB and can eliminate 80008000/80008002.

### 5.3 Try `initial_reset` and baud

- In launch or when running the node:
  - **initial_reset:=true** (node resets the LiDAR before starting).
  - Try **256000** first; if that fails, try **115200** (and vice versa).  
- Example:
  ```bash
  rosrun rplidar_ros rplidarNode _serial_port:=/dev/ttyUSB0 _serial_baudrate:=256000 _initial_reset:=true
  ```

### 5.4 Verify serial command/response (optional)

- Run the minimal test script that **sends** the RPLidar “Get Device Info” command and **reads** the response:
  ```bash
  python3 tools/lidar/lidar_serial_ping.py /dev/ttyUSB0 256000
  ```
- If this **gets a valid response** (e.g. device info bytes), the serial path is fine and the problem is almost certainly **power** or **scan-start** timing/delivery, not “no data at all”.

### 5.5 Different USB port / cable

- Plug the LiDAR into **another USB port** on the Nano.  
- Use a **data-capable** USB cable (not charge-only).  
- Rule out a bad port or cable.

### 5.6 If still 0 bytes on raw read and no response from script

- Then consider **hardware**: reseat the internal CP2102–to–PCB cable, try another LiDAR unit.  
- See **docs/lidar_motor_spinning_no_data.md**.

### 5.7 Use dummy LiDAR so the rest of the system runs

- Until the real LiDAR is fixed:
  ```bash
  roslaunch jetbot_nano_bringup nano_bringup_full.launch use_lidar:=false
  ```
- Orin and the rest of the pipeline can still run with camera + odom; `/scan` will be from the dummy node.

---

## 6. Summary Table

| Observation              | Interpretation |
|--------------------------|----------------|
| S/N, firmware, health OK | Serial RX works when host sends commands. |
| Raw `cat` = 0 bytes      | Expected if no scan started; not proof of broken link. |
| Motor spinning           | Motor power OK; not proof that scan power is OK. |
| 80008000 / 80008002      | Scan start failed; often power / power delivery. |
| **First thing to try**   | **Stable 5V to Nano + external 5V to LiDAR.** |
| **Then**                 | initial_reset, baud 256000/115200, other USB port/cable. |
| **Last**                 | Hardware (cable/connector/unit). |

---

## 7. References

- [Slamtec/rplidar_ros#86](https://github.com/Slamtec/rplidar_ros/issues/86) – Can not start scan: 80008002 (power / USB-C on RPi4, etc.).  
- **docs/LIDAR_RESOLVE.md** – Step-by-step resolve flow.  
- **docs/lidar_motor_spinning_no_data.md** – When motor spins but no data (different port, cable, internal connector).  
- **docs/lidar_troubleshooting.md** – General LiDAR troubleshooting.  
- **tools/lidar/lidar_serial_ping.py** – Minimal serial command/response test.
