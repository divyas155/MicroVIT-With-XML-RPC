#!/usr/bin/env python3
"""
Minimal RPLidar serial test: send GET_DEVICE_INFO (0xA5 0x50) and read response.
Use this to verify that the serial link works in both directions when we send a command.
Run on the Nano (or any host with the LiDAR on the given port). No ROS required.

Usage:
  python3 lidar_serial_ping.py /dev/ttyUSB0 [baudrate]
  python3 lidar_serial_ping.py /dev/ttyUSB0 256000

Requires: pyserial (pip install pyserial)
"""

import sys
import time

# RPLidar GET_DEVICE_INFO command
CMD_GET_INFO = bytes([0xA5, 0x50])

def main():
    if len(sys.argv) < 2:
        print("Usage: %s <serial_port> [baudrate]" % sys.argv[0], file=sys.stderr)
        print("  e.g. %s /dev/ttyUSB0 256000" % sys.argv[0], file=sys.stderr)
        sys.exit(1)

    port = sys.argv[1]
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 256000

    try:
        import serial
    except ImportError:
        print("Install pyserial: pip install pyserial", file=sys.stderr)
        sys.exit(2)

    print("Opening %s at %d baud..." % (port, baud))
    try:
        ser = serial.Serial(port, baud, timeout=0.5, write_timeout=1)
    except Exception as e:
        print("ERROR opening port: %s" % e, file=sys.stderr)
        sys.exit(3)

    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(0.1)

        print("Sending GET_DEVICE_INFO (0xA5 0x50)...")
        ser.write(CMD_GET_INFO)
        ser.flush()

        time.sleep(0.3)
        data = b""
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            chunk = ser.read(256)
            if chunk:
                data += chunk
            else:
                if data:
                    break
            time.sleep(0.05)

        print("Read %d bytes" % len(data))
        if data:
            # Response should start with 0xA5 0x5A (response header)
            hex_preview = " ".join("%02X" % b for b in data[:64])
            if len(data) > 64:
                hex_preview += " ..."
            print("First bytes (hex): %s" % hex_preview)
            if len(data) >= 2 and data[0] == 0xA5 and data[1] == 0x5A:
                print("OK: Valid RPLidar response header (0xA5 0x5A). Serial command/response path works.")
            elif len(data) >= 2:
                print("WARN: Response does not start with 0xA5 0x5A (got %02X %02X). Check baud or device." % (data[0], data[1]))
        else:
            print("No data received. Possible causes: wrong port, wrong baud, or device not responding to commands.")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
