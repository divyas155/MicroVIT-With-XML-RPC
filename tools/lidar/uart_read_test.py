#!/usr/bin/env python3
"""
Raw UART serial read test for LiDAR
Tests direct serial communication to LiDAR device
"""

import serial
import sys
import time

def test_uart(port="/dev/ttyUSB0", baudrate=115200, timeout=2.0):
    """Test UART connection to LiDAR"""
    print(f"Testing UART: {port} at {baudrate} baud")
    print("=" * 50)
    
    try:
        # Open serial port
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=timeout,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        )
        
        print(f"✅ Serial port opened: {port}")
        print(f"   Baudrate: {baudrate}")
        print(f"   Timeout: {timeout}s")
        print("")
        
        # Try to read data
        print("Reading data (5 seconds)...")
        start_time = time.time()
        bytes_received = 0
        
        while time.time() - start_time < 5.0:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                bytes_received += len(data)
                print(f"   Received {len(data)} bytes: {data[:20].hex()}...")
            time.sleep(0.1)
        
        ser.close()
        
        if bytes_received > 0:
            print(f"\n✅ SUCCESS: Received {bytes_received} bytes total")
            return True
        else:
            print(f"\n❌ FAILURE: No data received (0 bytes)")
            print("   Possible issues:")
            print("   - LiDAR motor not spinning")
            print("   - Wrong baud rate")
            print("   - Wrong UART port")
            print("   - Power issue")
            return False
            
    except serial.SerialException as e:
        print(f"❌ Serial port error: {e}")
        print("   Check:")
        print("   - Device exists: ls -l /dev/ttyUSB*")
        print("   - Permissions: sudo usermod -aG dialout $USER")
        print("   - Device not in use by another process")
        return False
    except Exception as e:
        print(f"❌ Error: {e}")
        return False

if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
    baudrate = int(sys.argv[2]) if len(sys.argv) > 2 else 115200
    
    # Test common baud rates
    baudrates = [115200, 230400, 256000]
    
    for baud in baudrates:
        print(f"\n{'='*50}")
        result = test_uart(port, baud)
        if result:
            print(f"\n✅ Working baud rate found: {baud}")
            break
        time.sleep(1)
    
    if not result:
        print(f"\n❌ No working baud rate found. Check hardware connections.")
