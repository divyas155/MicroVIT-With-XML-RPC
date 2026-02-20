#!/usr/bin/env python3
"""
Check what the device actually expects by reading what it sends
and trying different initialization sequences
"""

import serial
import time

port = "/dev/ttyACM0"
baud = 115200

print("=" * 60)
print("DEVICE PROTOCOL ANALYSIS")
print("=" * 60)
print()

try:
    ser = serial.Serial(port, baud, timeout=1)
    print(f"✅ Opened {port} at {baud} baud")
    time.sleep(0.5)
    
    # Step 1: Read what device sends (first few packets)
    print("\n[Step 1] Reading what device sends (first 3 packets)...")
    for i in range(3):
        data = ser.read(100)
        if data:
            hex_str = ' '.join(['0x%02X' % b for b in data[:20]])  # First 20 bytes
            print(f"  Packet {i+1}: {hex_str}...")
            print(f"    Length: {len(data)} bytes")
            if len(data) >= 3:
                print(f"    Header: 0x%02X 0x%02X 0x%02X" % (data[0], data[1], data[2]))
        time.sleep(0.5)
    
    # Step 2: Try sending command WITHOUT reading first
    print("\n[Step 2] Sending command (no read first)...")
    packet = bytearray([0xAA, 0x55, 0x06, 0x11, 0x32, 0x00, 0x32, 0x00, 0x7A])
    ser.write(packet)
    print(f"  Sent: {' '.join(['0x%02X' % b for b in packet])}")
    time.sleep(2)
    print("  Did robot move? (check hardware)")
    
    # Step 3: Try reading THEN sending
    print("\n[Step 3] Reading one packet THEN sending command...")
    data = ser.read(100)
    if data:
        print(f"  Read: {len(data)} bytes")
    packet = bytearray([0xAA, 0x55, 0x06, 0x11, 0x32, 0x00, 0x32, 0x00, 0x7A])
    ser.write(packet)
    print(f"  Sent: {' '.join(['0x%02X' % b for b in packet])}")
    time.sleep(2)
    print("  Did robot move? (check hardware)")
    
    # Step 4: Try sending initialization command (type 0x12 = PID?)
    print("\n[Step 4] Trying initialization command (type 0x12)...")
    # PID command: [AA 55 size 0x12 pid_data checksum]
    # Let's try a simple one
    init_packet = bytearray([0xAA, 0x55, 0x06, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00])
    checksum = sum(init_packet[:-1]) & 0xFF
    init_packet[-1] = checksum
    ser.write(init_packet)
    print(f"  Sent init: {' '.join(['0x%02X' % b for b in init_packet])}")
    time.sleep(0.5)
    
    # Now send velocity command
    packet = bytearray([0xAA, 0x55, 0x06, 0x11, 0x32, 0x00, 0x32, 0x00, 0x7A])
    ser.write(packet)
    print(f"  Sent velocity: {' '.join(['0x%02X' % b for b in packet])}")
    time.sleep(2)
    print("  Did robot move? (check hardware)")
    
    # Step 5: Try continuous sending (10 times per second)
    print("\n[Step 5] Sending command continuously (10 times)...")
    packet = bytearray([0xAA, 0x55, 0x06, 0x11, 0x64, 0x00, 0x64, 0x00, 0xDE])  # Speed 100
    for i in range(10):
        ser.write(packet)
        print(f"  Sent {i+1}/10", end='\r')
        time.sleep(0.1)
    print("\n  Did robot move? (check hardware)")
    
    # Stop
    print("\n[Step 6] Sending stop command...")
    packet = bytearray([0xAA, 0x55, 0x06, 0x11, 0x00, 0x00, 0x00, 0x00, 0x16])
    ser.write(packet)
    time.sleep(1)
    
    ser.close()
    
    print("\n" + "=" * 60)
    print("ANALYSIS COMPLETE")
    print("=" * 60)
    print("\nQuestions:")
    print("1. Did robot move during ANY step? (Which one?)")
    print("2. What did device send in Step 1? (Copy the hex values)")
    print("3. Is motor power ON?")
    print("4. Are motors physically connected?")
    
except Exception as e:
    print(f"❌ Error: {e}")
    import traceback
    traceback.print_exc()

