#!/usr/bin/env python3
"""One-time patch: fix JetBotMotorDriver _parsed_packet_count init order. Run on Nano."""
import os
path = os.path.expanduser("~/MicroVIT/robot1/nano_ros1_master/src/jetbot_nano_bringup/nodes/jetbot_motor_driver.py")
if not os.path.isfile(path):
    print("File not found:", path)
    exit(1)
with open(path) as f:
    s = f.read()
old = """        # Subscribers
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        # Start read thread
        self.running = True
        self.read_thread = threading.Thread(target=self.read_serial_thread)
        self.read_thread.daemon = True
        self.read_thread.start()

        # Diagnostics
        self._bad_checksum_count = 0
        self._parsed_packet_count = 0
        
        # Timer for safety stop"""
new = """        # Subscribers
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        # Diagnostics (must exist before read thread uses them)
        self._bad_checksum_count = 0
        self._parsed_packet_count = 0
        
        # Start read thread
        self.running = True
        self.read_thread = threading.Thread(target=self.read_serial_thread)
        self.read_thread.daemon = True
        self.read_thread.start()
        
        # Timer for safety stop"""
if old not in s:
    print("Patch already applied or file format different. Check file manually.")
    exit(0)
s = s.replace(old, new, 1)
with open(path, "w") as f:
    f.write(s)
print("Patched:", path)
