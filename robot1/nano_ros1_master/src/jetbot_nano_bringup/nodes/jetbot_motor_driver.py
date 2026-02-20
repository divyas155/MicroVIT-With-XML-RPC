#!/usr/bin/env python3
"""
JetBot Pro Motor Driver - Python Implementation
Replaces the C++ jetbot node with a Python version that's more reliable.

Protocol:
- Header: 0xAA 0x55
- Velocity command: [AA 55 0B 11 vx_h vx_l vy_h vy_l vyaw_h vyaw_l checksum]
- Subscribes to /cmd_vel and sends commands to motor controller via serial
"""

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import serial
import struct
import threading
import time
import math

# Protocol constants
HEAD1 = 0xAA
HEAD2 = 0x55
SEND_TYPE_VELOCITY = 0x11
RECV_TYPE_ODOM_IMU = 0x01

class JetBotMotorDriver:
    def __init__(self):
        rospy.init_node('jetbot_motor_driver', anonymous=False)
        
        # Parameters
        self.port_name = rospy.get_param('~port_name', '/dev/ttyACM0')
        self.baud_rate = rospy.get_param('~baud_rate', 115200)
        self.linear_correction = rospy.get_param('~linear_correction', 1.0)
        self.angular_correction = rospy.get_param('~angular_correction', 1.0)
        self.cmd_timeout = rospy.get_param('~cmd_timeout', 0.5)  # Stop if no cmd for 0.5s
        
        # State
        self.last_cmd_time = rospy.Time.now()
        self.current_vx = 0.0
        self.current_vyaw = 0.0
        self.serial_lock = threading.Lock()
        
        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        
        # Initialize serial
        self.ser = None
        self.connect_serial()
        
        # Publishers
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.imu_pub = rospy.Publisher('/imu', Imu, queue_size=10)
        
        # Subscribers
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        
        # Diagnostics (must exist before read thread uses them)
        self._bad_checksum_count = 0
        self._parsed_packet_count = 0
        
        # Start read thread
        self.running = True
        self.read_thread = threading.Thread(target=self.read_serial_thread)
        self.read_thread.daemon = True
        self.read_thread.start()
        
        # Timer for safety stop
        self.safety_timer = rospy.Timer(rospy.Duration(0.1), self.safety_check)
        
        rospy.loginfo(f"[JetBot Motor Driver] Started on {self.port_name}")
        rospy.loginfo(f"[JetBot Motor Driver] Subscribed to /cmd_vel, publishing /odom and /imu")
    
    def connect_serial(self):
        """Connect to serial port"""
        try:
            self.ser = serial.Serial(
                self.port_name,
                self.baud_rate,
                timeout=0.1
            )
            time.sleep(0.5)  # Wait for connection
            self.ser.reset_input_buffer()
            rospy.loginfo(f"[JetBot Motor Driver] Serial connected to {self.port_name}")
            return True
        except Exception as e:
            rospy.logerr(f"[JetBot Motor Driver] Serial connection failed: {e}")
            return False
    
    def checksum(self, data):
        """Calculate checksum"""
        return sum(data) & 0xFF
    
    def set_velocity(self, vx, vy, vyaw):
        """Send velocity command to motor controller"""
        if self.ser is None or not self.ser.is_open:
            return False
        
        # Apply corrections
        vx = vx * self.linear_correction
        vyaw = vyaw * self.angular_correction
        
        # Convert to int16 (mm/s and mrad/s)
        vx_int = int(vx * 1000)
        vy_int = int(vy * 1000)
        vyaw_int = int(vyaw * 1000)
        
        # Build packet
        packet = bytearray([
            HEAD1, HEAD2,
            0x0B,  # Length
            SEND_TYPE_VELOCITY,
            (vx_int >> 8) & 0xFF,
            vx_int & 0xFF,
            (vy_int >> 8) & 0xFF,
            vy_int & 0xFF,
            (vyaw_int >> 8) & 0xFF,
            vyaw_int & 0xFF,
        ])
        packet.append(self.checksum(packet))
        
        try:
            with self.serial_lock:
                self.ser.write(packet)
            return True
        except Exception as e:
            rospy.logwarn(f"[JetBot Motor Driver] Write error: {e}")
            return False
    
    def cmd_vel_callback(self, msg):
        """Handle velocity commands"""
        self.last_cmd_time = rospy.Time.now()
        self.current_vx = msg.linear.x
        self.current_vyaw = msg.angular.z
        
        # Send to motor
        self.set_velocity(msg.linear.x, 0.0, msg.angular.z)
        rospy.logdebug(f"[JetBot Motor Driver] cmd_vel: vx={msg.linear.x:.2f}, vyaw={msg.angular.z:.2f}")
    
    def safety_check(self, event):
        """Stop motors if no command received recently"""
        if (rospy.Time.now() - self.last_cmd_time).to_sec() > self.cmd_timeout:
            if self.current_vx != 0.0 or self.current_vyaw != 0.0:
                self.set_velocity(0.0, 0.0, 0.0)
                self.current_vx = 0.0
                self.current_vyaw = 0.0
                rospy.logdebug("[JetBot Motor Driver] Safety stop - no cmd_vel")
    
    def read_serial_thread(self):
        """Read and parse data from motor controller"""
        buffer = bytearray()
        
        while self.running and not rospy.is_shutdown():
            if self.ser is None or not self.ser.is_open:
                time.sleep(1)
                self.connect_serial()
                continue
            
            try:
                with self.serial_lock:
                    if self.ser.in_waiting > 0:
                        data = self.ser.read(self.ser.in_waiting)
                        buffer.extend(data)
            except Exception as e:
                rospy.logwarn(f"[JetBot Motor Driver] Read error: {e}")
                time.sleep(0.1)
                continue
            
            # Parse packets from buffer.
            # IMPORTANT: The protocol includes a length byte. Do not assume a fixed packet size.
            # Format: [AA 55 LEN TYPE ... PAYLOAD ... CHECKSUM]
            while len(buffer) >= 4:
                # Find header
                try:
                    idx = buffer.index(HEAD1)
                    if idx > 0:
                        buffer = buffer[idx:]
                except ValueError:
                    buffer.clear()
                    break

                if len(buffer) < 2:
                    break

                if buffer[1] != HEAD2:
                    buffer.pop(0)
                    continue

                if len(buffer) < 3:
                    break

                pkt_len = int(buffer[2])
                # Sanity check: must at least contain AA 55 LEN TYPE CHK = 5 bytes.
                if pkt_len < 5:
                    # Corrupt length; drop one byte and resync.
                    buffer.pop(0)
                    continue

                if len(buffer) < pkt_len:
                    # Wait for more bytes.
                    break

                packet = buffer[:pkt_len]

                # Verify checksum (last byte)
                if self.checksum(packet[:-1]) == packet[-1]:
                    self._parsed_packet_count += 1
                    self.parse_odom_imu(packet)
                else:
                    self._bad_checksum_count += 1
                    # Log occasionally so we can diagnose protocol mismatches without spamming.
                    if (self._bad_checksum_count % 200) == 1:
                        rospy.logwarn(
                            f"[JetBot Motor Driver] Bad checksum ({self._bad_checksum_count}); "
                            f"last_len={pkt_len}, parsed_ok={self._parsed_packet_count}"
                        )

                buffer = buffer[pkt_len:]
            
            time.sleep(0.01)
    
    def parse_odom_imu(self, packet):
        """Parse odometry and IMU data from packet"""
        try:
            # Packet format (after header):
            # [2]: length, [3]: type
            # [4-5]: vx, [6-7]: vy, [8-9]: vyaw
            # [10-13]: x, [14-17]: y, [18-21]: yaw
            # [22-23]: accel_x, [24-25]: accel_y, [26-27]: accel_z
            # [28-29]: gyro_x, [30-31]: gyro_y, [32-33]: gyro_z
            # etc.
            
            if packet[3] != RECV_TYPE_ODOM_IMU:
                return
            
            # Extract velocities (int16, mm/s)
            vx = struct.unpack('>h', bytes(packet[4:6]))[0] / 1000.0
            vy = struct.unpack('>h', bytes(packet[6:8]))[0] / 1000.0
            vyaw = struct.unpack('>h', bytes(packet[8:10]))[0] / 1000.0
            
            # Extract position (int32, mm)
            x = struct.unpack('>i', bytes(packet[10:14]))[0] / 1000.0
            y = struct.unpack('>i', bytes(packet[14:18]))[0] / 1000.0
            yaw = struct.unpack('>i', bytes(packet[18:22]))[0] / 1000.0
            
            # Publish odometry
            odom = Odometry()
            odom.header.stamp = rospy.Time.now()
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_footprint"
            
            odom.pose.pose.position.x = x
            odom.pose.pose.position.y = y
            odom.pose.pose.position.z = 0.0
            
            # Convert yaw to quaternion
            qz = math.sin(yaw / 2.0)
            qw = math.cos(yaw / 2.0)
            odom.pose.pose.orientation.z = qz
            odom.pose.pose.orientation.w = qw
            
            odom.twist.twist.linear.x = vx
            odom.twist.twist.linear.y = vy
            odom.twist.twist.angular.z = vyaw
            
            self.odom_pub.publish(odom)
            
            # Extract and publish IMU (if available in packet)
            if len(packet) >= 34:
                imu = Imu()
                imu.header.stamp = rospy.Time.now()
                imu.header.frame_id = "base_imu_link"
                
                # Accelerometer (int16, mg -> m/s^2)
                ax = struct.unpack('>h', bytes(packet[22:24]))[0] * 9.81 / 1000.0
                ay = struct.unpack('>h', bytes(packet[24:26]))[0] * 9.81 / 1000.0
                az = struct.unpack('>h', bytes(packet[26:28]))[0] * 9.81 / 1000.0
                
                # Gyroscope (int16, mdeg/s -> rad/s)
                gx = struct.unpack('>h', bytes(packet[28:30]))[0] * math.pi / 180000.0
                gy = struct.unpack('>h', bytes(packet[30:32]))[0] * math.pi / 180000.0
                gz = struct.unpack('>h', bytes(packet[32:34]))[0] * math.pi / 180000.0
                
                imu.linear_acceleration.x = ax
                imu.linear_acceleration.y = ay
                imu.linear_acceleration.z = az
                
                imu.angular_velocity.x = gx
                imu.angular_velocity.y = gy
                imu.angular_velocity.z = gz
                
                self.imu_pub.publish(imu)
                
        except Exception as e:
            rospy.logdebug(f"[JetBot Motor Driver] Parse error: {e}")
    
    def shutdown(self):
        """Clean shutdown"""
        self.running = False
        self.set_velocity(0.0, 0.0, 0.0)
        if self.ser and self.ser.is_open:
            self.ser.close()
        rospy.loginfo("[JetBot Motor Driver] Shutdown complete")

if __name__ == '__main__':
    try:
        driver = JetBotMotorDriver()
        rospy.on_shutdown(driver.shutdown)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
