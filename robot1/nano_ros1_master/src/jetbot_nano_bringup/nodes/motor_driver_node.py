#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import os
import sys

# Try to import hardware libraries
try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    rospy.logwarn("pyserial not installed. Install with: pip3 install pyserial")

try:
    import Adafruit_PCA9685
    PCA9685_AVAILABLE = True
except ImportError:
    PCA9685_AVAILABLE = False
    rospy.logwarn("Adafruit_PCA9685 not installed. Install with: pip3 install adafruit-pca9685")

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False
    rospy.logwarn("RPi.GPIO not available. This is normal if not on Raspberry Pi/Jetson.")


class NanoMotorDriver(object):
    """
    Real motor driver node for Jetson Nano.
    
    Supports multiple hardware types:
    1. Serial motor controller (/dev/ttyACM0 or similar)
    2. PCA9685 I2C PWM controller
    3. Direct GPIO control (fallback)
    
    Subscribes to /cmd_vel and controls actual motors.
    """

    def __init__(self):
        self.node_name = "nano_motor_driver"
        rospy.init_node(self.node_name)

        # Parameters for differential drive kinematics
        self.wheel_base = rospy.get_param("~wheel_base", 0.2)   # meters
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.03)  # meters
        
        # Hardware selection
        self.motor_type = rospy.get_param("~motor_type", "auto")  # auto, serial, pca9685, gpio
        self.serial_port = rospy.get_param("~serial_port", "/dev/ttyACM0")
        self.serial_baud = rospy.get_param("~serial_baud", 115200)
        self.serial_protocol = rospy.get_param("~serial_protocol", "binary")  # binary (jetbot_pro) or text (arduino)
        
        # Motor control objects
        self.serial_conn = None
        self.pca9685 = None
        self.gpio_initialized = False
        
        # Initialize hardware
        self.init_hardware()
        
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback, queue_size=10)
        rospy.loginfo("[%s] Started. Subscribing to /cmd_vel" % self.node_name)
        rospy.loginfo("[%s] Motor type: %s" % (self.node_name, self.motor_type))

    def init_hardware(self):
        """Initialize motor hardware based on available options."""
        
        if self.motor_type == "auto":
            # Try serial first (most common for JetBot Pro)
            if SERIAL_AVAILABLE and os.path.exists(self.serial_port):
                self.motor_type = "serial"
                self.init_serial()
            elif PCA9685_AVAILABLE:
                self.motor_type = "pca9685"
                self.init_pca9685()
            elif GPIO_AVAILABLE:
                self.motor_type = "gpio"
                self.init_gpio()
            else:
                rospy.logerr("[%s] No motor hardware found! Running in dummy mode." % self.node_name)
                self.motor_type = "dummy"
        
        elif self.motor_type == "serial":
            self.init_serial()
        elif self.motor_type == "pca9685":
            self.init_pca9685()
        elif self.motor_type == "gpio":
            self.init_gpio()
        else:
            rospy.logwarn("[%s] Unknown motor_type '%s', using dummy mode" % (self.node_name, self.motor_type))
            self.motor_type = "dummy"

    def init_serial(self):
        """Initialize serial motor controller."""
        if not SERIAL_AVAILABLE:
            rospy.logerr("[%s] pyserial not installed. Install with: pip3 install pyserial" % self.node_name)
            self.motor_type = "dummy"
            return
        
        if not os.path.exists(self.serial_port):
            rospy.logerr("[%s] Serial port %s not found!" % (self.node_name, self.serial_port))
            self.motor_type = "dummy"
            return
        
        try:
            rospy.loginfo("[%s] 🔌 Opening serial port %s at %d baud..." % 
                         (self.node_name, self.serial_port, self.serial_baud))
            self.serial_conn = serial.Serial(self.serial_port, self.serial_baud, timeout=1)
            # Wait for device to initialize (important for binary protocol devices)
            rospy.sleep(0.5)
            rospy.loginfo("[%s] ✅ Serial motor controller initialized on %s (protocol: %s)" % 
                         (self.node_name, self.serial_port, self.serial_protocol))
            rospy.loginfo("[%s] ✅ Serial port is open: %s" % (self.node_name, self.serial_conn.is_open))
        except Exception as e:
            rospy.logerr("[%s] ❌ Failed to open serial port: %s" % (self.node_name, str(e)))
            rospy.logerr("[%s] ❌ Error details: %s" % (self.node_name, repr(e)))
            self.motor_type = "dummy"

    def init_pca9685(self):
        """Initialize PCA9685 I2C PWM controller."""
        if not PCA9685_AVAILABLE:
            rospy.logerr("[%s] Adafruit_PCA9685 not installed. Install with: pip3 install adafruit-pca9685" % self.node_name)
            self.motor_type = "dummy"
            return
        
        try:
            self.pca9685 = Adafruit_PCA9685.PCA9685()
            self.pca9685.set_pwm_freq(60)  # 60Hz frequency
            rospy.loginfo("[%s] PCA9685 initialized" % self.node_name)
        except Exception as e:
            rospy.logerr("[%s] Failed to initialize PCA9685: %s" % (self.node_name, str(e)))
            self.motor_type = "dummy"

    def init_gpio(self):
        """Initialize GPIO pins for direct motor control."""
        if not GPIO_AVAILABLE:
            rospy.logerr("[%s] RPi.GPIO not available" % self.node_name)
            self.motor_type = "dummy"
            return
        
        try:
            GPIO.setmode(GPIO.BOARD)
            # Define motor pins (adjust based on your wiring)
            self.left_motor_pin1 = rospy.get_param("~left_motor_pin1", 11)
            self.left_motor_pin2 = rospy.get_param("~left_motor_pin2", 13)
            self.right_motor_pin1 = rospy.get_param("~right_motor_pin1", 15)
            self.right_motor_pin2 = rospy.get_param("~right_motor_pin2", 16)
            
            GPIO.setup(self.left_motor_pin1, GPIO.OUT)
            GPIO.setup(self.left_motor_pin2, GPIO.OUT)
            GPIO.setup(self.right_motor_pin1, GPIO.OUT)
            GPIO.setup(self.right_motor_pin2, GPIO.OUT)
            
            self.gpio_initialized = True
            rospy.loginfo("[%s] GPIO initialized for motor control" % self.node_name)
        except Exception as e:
            rospy.logerr("[%s] Failed to initialize GPIO: %s" % (self.node_name, str(e)))
            self.motor_type = "dummy"

    def cmd_vel_callback(self, msg):
        """
        Compute left/right wheel speeds from cmd_vel and send to motors.
        """
        v = msg.linear.x      # forward velocity (m/s)
        omega = msg.angular.z  # angular velocity (rad/s)

        # Simple differential drive inverse kinematics
        v_left = v - (omega * self.wheel_base / 2.0)
        v_right = v + (omega * self.wheel_base / 2.0)

        # Convert to wheel angular speed (rad/s)
        if self.wheel_radius > 0:
            w_left = v_left / self.wheel_radius
            w_right = v_right / self.wheel_radius
        else:
            w_left = v_left
            w_right = v_right

        # Convert to motor commands (normalized -1.0 to 1.0)
        # Assuming max speed of 1.0 m/s
        max_speed = 1.0
        left_cmd = max(-1.0, min(1.0, v_left / max_speed))
        right_cmd = max(-1.0, min(1.0, v_right / max_speed))

        # Send commands to hardware
        rospy.loginfo("[nano_motor_driver] 📥 Received cmd_vel: v=%.3f m/s, omega=%.3f rad/s" % (v, omega))
        rospy.loginfo("[nano_motor_driver] 📤 Sending to motors: Left=%.3f, Right=%.3f" % (left_cmd, right_cmd))
        
        self.send_motor_command(left_cmd, right_cmd)

    def send_motor_command(self, left_cmd, right_cmd):
        """Send motor commands to hardware."""
        
        if self.motor_type == "serial":
            self.send_serial_command(left_cmd, right_cmd)
        elif self.motor_type == "pca9685":
            self.send_pca9685_command(left_cmd, right_cmd)
        elif self.motor_type == "gpio":
            self.send_gpio_command(left_cmd, right_cmd)
        # else: dummy mode, do nothing

    def send_serial_command(self, left_cmd, right_cmd):
        """Send motor commands via serial.
        
        Supports two protocols:
        1. Binary (jetbot_pro): [head1 head2 size type data checksum]
        2. Text (arduino): "L:left_speed R:right_speed\n"
        """
        if self.serial_conn is None:
            rospy.logwarn("[nano_motor_driver] ⚠️  Serial connection is None! Cannot send command.")
            return
        
        if not self.serial_conn.is_open:
            rospy.logerr("[nano_motor_driver] ❌ Serial port is closed! Cannot send command.")
            return
        
        if self.serial_protocol == "binary":
            self._send_binary_command(left_cmd, right_cmd)
        else:
            self._send_text_command(left_cmd, right_cmd)
    
    def _send_binary_command(self, left_cmd, right_cmd):
        """Send motor commands using JetBot Pro binary protocol.
        
        Protocol format: [head1 head2 size type data checksum]
        - head1 = 0xAA
        - head2 = 0x55
        - type = 0x11 (velocity command)
        - data = left_speed (int16) + right_speed (int16) = 4 bytes
        - size = 1 (type) + 4 (data) + 1 (checksum) = 6 bytes
        - checksum = sum of all bytes except checksum itself
        """
        try:
            # Protocol constants (from jetbot.cpp)
            HEAD1 = 0xAA
            HEAD2 = 0x55
            SEND_TYPE_VELOCITY = 0x11
            
            # Convert normalized commands (-1.0 to 1.0) to motor speeds
            # Assuming max speed is 100 (adjust based on your motor controller)
            MAX_SPEED = 100
            left_speed = int(left_cmd * MAX_SPEED)
            right_speed = int(right_cmd * MAX_SPEED)
            
            # Clamp to valid range
            left_speed = max(-32767, min(32767, left_speed))
            right_speed = max(-32767, min(32767, right_speed))
            
            # Build binary packet
            # Size = 1 (type) + 4 (data: 2 bytes left + 2 bytes right) + 1 (checksum) = 6
            packet_size = 6
            
            # Convert speeds to int16 (little-endian)
            left_bytes = left_speed.to_bytes(2, byteorder='little', signed=True)
            right_bytes = right_speed.to_bytes(2, byteorder='little', signed=True)
            
            # Build packet: [HEAD1, HEAD2, SIZE, TYPE, LEFT_LOW, LEFT_HIGH, RIGHT_LOW, RIGHT_HIGH, CHECKSUM]
            packet = bytearray([
                HEAD1,
                HEAD2,
                packet_size,
                SEND_TYPE_VELOCITY,
                left_bytes[0],   # left speed low byte
                left_bytes[1],   # left speed high byte
                right_bytes[0],  # right speed low byte
                right_bytes[1],   # right speed high byte
                0x00  # checksum placeholder
            ])
            
            # Calculate checksum (sum of all bytes except checksum)
            checksum = sum(packet[:-1]) & 0xFF
            packet[-1] = checksum
            
            # Send packet
            self.serial_conn.write(packet)
            
            # Log the command (use INFO level so it shows up)
            rospy.loginfo("[nano_motor_driver] ✅ Sent binary cmd: L=%d R=%d | Packet: %s | Checksum: 0x%02X" % 
                          (left_speed, right_speed, 
                           ' '.join(['0x%02X' % b for b in packet]),
                           checksum))
            
        except Exception as e:
            rospy.logerr("[nano_motor_driver] Binary serial write error: %s" % str(e))
    
    def _send_text_command(self, left_cmd, right_cmd):
        """Send motor commands using text protocol (Arduino format)."""
        try:
            # Format: "L:left_speed R:right_speed\n"
            # Speed range: -255 to 255 (or -100 to 100)
            left_speed = int(left_cmd * 100)
            right_speed = int(right_cmd * 100)
            
            cmd = "L:%d R:%d\n" % (left_speed, right_speed)
            self.serial_conn.write(cmd.encode())
        except Exception as e:
            rospy.logerr("[nano_motor_driver] Text serial write error: %s" % str(e))

    def send_pca9685_command(self, left_cmd, right_cmd):
        """Send motor commands via PCA9685 PWM."""
        if self.pca9685 is None:
            return
        
        try:
            # Convert -1.0 to 1.0 to PWM values (0-4095)
            # PCA9685 channels: adjust based on your wiring
            left_channel = 0   # Adjust these channel numbers
            right_channel = 1  # Adjust these channel numbers
            
            # Convert command to PWM (0-4095, with 2048 as neutral)
            left_pwm = int(2048 + (left_cmd * 2048))
            right_pwm = int(2048 + (right_cmd * 2048))
            
            self.pca9685.set_pwm(left_channel, 0, left_pwm)
            self.pca9685.set_pwm(right_channel, 0, right_pwm)
        except Exception as e:
            rospy.logerr("[nano_motor_driver] PCA9685 error: %s" % str(e))

    def send_gpio_command(self, left_cmd, right_cmd):
        """Send motor commands via GPIO pins."""
        if not self.gpio_initialized:
            return
        
        try:
            # Simple GPIO control (adjust based on your motor driver circuit)
            # This is a basic example - you may need PWM for speed control
            if left_cmd > 0.1:
                GPIO.output(self.left_motor_pin1, GPIO.HIGH)
                GPIO.output(self.left_motor_pin2, GPIO.LOW)
            elif left_cmd < -0.1:
                GPIO.output(self.left_motor_pin1, GPIO.LOW)
                GPIO.output(self.left_motor_pin2, GPIO.HIGH)
            else:
                GPIO.output(self.left_motor_pin1, GPIO.LOW)
                GPIO.output(self.left_motor_pin2, GPIO.LOW)
            
            if right_cmd > 0.1:
                GPIO.output(self.right_motor_pin1, GPIO.HIGH)
                GPIO.output(self.right_motor_pin2, GPIO.LOW)
            elif right_cmd < -0.1:
                GPIO.output(self.right_motor_pin1, GPIO.LOW)
                GPIO.output(self.right_motor_pin2, GPIO.HIGH)
            else:
                GPIO.output(self.right_motor_pin1, GPIO.LOW)
                GPIO.output(self.right_motor_pin2, GPIO.LOW)
        except Exception as e:
            rospy.logerr("[nano_motor_driver] GPIO error: %s" % str(e))

    def shutdown(self):
        """Cleanup on shutdown."""
        if self.serial_conn:
            self.serial_conn.close()
        if self.gpio_initialized:
            GPIO.cleanup()


if __name__ == "__main__":
    try:
        driver = NanoMotorDriver()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'driver' in locals():
            driver.shutdown()



