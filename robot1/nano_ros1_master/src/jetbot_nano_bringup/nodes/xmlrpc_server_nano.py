#!/usr/bin/env python3
"""
Direct XML-RPC Server for Jetson Nano (DUMMY/SIMULATION VERSION)

This server provides motor control and sensor data via direct XML-RPC,
following the same pattern as the user's example.

This is the ORIGINAL DUMMY version that:
- Reads /scan from dummy LiDAR node
- Reads /odom from dummy odometry node
- Does NOT include camera (use test images on Orin)

For REALTIME version with camera, use:
  robot1_realtime_setup/nano/xmlrpc_server_nano_realtime.py
"""

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from xmlrpc.server import SimpleXMLRPCServer, SimpleXMLRPCRequestHandler
import threading
import time


class QuietHandler(SimpleXMLRPCRequestHandler):
    """Suppress server log messages."""
    def log_message(self, format, *args):
        pass


class NanoXMLRPCServer:
    """
    XML-RPC server that provides motor control and sensor data.
    Similar to the user's example pattern.
    
    DUMMY VERSION: Uses simulated LiDAR/odom from dummy nodes.
    """
    
    def __init__(self, host='0.0.0.0', port=8000):
        self.host = host
        self.port = port
        
        # Initialize ROS node
        rospy.init_node('nano_xmlrpc_server', anonymous=True)
        
        # Current state
        self.current_velocity = Twist()
        self.motors_active = False
        self.latest_scan = None
        self.latest_odom = None
        
        # Subscribe to sensor topics (from dummy nodes)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        
        # Publisher for motor commands (to motor driver)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        
        # Wait for publisher to register with ROS master
        rospy.sleep(1.0)
        
        # Setup XML-RPC server
        self.server = SimpleXMLRPCServer(
            (self.host, self.port),
            requestHandler=QuietHandler,
            allow_none=True
        )
        
        # Register functions (like the user's example)
        self.server.register_function(self.set_motor_velocity, 'set_motor_velocity')
        self.server.register_function(self.get_robot_status, 'get_robot_status')
        self.server.register_function(self.get_lidar_data, 'get_lidar_data')
        self.server.register_function(self.get_odometry_data, 'get_odometry_data')
        
        rospy.loginfo(f"[Nano XML-RPC Server] Started on {self.host}:{self.port}")
        rospy.loginfo("[Nano XML-RPC Server] DUMMY MODE - Using simulated sensors")
        rospy.loginfo("[Nano XML-RPC Server] Waiting for commands from Orin...")
    
    def scan_callback(self, msg: LaserScan):
        """Store latest LiDAR scan."""
        self.latest_scan = msg
    
    def odom_callback(self, msg: Odometry):
        """Store latest odometry."""
        self.latest_odom = msg
    
    def set_motor_velocity(self, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
        """
        Set motor velocity (like the user's remote_process function).
        
        Args:
            linear_x, linear_y, linear_z: Linear velocity components
            angular_x, angular_y, angular_z: Angular velocity components
            
        Returns:
            Dictionary with success status and message
        """
        try:
            print(f"\n[Received] Orin sent velocity command: linear=({linear_x}, {linear_y}, {linear_z}), angular=({angular_x}, {angular_y}, {angular_z})")
            print("[Action] Setting motor velocity...")
            
            # Create Twist message
            twist = Twist()
            twist.linear.x = float(linear_x)
            twist.linear.y = float(linear_y)
            twist.linear.z = float(linear_z)
            twist.angular.x = float(angular_x)
            twist.angular.y = float(angular_y)
            twist.angular.z = float(angular_z)
            
            # Store current velocity
            self.current_velocity = twist
            self.motors_active = True
            
            # Publish to motor driver
            self.cmd_vel_pub.publish(twist)
            
            print("[Success] Velocity command executed.")
            print("-------------------------------------------------------------")
            print("Waiting for next command from Orin...")
            
            return {
                "success": True,
                "message": f"Velocity set: linear=({linear_x:.3f}, {linear_y:.3f}, {linear_z:.3f}), angular=({angular_x:.3f}, {angular_y:.3f}, {angular_z:.3f})"
            }
            
        except Exception as e:
            rospy.logerr(f"[Nano XML-RPC] Error setting velocity: {e}")
            return {
                "success": False,
                "message": f"Error: {str(e)}"
            }
    
    def get_robot_status(self):
        """
        Get robot status.
        
        Returns:
            Dictionary with robot status
        """
        try:
            return {
                "motors_active": self.motors_active,
                "current_linear_velocity": self.current_velocity.linear.x,
                "current_angular_velocity": self.current_velocity.angular.z,
                "status_message": "Robot operational (DUMMY MODE)",
                "success": True
            }
        except Exception as e:
            return {
                "motors_active": False,
                "current_linear_velocity": 0.0,
                "current_angular_velocity": 0.0,
                "status_message": f"Error: {str(e)}",
                "success": False
            }
    
    def get_lidar_data(self):
        """
        Get latest LiDAR scan data (from dummy node).
        
        Returns:
            Dictionary with LiDAR data (simplified)
        """
        try:
            if self.latest_scan is None:
                return {
                    "success": False,
                    "message": "No LiDAR data available yet",
                    "ranges": [],
                    "min_range": 0.0,
                    "max_range": 0.0
                }
            
            # Extract key data from scan
            ranges = list(self.latest_scan.ranges) if self.latest_scan.ranges else []
            min_range = min(ranges) if ranges else 0.0
            max_range = max(ranges) if ranges else 0.0
            
            return {
                "success": True,
                "message": "LiDAR data retrieved successfully (DUMMY)",
                "ranges": ranges[:10],  # First 10 ranges (to avoid huge data)
                "min_range": float(min_range),
                "max_range": float(max_range),
                "num_readings": len(ranges)
            }
        except Exception as e:
            return {
                "success": False,
                "message": f"Error: {str(e)}",
                "ranges": [],
                "min_range": 0.0,
                "max_range": 0.0
            }
    
    def get_odometry_data(self):
        """
        Get latest odometry data (from dummy node).
        
        Returns:
            Dictionary with odometry data (simplified)
        """
        try:
            if self.latest_odom is None:
                return {
                    "success": False,
                    "message": "No odometry data available yet",
                    "position_x": 0.0,
                    "position_y": 0.0,
                    "position_z": 0.0
                }
            
            # Extract position from odometry
            pos = self.latest_odom.pose.pose.position
            
            return {
                "success": True,
                "message": "Odometry data retrieved successfully (DUMMY)",
                "position_x": float(pos.x),
                "position_y": float(pos.y),
                "position_z": float(pos.z)
            }
        except Exception as e:
            return {
                "success": False,
                "message": f"Error: {str(e)}",
                "position_x": 0.0,
                "position_y": 0.0,
                "position_z": 0.0
            }
    
    def run(self):
        """Run the XML-RPC server in a separate thread."""
        def server_thread():
            try:
                self.server.serve_forever()
            except KeyboardInterrupt:
                print("\n[Nano XML-RPC Server] Stopping server...")
                self.server.shutdown()
        
        # Start server in background thread
        thread = threading.Thread(target=server_thread, daemon=True)
        thread.start()
        
        # Keep ROS node running
        rospy.spin()


if __name__ == "__main__":
    try:
        # Create and run server
        server = NanoXMLRPCServer(host='0.0.0.0', port=8000)
        print("-------------------------------------------------------------")
        print("Nano XML-RPC Server Started (DUMMY MODE)")
        print("Using simulated LiDAR and odometry from dummy nodes")
        print("For REALTIME mode, use: robot1_realtime_setup/nano/")
        print("Waiting for command from Orin...")
        print("-------------------------------------------------------------")
        server.run()
    except KeyboardInterrupt:
        print("\nStopping server...")
