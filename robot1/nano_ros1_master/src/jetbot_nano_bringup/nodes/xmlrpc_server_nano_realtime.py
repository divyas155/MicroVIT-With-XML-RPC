#!/usr/bin/env python3
"""
REALTIME VERSION - XML-RPC Server for Jetson Nano

Uses camera from /nano/camera/image_compressed (published by camera_publisher_node).
Camera is in a SEPARATE node - if it crashes, XML-RPC keeps running.
"""

import base64
import threading
import math
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, CompressedImage
from xmlrpc.server import SimpleXMLRPCRequestHandler, SimpleXMLRPCServer


class QuietHandler(SimpleXMLRPCRequestHandler):
    def log_message(self, format, *args):
        pass


class NanoXMLRPCServerRealtime:
    def __init__(self, host: str = "0.0.0.0", port: int = 8000):
        self.host = host
        self.port = port

        rospy.loginfo("[Nano XML-RPC] Node initializing...")
        rospy.init_node("nano_xmlrpc_server_realtime", anonymous=True)

        self.current_velocity = Twist()
        self.motors_active = False
        self.latest_scan = None
        self.latest_odom = None
        self.latest_camera_msg = None
        self._camera_lock = threading.Lock()

        # Subscribe to camera topic (from separate camera_publisher_node)
        self.camera_sub = rospy.Subscriber("/nano/camera/image_compressed", CompressedImage, self._camera_callback)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        rospy.sleep(1.0)

        rospy.loginfo("[Nano XML-RPC] Binding to %s:%s...", self.host, self.port)
        try:
            self.server = SimpleXMLRPCServer(
                (self.host, self.port), requestHandler=QuietHandler, allow_none=True
            )
        except OSError as e:
            if getattr(e, "errno", None) == 98 or "Address already in use" in str(e):
                rospy.logerr("[Nano XML-RPC] Port %s in use. Free: sudo fuser -k %s/tcp", self.port, self.port)
            raise
        self.server.register_function(self.set_motor_velocity, "set_motor_velocity")
        self.server.register_function(self.get_robot_status, "get_robot_status")
        self.server.register_function(self.get_lidar_data, "get_lidar_data")
        self.server.register_function(self.get_odometry_data, "get_odometry_data")
        self.server.register_function(self.get_camera_image, "get_camera_image")

        rospy.loginfo("[Nano XML-RPC Server REALTIME] Started on %s:%s", self.host, self.port)
        rospy.loginfo("[Nano XML-RPC Server REALTIME] Camera from /nano/camera/image_compressed (separate node)")

    def _camera_callback(self, msg: CompressedImage):
        with self._camera_lock:
            self.latest_camera_msg = msg

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def odom_callback(self, msg: Odometry):
        self.latest_odom = msg

    def set_motor_velocity(self, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
        try:
            twist = Twist()
            twist.linear.x = float(linear_x)
            twist.linear.y = float(linear_y)
            twist.linear.z = float(linear_z)
            twist.angular.x = float(angular_x)
            twist.angular.y = float(angular_y)
            twist.angular.z = float(angular_z)
            self.current_velocity = twist
            self.motors_active = True
            self.cmd_vel_pub.publish(twist)
            return {"success": True, "message": f"Velocity set"}
        except Exception as e:
            rospy.logerr("[Nano REALTIME] Error setting velocity: %s", e)
            return {"success": False, "message": str(e)}

    def get_robot_status(self):
        try:
            with self._camera_lock:
                cam_ok = self.latest_camera_msg is not None and len(self.latest_camera_msg.data) > 0
            return {
                "motors_active": self.motors_active,
                "current_linear_velocity": self.current_velocity.linear.x,
                "current_angular_velocity": self.current_velocity.angular.z,
                "status_message": "Robot operational (REALTIME MODE)",
                "camera_available": cam_ok,
                "camera_message": None if cam_ok else "Camera node not publishing yet (check /nano/camera/image_compressed)",
                "success": True,
            }
        except Exception as e:
            return {"motors_active": False, "current_linear_velocity": 0.0, "current_angular_velocity": 0.0,
                    "status_message": str(e), "success": False}

    def get_lidar_data(self):
        try:
            lidar_source = "UNKNOWN"
            scan_publishers = []
            try:
                code, msg, system_state = rospy.get_master().getSystemState()
                publishers = system_state[0] if system_state and len(system_state) > 0 else []
                for topic, nodes in publishers:
                    if topic == "/scan":
                        scan_publishers = list(nodes) if nodes else []
                        break
                lower = " ".join(scan_publishers).lower()
                lidar_source = "DUMMY" if "nano_lidar_dummy" in lower else ("RPLIDAR" if "rplidar" in lower else "FALLBACK")
            except Exception:
                pass

            if self.latest_scan is None:
                return {
                    "success": True, "message": "No LiDAR scan yet",
                    "ranges": [0.5] * 20, "min_range": 0.5, "max_range": 1.5, "num_readings": 20,
                    "angle_min": -1.57, "angle_max": 1.57, "angle_increment": 0.017,
                    "min_index": 0, "min_angle": 0.0, "min_angle_deg": 0.0,
                    "lidar_source": lidar_source, "scan_publishers": scan_publishers,
                }

            ranges = list(self.latest_scan.ranges) if self.latest_scan.ranges else []
            valid = [(i, float(r)) for i, r in enumerate(ranges) if r is not None and not (math.isnan(r) or math.isinf(r)) and 0.01 < r < 100.0]
            if valid:
                min_index, min_range = min(valid, key=lambda t: t[1])
                max_range = max(r for _, r in valid)
                angle_increment = float(getattr(self.latest_scan, 'angle_increment', 0.0))
                min_angle = float(self.latest_scan.angle_min) + min_index * angle_increment
            else:
                min_index, min_range, max_range = -1, 0.0, 0.0
                min_angle = 0.0

            return {
                "success": True, "message": "LiDAR data retrieved",
                "ranges": ranges[:20], "min_range": float(min_range), "max_range": float(max_range),
                "num_readings": len(ranges),
                "angle_min": float(self.latest_scan.angle_min), "angle_max": float(self.latest_scan.angle_max),
                "angle_increment": float(getattr(self.latest_scan, 'angle_increment', 0.0)),
                "min_index": int(min_index), "min_angle": float(min_angle),
                "min_angle_deg": float(min_angle * 180.0 / math.pi),
                "lidar_source": lidar_source, "scan_publishers": scan_publishers,
            }
        except Exception as e:
            return {"success": True, "message": f"LiDAR error: {e}", "ranges": [0.5] * 20,
                    "min_range": 0.5, "max_range": 1.5, "num_readings": 20, "angle_min": -1.57,
                    "angle_max": 1.57, "angle_increment": 0.017, "min_index": 0, "min_angle": 0.0,
                    "min_angle_deg": 0.0, "lidar_source": "FALLBACK", "scan_publishers": []}

    def get_odometry_data(self):
        try:
            odom_source = "UNKNOWN"
            odom_publishers = []
            try:
                code, msg, system_state = rospy.get_master().getSystemState()
                publishers = system_state[0] if system_state and len(system_state) > 0 else []
                for topic, nodes in publishers:
                    if topic == "/odom":
                        odom_publishers = list(nodes) if nodes else []
                        break
                lower = " ".join(odom_publishers).lower()
                has_motor = "jetbot_motor_driver" in lower or "motor_driver" in lower
                has_dummy = "nano_odom_dummy" in lower
                odom_source = "MOTOR_DRIVER_AND_DUMMY" if (has_motor and has_dummy) else ("MOTOR_DRIVER" if has_motor else ("DUMMY" if has_dummy else "UNKNOWN"))
            except Exception:
                pass

            if self.latest_odom is None:
                try:
                    self.latest_odom = rospy.wait_for_message("/odom", Odometry, timeout=0.35)
                except Exception:
                    pass

            if self.latest_odom is None:
                return {"success": False, "message": "No odometry yet", "position_x": 0.0, "position_y": 0.0,
                        "position_z": 0.0, "odom_source": odom_source, "odom_publishers": odom_publishers}

            pos = self.latest_odom.pose.pose.position
            orient = self.latest_odom.pose.pose.orientation
            vel = self.latest_odom.twist.twist
            return {
                "success": True, "message": "Odometry retrieved",
                "position_x": float(pos.x), "position_y": float(pos.y), "position_z": float(pos.z),
                "orientation_x": float(orient.x), "orientation_y": float(orient.y),
                "orientation_z": float(orient.z), "orientation_w": float(orient.w),
                "linear_velocity": float(vel.linear.x), "angular_velocity": float(vel.angular.z),
                "odom_source": odom_source, "odom_publishers": odom_publishers,
            }
        except Exception as e:
            return {"success": False, "message": str(e), "position_x": 0.0, "position_y": 0.0, "position_z": 0.0,
                    "odom_source": "UNKNOWN", "odom_publishers": []}

    def get_camera_image(self):
        """Return latest image from camera_publisher_node (no direct camera access)."""
        try:
            with self._camera_lock:
                msg = self.latest_camera_msg
            if msg is None or not msg.data:
                return {"success": False, "message": "No camera image yet (camera node starting?)", "image_data": ""}
            image_base64 = base64.b64encode(bytes(msg.data)).decode("utf-8")
            w = rospy.get_param("/camera_width", 320)
            h = rospy.get_param("/camera_height", 240)
            return {
                "success": True, "message": "Camera image from topic",
                "image_data": image_base64, "width": int(w), "height": int(h),
            }
        except Exception as e:
            rospy.logerr("[Nano REALTIME] get_camera_image error: %s", e)
            return {"success": False, "message": str(e), "image_data": ""}

    def run(self):
        def server_thread():
            try:
                self.server.serve_forever()
            except KeyboardInterrupt:
                self.server.shutdown()
        t = threading.Thread(target=server_thread, daemon=True)
        t.start()
        rospy.spin()


if __name__ == "__main__":
    try:
        server = NanoXMLRPCServerRealtime(host="0.0.0.0", port=8000)
        print("=" * 60)
        print("Nano XML-RPC Server (REALTIME - camera from topic)")
        print("=" * 60)
        server.run()
    except OSError as e:
        print("[Nano XML-RPC] FATAL:", e)
        if getattr(e, "errno", None) == 98:
            print("  Port 8000 in use. Run: sudo fuser -k 8000/tcp")
        raise
    except KeyboardInterrupt:
        print("\nStopping...")
