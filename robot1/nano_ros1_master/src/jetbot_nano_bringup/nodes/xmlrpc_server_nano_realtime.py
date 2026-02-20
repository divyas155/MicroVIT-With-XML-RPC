#!/usr/bin/env python3
"""
REALTIME VERSION - XML-RPC Server for Jetson Nano

This server provides motor control and REAL sensor data via direct XML-RPC:
- set_motor_velocity(...)  -> publishes ROS1 /cmd_vel
- get_lidar_data()         -> reads REAL ROS1 /scan (from real LiDAR driver)
- get_odometry_data()      -> reads REAL ROS1 /odom (from real encoder/IMU)
- get_camera_image()       -> captures from Nano USB camera (/dev/video0) and returns base64 JPEG

USAGE:
  1. Copy this file to Nano: ~/robot1_ros_stack/nano_ros1_ws/src/jetbot_nano_bringup/nodes/
  2. Start real LiDAR driver (e.g., rplidar_ros) publishing to /scan
  3. Start real odometry source publishing to /odom
  4. Run this server instead of the dummy version:
     python3 xmlrpc_server_nano_realtime.py
"""

import base64
import threading
import math
import time

import cv2
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from xmlrpc.server import SimpleXMLRPCRequestHandler, SimpleXMLRPCServer


class QuietHandler(SimpleXMLRPCRequestHandler):
    """Suppress server log messages."""

    def log_message(self, format, *args):
        pass


class NanoXMLRPCServerRealtime:
    """
    XML-RPC server that provides motor control + REALTIME sensor data.
    Includes camera capture from Nano's USB camera.
    """

    def __init__(self, host: str = "0.0.0.0", port: int = 8000):
        self.host = host
        self.port = port

        rospy.loginfo("[Nano XML-RPC] Node initializing...")
        # Initialize ROS node
        rospy.init_node("nano_xmlrpc_server_realtime", anonymous=True)

        # Current state
        self.current_velocity = Twist()
        self.motors_active = False
        self.latest_scan = None
        self.latest_odom = None

        # Camera setup (on Nano). Default /dev/video1 to avoid VIDEOIO errors on video0 (many Jetsons use video1 for USB cam).
        self.camera = None
        self.camera_device = rospy.get_param("~camera_device", "/dev/video1")
        self._last_camera_failure = ""  # reason when init or read fails
        self._init_camera()
        rospy.loginfo("[Nano XML-RPC] Camera init done (camera=%s)", "ok" if (self.camera is not None and self.camera.isOpened()) else "not available")

        # Subscribe to REAL sensor topics
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # Publisher for motor commands (to motor driver)
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Wait for publisher to register with ROS master
        rospy.sleep(1.0)

        # Setup XML-RPC server (bind to port; serve_forever runs later in run())
        rospy.loginfo("[Nano XML-RPC] Binding to %s:%s...", self.host, self.port)
        try:
            self.server = SimpleXMLRPCServer(
                (self.host, self.port), requestHandler=QuietHandler, allow_none=True
            )
        except OSError as e:
            if getattr(e, "errno", None) == 98 or "Address already in use" in str(e):
                rospy.logerr("[Nano XML-RPC] Port %s is already in use. Free it with: sudo fuser -k %s/tcp", self.port, self.port)
            raise
        self.server.register_function(self.set_motor_velocity, "set_motor_velocity")
        self.server.register_function(self.get_robot_status, "get_robot_status")
        self.server.register_function(self.get_lidar_data, "get_lidar_data")
        self.server.register_function(self.get_odometry_data, "get_odometry_data")
        self.server.register_function(self.get_camera_image, "get_camera_image")

        rospy.loginfo("[Nano XML-RPC Server REALTIME] Started on %s:%s", self.host, self.port)
        rospy.loginfo("[Nano XML-RPC Server REALTIME] Using REAL sensors + camera")
        rospy.loginfo("[Nano XML-RPC Server REALTIME] Waiting for commands from Orin...")

    def _init_camera(self):
        """Initialize camera on Nano. Tries V4L2 and default backend, indices 0-5, and a test read."""
        self._last_camera_failure = ""
        try:
            capture_source = self.camera_device
            if isinstance(capture_source, str):
                s = capture_source.strip()
                if s.isdigit():
                    capture_source = int(s)
                elif s.startswith("/dev/video"):
                    try:
                        capture_source = int(s.replace("/dev/video", ""))
                    except Exception:
                        capture_source = s

            last_err = [None]  # use list so inner function can set it

            def _configure(cap):
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

            def _try_open(src, backend=None):
                cap = None
                try:
                    cap = cv2.VideoCapture(src, backend) if backend is not None else cv2.VideoCapture(src)
                    if not cap.isOpened():
                        last_err[0] = "isOpened()=False"
                        return None
                    _configure(cap)
                    # Verify we can actually read a frame (some devices open but don't deliver)
                    ret, _ = cap.read()
                    if not ret:
                        last_err[0] = "read() failed (no frame)"
                        cap.release()
                        return None
                    return cap
                except Exception as e:
                    last_err[0] = str(e)
                    if cap is not None:
                        try:
                            cap.release()
                        except Exception:
                            pass
                    return None

            cap = None
            chosen = None
            # When requested device is video0, try video1 first (many Jetsons use video1 for USB camera; avoids VIDEOIO errors on video0)
            if capture_source == 0:
                cap = _try_open(1, cv2.CAP_V4L2)
                if cap is None:
                    cap = _try_open(1, None)
                if cap is not None:
                    chosen = 1
            # Try requested source with V4L2 then default
            if cap is None:
                cap = _try_open(capture_source, cv2.CAP_V4L2)
            if cap is None:
                cap = _try_open(capture_source, None)
            # Auto-scan: try 1 before 0 to avoid opening video0 when camera is on video1
            if cap is None:
                for idx in [1, 0, 2, 3, 4, 5]:
                    cap = _try_open(idx, cv2.CAP_V4L2)
                    if cap is None:
                        cap = _try_open(idx, None)
                    if cap is not None:
                        chosen = idx
                        break

            if cap is not None:
                self.camera = cap
                if chosen is not None:
                    self.camera_device = f"/dev/video{chosen}"
                    capture_source = chosen
                self._last_camera_failure = ""
                rospy.loginfo(f"[Nano REALTIME] ✅ Camera initialized on {self.camera_device} (source={capture_source}, MJPEG 640x480)")
            else:
                self._last_camera_failure = last_err[0] or "open or read failed"
                rospy.logwarn(
                    "[Nano REALTIME] ⚠️ Camera %s not found/open. Last error: %s. "
                    "Check: ls -la /dev/video*; sudo chmod 666 /dev/video0; ensure no other app uses camera.",
                    self.camera_device, self._last_camera_failure
                )
                self.camera = None
        except Exception as e:
            self._last_camera_failure = str(e)
            rospy.logerr(f"[Nano REALTIME] ❌ Camera initialization failed: {e}")
            self.camera = None

    def scan_callback(self, msg: LaserScan):
        """Store latest LiDAR scan (from REAL LiDAR)."""
        self.latest_scan = msg

    def odom_callback(self, msg: Odometry):
        """Store latest odometry (from REAL odometry source)."""
        self.latest_odom = msg

    def set_motor_velocity(self, linear_x, linear_y, linear_z, angular_x, angular_y, angular_z):
        """Set motor velocity."""
        try:
            print(f"\n[REALTIME] Received velocity: linear=({linear_x}, {linear_y}, {linear_z}), angular=({angular_x}, {angular_y}, {angular_z})")

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

            return {
                "success": True,
                "message": f"Velocity set (REALTIME): linear=({linear_x:.3f}, {linear_y:.3f}, {linear_z:.3f})",
            }

        except Exception as e:
            rospy.logerr(f"[Nano REALTIME] Error setting velocity: {e}")
            return {"success": False, "message": f"Error: {str(e)}"}

    def get_robot_status(self):
        """Get robot status."""
        try:
            cam_ok = self.camera is not None and self.camera.isOpened()
            return {
                "motors_active": self.motors_active,
                "current_linear_velocity": self.current_velocity.linear.x,
                "current_angular_velocity": self.current_velocity.angular.z,
                "status_message": "Robot operational (REALTIME MODE)",
                "camera_available": cam_ok,
                "camera_message": None if cam_ok else (self._last_camera_failure or "Camera not available on Nano"),
                "success": True,
            }
        except Exception as e:
            return {
                "motors_active": False,
                "current_linear_velocity": 0.0,
                "current_angular_velocity": 0.0,
                "status_message": f"Error: {str(e)}",
                "success": False,
            }

    def get_lidar_data(self):
        """Get latest REAL LiDAR scan data."""
        try:
            # Detect which node is publishing /scan (dummy vs real) via ROS master.
            lidar_source = "UNKNOWN"
            scan_publishers = []
            try:
                code, msg, system_state = rospy.get_master().getSystemState()
                # system_state[0] = publishers: [[topic, [nodes...]], ...]
                publishers = system_state[0] if system_state and len(system_state) > 0 else []
                for topic, nodes in publishers:
                    if topic == "/scan":
                        scan_publishers = list(nodes) if nodes else []
                        break
                lower = " ".join(scan_publishers).lower()
                if "nano_lidar_dummy" in lower:
                    lidar_source = "DUMMY"
                elif "rplidar" in lower:
                    lidar_source = "RPLIDAR"
            except Exception:
                pass

            # When no /scan data (rplidar down, timeouts, or use_lidar:=false with no dummy): return fallback so pipeline keeps running.
            if self.latest_scan is None:
                fallback_source = "FALLBACK" if lidar_source == "RPLIDAR" else lidar_source
                if fallback_source == "UNKNOWN":
                    fallback_source = "FALLBACK"
                return {
                    "success": True,
                    "message": "No LiDAR scan yet; using fallback (real LiDAR may be timing out or disconnected).",
                    "ranges": [0.5] * 20,
                    "min_range": 0.5,
                    "max_range": 1.5,
                    "num_readings": 20,
                    "angle_min": -1.57,
                    "angle_max": 1.57,
                    "angle_increment": 0.017,
                    "min_index": 0,
                    "min_angle": 0.0,
                    "min_angle_deg": 0.0,
                    "lidar_source": fallback_source,
                    "scan_publishers": scan_publishers,
                }

            ranges = list(self.latest_scan.ranges) if self.latest_scan.ranges else []
            # Filter out inf/nan and out-of-range values
            valid = []
            for i, r in enumerate(ranges):
                try:
                    if r is None:
                        continue
                    if math.isnan(r) or math.isinf(r):
                        continue
                    if r <= 0.01 or r >= 100.0:
                        continue
                    valid.append((i, float(r)))
                except Exception:
                    continue

            if valid:
                min_index, min_range = min(valid, key=lambda t: t[1])
                max_range = max(r for _, r in valid)
                angle_min = float(self.latest_scan.angle_min)
                angle_increment = float(getattr(self.latest_scan, 'angle_increment', 0.0))
                min_angle = angle_min + float(min_index) * angle_increment
            else:
                min_index, min_range, max_range = -1, 0.0, 0.0
                angle_min = float(self.latest_scan.angle_min)
                angle_increment = float(getattr(self.latest_scan, 'angle_increment', 0.0))
                min_angle = 0.0

            return {
                "success": True,
                "message": "LiDAR data retrieved successfully",
                "ranges": ranges[:20],  # First 20 ranges
                "min_range": float(min_range),
                "max_range": float(max_range),
                "num_readings": len(ranges),
                "angle_min": float(self.latest_scan.angle_min),
                "angle_max": float(self.latest_scan.angle_max),
                "angle_increment": float(angle_increment),
                # Added fields so Orin can compute obstacle direction even though we only return first 20 ranges
                "min_index": int(min_index),
                "min_angle": float(min_angle),
                "min_angle_deg": float(min_angle * 180.0 / math.pi) if valid else 0.0,
                # Added fields so Orin can label REAL vs SIMULATED properly
                "lidar_source": lidar_source,
                "scan_publishers": scan_publishers,
            }
        except Exception as e:
            # Return fallback so Orin keeps running; do not leave pipeline with no LiDAR.
            return {
                "success": True,
                "message": f"LiDAR error ({str(e)}); using fallback.",
                "ranges": [0.5] * 20,
                "min_range": 0.5,
                "max_range": 1.5,
                "num_readings": 20,
                "angle_min": -1.57,
                "angle_max": 1.57,
                "angle_increment": 0.017,
                "min_index": 0,
                "min_angle": 0.0,
                "min_angle_deg": 0.0,
                "lidar_source": "FALLBACK",
                "scan_publishers": [],
            }

    def get_odometry_data(self):
        """Get latest REAL odometry data."""
        try:
            # Detect which node is publishing /odom (dummy vs motor driver) via ROS master.
            # Note: Multiple publishers is possible (and problematic). We report that explicitly.
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
                has_dummy = "nano_odom_dummy" in lower
                has_motor = ("jetbot_motor_driver" in lower) or ("motor_driver" in lower)
                if has_motor and has_dummy:
                    odom_source = "MOTOR_DRIVER_AND_DUMMY"
                elif has_motor:
                    odom_source = "MOTOR_DRIVER"
                elif has_dummy:
                    odom_source = "DUMMY"
            except Exception:
                pass

            # If we haven't received an Odometry message yet, wait briefly once.
            # This avoids "failing" at startup when Orin polls before the first /odom arrives.
            if self.latest_odom is None:
                try:
                    self.latest_odom = rospy.wait_for_message("/odom", Odometry, timeout=0.35)
                except Exception:
                    pass

            if self.latest_odom is None:
                return {
                    "success": False,
                    "message": "No odometry data available yet (check real odom source)",
                    "position_x": 0.0,
                    "position_y": 0.0,
                    "position_z": 0.0,
                    "odom_source": odom_source,
                    "odom_publishers": odom_publishers,
                }

            pos = self.latest_odom.pose.pose.position
            orient = self.latest_odom.pose.pose.orientation
            vel = self.latest_odom.twist.twist

            # Helpful diagnostics: age of last odom message (seconds)
            odom_age_sec = None
            try:
                stamp = getattr(self.latest_odom, "header", None)
                stamp = getattr(stamp, "stamp", None)
                if stamp is not None:
                    odom_age_sec = float(rospy.Time.now().to_sec() - stamp.to_sec())
            except Exception:
                odom_age_sec = None

            return {
                "success": True,
                "message": "REAL odometry data retrieved successfully",
                "position_x": float(pos.x),
                "position_y": float(pos.y),
                "position_z": float(pos.z),
                "orientation_x": float(orient.x),
                "orientation_y": float(orient.y),
                "orientation_z": float(orient.z),
                "orientation_w": float(orient.w),
                "linear_velocity": float(vel.linear.x),
                "angular_velocity": float(vel.angular.z),
                "odom_source": odom_source,
                "odom_publishers": odom_publishers,
                "odom_age_sec": odom_age_sec,
            }
        except Exception as e:
            return {
                "success": False,
                "message": f"Error: {str(e)}",
                "position_x": 0.0,
                "position_y": 0.0,
                "position_z": 0.0,
                "odom_source": "UNKNOWN",
                "odom_publishers": [],
            }

    def get_camera_image(self):
        """Get REAL camera image from Nano's USB camera."""
        try:
            # If camera disappeared (USB reset), try to re-init before failing.
            if self.camera is None or not self.camera.isOpened():
                rospy.logwarn("[Nano REALTIME] Camera not opened; attempting re-initialize...")
                try:
                    self._init_camera()
                except Exception:
                    pass
                if self.camera is None or not self.camera.isOpened():
                    msg = self._last_camera_failure or "Camera not available on Nano"
                    return {
                        "success": False,
                        "message": msg,
                        "image_data": "",
                    }

            # Warm-up / retry reads (some UVC cams fail the first few reads)
            ret, frame = False, None
            for _ in range(5):
                ret, frame = self.camera.read()
                if ret and frame is not None:
                    break
                time.sleep(0.05)
            if not ret:
                # If reads fail, try a full camera reset once (common after USB resets).
                rospy.logwarn("[Nano REALTIME] Camera read failed; resetting camera and retrying...")
                try:
                    self.camera.release()
                except Exception:
                    pass
                self.camera = None
                try:
                    self._init_camera()
                except Exception:
                    pass

                # Retry after re-init
                if self.camera is not None and self.camera.isOpened():
                    for _ in range(5):
                        ret, frame = self.camera.read()
                        if ret and frame is not None:
                            break
                        time.sleep(0.05)

                if not ret:
                    self._last_camera_failure = "Failed to capture frame from camera (read failed after retry)"
                    return {
                        "success": False,
                        "message": self._last_camera_failure,
                        "image_data": "",
                    }

            # Encode to JPEG and then base64
            _, buffer = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            image_bytes = buffer.tobytes()
            image_base64 = base64.b64encode(image_bytes).decode("utf-8")

            rospy.loginfo("[Nano REALTIME] Served camera image %dx%d to Orin (%d bytes base64)", frame.shape[1], frame.shape[0], len(image_base64))
            return {
                "success": True,
                "message": "REAL camera image captured successfully",
                "image_data": image_base64,
                "width": frame.shape[1],
                "height": frame.shape[0],
            }

        except Exception as e:
            rospy.logerr(f"[Nano REALTIME] Error capturing camera image: {e}")
            return {
                "success": False,
                "message": f"Error: {str(e)}",
                "image_data": "",
            }

    def run(self):
        """Run the XML-RPC server in a separate thread."""
        rospy.loginfo("[Nano XML-RPC] Starting server thread (listening for Orin requests)...")

        def server_thread():
            try:
                self.server.serve_forever()
            except KeyboardInterrupt:
                print("\n[Nano REALTIME] Stopping server...")
                self.server.shutdown()

        thread = threading.Thread(target=server_thread, daemon=True)
        thread.start()

        rospy.spin()

        # Cleanup
        if self.camera is not None:
            self.camera.release()


if __name__ == "__main__":
    try:
        server = NanoXMLRPCServerRealtime(host="0.0.0.0", port=8000)
        print("=" * 60)
        print("Nano XML-RPC Server Started (REALTIME MODE)")
        print("Using REAL LiDAR, REAL odometry, REAL camera")
        print("=" * 60)
        server.run()
    except OSError as e:
        print("[Nano XML-RPC] FATAL: Could not bind to 0.0.0.0:8000:", e)
        if getattr(e, "errno", None) == 98:
            print("  Port 8000 in use. On Nano run:  sudo fuser -k 8000/tcp")
        raise
    except KeyboardInterrupt:
        print("\nStopping server...")
