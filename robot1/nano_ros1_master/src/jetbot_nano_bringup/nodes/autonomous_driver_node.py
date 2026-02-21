#!/usr/bin/env python3
"""
Autonomous driver: move forward when clear, stop when obstacle < threshold.
Subscribes to /scan, publishes to /cmd_vel.
Only considers obstacles in FRONT sector — side/rear obstacles are ignored so robot resumes when path ahead is clear.
"""

import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


def main():
    rospy.init_node("autonomous_driver", anonymous=False)
    obstacle_threshold = rospy.get_param("~obstacle_threshold", 0.5)
    forward_speed = rospy.get_param("~forward_speed", 0.2)
    rate_hz = rospy.get_param("~rate", 10.0)
    # Front sector: only obstacles within ±half_sector of forward direction. Side/rear ignored.
    forward_sector_deg = float(rospy.get_param("~forward_sector_deg", 90.0))
    # Angle (deg) that points forward. 0=standard. 180 if LiDAR cable/0° is at robot's back.
    forward_angle_offset_deg = float(rospy.get_param("~forward_angle_offset_deg", 0.0))

    cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rate = rospy.Rate(rate_hz)

    latest_scan = [None]
    was_stopped = [False]

    def scan_cb(msg):
        latest_scan[0] = msg

    rospy.Subscriber("/scan", LaserScan, scan_cb)
    half_sector_rad = math.radians(forward_sector_deg / 2.0)
    forward_rad = math.radians(forward_angle_offset_deg)
    rospy.loginfo(
        "[Autonomous Driver] Move %.2f m/s, stop when obstacle < %.2f m in front (±%.0f°, offset=%.0f°)",
        forward_speed, obstacle_threshold, forward_sector_deg / 2.0, forward_angle_offset_deg
    )

    while not rospy.is_shutdown():
        twist = Twist()
        msg = latest_scan[0]
        if msg is None or not msg.ranges:
            twist.linear.x = 0.0
        else:
            # Only consider front sector: angles within ±half_sector of forward_rad
            valid = []
            for i, r in enumerate(msg.ranges):
                if r is None or math.isnan(r) or math.isinf(r) or not (0.01 < r < 100.0):
                    continue
                angle = msg.angle_min + i * msg.angle_increment
                # Normalize to [-pi, pi]
                while angle > math.pi:
                    angle -= 2 * math.pi
                while angle < -math.pi:
                    angle += 2 * math.pi
                # Angular distance from forward direction (handles wraparound)
                diff = abs(angle - forward_rad)
                if diff > math.pi:
                    diff = 2 * math.pi - diff
                if diff <= half_sector_rad:
                    valid.append(float(r))
            min_dist = min(valid) if valid else 999.0
            if min_dist < obstacle_threshold:
                twist.linear.x = 0.0
                was_stopped[0] = True
                rospy.loginfo_throttle(2, "[Autonomous Driver] Obstacle at %.2f m (front) — STOPPED", min_dist)
            else:
                twist.linear.x = forward_speed
                if was_stopped[0]:
                    rospy.loginfo("[Autonomous Driver] Path clear (min %.2f m) — RESUMING", min_dist)
                    was_stopped[0] = False
                rospy.loginfo_throttle(5, "[Autonomous Driver] Moving (min front %.2f m)", min_dist)
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        cmd_pub.publish(twist)
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
