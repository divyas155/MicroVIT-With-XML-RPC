#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, Vector3


class NanoOdomDummy(object):
    """
    Odometry node that tracks robot movement based on /cmd_vel commands.
    
    Subscribes to /cmd_vel and integrates velocity to estimate position.
    This provides coordinates for AI messages even if jetbot_pro crashes.
    """

    def __init__(self):
        self.node_name = "nano_odom_dummy"
        rospy.init_node(self.node_name)

        self.pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        
        # Subscribe to /cmd_vel to track movement
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback, queue_size=10)

        self.rate_hz = rospy.get_param("~rate", 10.0)
        self.frame_id = rospy.get_param("~frame_id", "odom")
        self.base_frame_id = rospy.get_param("~base_frame_id", "base_link")

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Orientation (yaw angle)
        
        # Current velocity (from /cmd_vel)
        self.vx = 0.0  # Linear velocity in x direction
        self.vtheta = 0.0  # Angular velocity (yaw rate)

        rospy.loginfo("[%s] Publishing /odom at %.1f Hz, tracking /cmd_vel for position", self.node_name, self.rate_hz)

    def cmd_vel_callback(self, msg):
        """Store latest velocity command"""
        self.vx = msg.linear.x
        self.vtheta = msg.angular.z

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        last_time = rospy.Time.now()

        while not rospy.is_shutdown():
            current_time = rospy.Time.now()
            dt = (current_time - last_time).to_sec()
            last_time = current_time

            # Integrate velocity to update position
            # Simple differential drive model
            if abs(self.vtheta) > 0.001:  # Turning
                # Arc motion
                radius = self.vx / self.vtheta if abs(self.vtheta) > 0.001 else 0.0
                dtheta = self.vtheta * dt
                dx = radius * math.sin(dtheta) if abs(radius) > 0.001 else self.vx * dt
                dy = radius * (1 - math.cos(dtheta)) if abs(radius) > 0.001 else 0.0
            else:
                # Straight motion
                dx = self.vx * dt * math.cos(self.theta)
                dy = self.vx * dt * math.sin(self.theta)
                dtheta = 0.0

            # Update position
            self.x += dx
            self.y += dy
            self.theta += dtheta
            
            # Normalize angle to [-pi, pi]
            while self.theta > math.pi:
                self.theta -= 2 * math.pi
            while self.theta < -math.pi:
                self.theta += 2 * math.pi

            # Create odometry message
            odom = Odometry()
            odom.header.stamp = current_time
            odom.header.frame_id = self.frame_id
            odom.child_frame_id = self.base_frame_id

            # Position
            odom.pose.pose = Pose(
                position=Point(self.x, self.y, 0.0),
                orientation=Quaternion(0.0, 0.0, math.sin(self.theta/2), math.cos(self.theta/2))
            )

            # Velocity
            odom.twist.twist = Twist(
                linear=Vector3(self.vx, 0.0, 0.0),
                angular=Vector3(0.0, 0.0, self.vtheta)
            )

            self.pub.publish(odom)
            rate.sleep()


if __name__ == "__main__":
    try:
        node = NanoOdomDummy()
        node.run()
    except rospy.ROSInterruptException:
        pass



