#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
from sensor_msgs.msg import LaserScan


class NanoLidarDummy(object):
    """
    Dummy LiDAR node for Jetson Nano.

    Publishes a synthetic /scan at ~5 Hz with a fixed pattern.
    """

    def __init__(self):
        self.node_name = "nano_lidar_dummy"
        rospy.init_node(self.node_name)

        self.pub = rospy.Publisher("/scan", LaserScan, queue_size=10)

        self.rate_hz = rospy.get_param("~rate", 5.0)
        self.frame_id = rospy.get_param("~frame_id", "laser")

        # Define scan parameters: -90° to +90° in 1° increments
        self.angle_min = math.radians(-90.0)
        self.angle_max = math.radians(90.0)
        self.angle_increment = math.radians(1.0)
        self.range_min = 0.1
        self.range_max = 10.0

        rospy.loginfo("[%s] Publishing dummy /scan at %.1f Hz", self.node_name, self.rate_hz)

    def run(self):
        rate = rospy.Rate(self.rate_hz)
        step = 0
        while not rospy.is_shutdown():
            msg = LaserScan()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.frame_id

            msg.angle_min = self.angle_min
            msg.angle_max = self.angle_max
            msg.angle_increment = self.angle_increment
            msg.time_increment = 0.0
            msg.scan_time = 1.0 / self.rate_hz
            msg.range_min = self.range_min
            msg.range_max = self.range_max

            # Generate a simple synthetic pattern: mostly 1.0 m, with a \"bump\"
            num_readings = int((self.angle_max - self.angle_min) / self.angle_increment) + 1
            ranges = []
            for i in range(num_readings):
                angle = self.angle_min + i * self.angle_increment
                # Create a fake obstacle at ~+30 degrees that moves slightly over time
                center = math.radians(30.0) + 0.1 * math.sin(step * 0.1)
                if abs(angle - center) < math.radians(5.0):
                    r = 0.5  # closer obstacle
                else:
                    r = 1.5  # free space
                ranges.append(r)

            msg.ranges = ranges

            self.pub.publish(msg)
            step += 1
            rate.sleep()


if __name__ == "__main__":
    try:
        node = NanoLidarDummy()
        node.run()
    except rospy.ROSInterruptException:
        pass



