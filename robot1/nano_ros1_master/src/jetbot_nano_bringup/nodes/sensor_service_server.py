#!/usr/bin/env python3
"""
Sensor Service Server for Jetson Nano (ROS1)

Provides XML-RPC services for sensor data:
- /nano/get_lidar_data: Get latest LiDAR scan
- /nano/get_odometry_data: Get latest odometry data

This allows pure XML-RPC communication without topics.
"""

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from jetbot_nano_bringup.srv import GetLidarData, GetLidarDataResponse
from jetbot_nano_bringup.srv import GetOdometryData, GetOdometryDataResponse


class SensorServiceServer(object):
    """
    Service server that provides sensor data via XML-RPC.
    """

    def __init__(self):
        self.node_name = "nano_sensor_service_server"
        rospy.init_node(self.node_name)

        # Latest sensor data (subscribed from topic publishers)
        self.latest_scan = None
        self.latest_odom = None

        # Subscribe to sensor topics (from other nodes)
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_callback)

        # Advertise services
        self.lidar_service = rospy.Service(
            '/nano/get_lidar_data',
            GetLidarData,
            self.handle_get_lidar
        )

        self.odom_service = rospy.Service(
            '/nano/get_odometry_data',
            GetOdometryData,
            self.handle_get_odom
        )

        rospy.loginfo("[%s] Started. Services available:" % self.node_name)
        rospy.loginfo("[%s]   - /nano/get_lidar_data" % self.node_name)
        rospy.loginfo("[%s]   - /nano/get_odometry_data" % self.node_name)

    def scan_callback(self, msg: LaserScan):
        """Store latest LiDAR scan."""
        self.latest_scan = msg

    def odom_callback(self, msg: Odometry):
        """Store latest odometry."""
        self.latest_odom = msg

    def handle_get_lidar(self, req):
        """
        Handle GetLidarData service request.
        
        Args:
            req: GetLidarData request
            
        Returns:
            GetLidarDataResponse with latest scan data
        """
        try:
            if self.latest_scan is None:
                return GetLidarDataResponse(
                    scan_data=LaserScan(),
                    success=False,
                    message="No LiDAR data available yet"
                )
            
            return GetLidarDataResponse(
                scan_data=self.latest_scan,
                success=True,
                message="LiDAR data retrieved successfully"
            )
        except Exception as e:
            rospy.logerr("[nano_sensor_service] Error getting LiDAR data: %s", str(e))
            return GetLidarDataResponse(
                scan_data=LaserScan(),
                success=False,
                message=f"Error: {str(e)}"
            )

    def handle_get_odom(self, req):
        """
        Handle GetOdometryData service request.
        
        Args:
            req: GetOdometryData request
            
        Returns:
            GetOdometryDataResponse with latest odometry data
        """
        try:
            if self.latest_odom is None:
                return GetOdometryDataResponse(
                    odom_data=Odometry(),
                    success=False,
                    message="No odometry data available yet"
                )
            
            return GetOdometryDataResponse(
                odom_data=self.latest_odom,
                success=True,
                message="Odometry data retrieved successfully"
            )
        except Exception as e:
            rospy.logerr("[nano_sensor_service] Error getting odometry data: %s", str(e))
            return GetOdometryDataResponse(
                odom_data=Odometry(),
                success=False,
                message=f"Error: {str(e)}"
            )


if __name__ == "__main__":
    try:
        server = SensorServiceServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

