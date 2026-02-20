#!/usr/bin/env python3
"""
Motor Service Server for Jetson Nano (ROS1)

Provides XML-RPC services for motor control:
- /nano/set_motor_velocity: Set motor velocity with confirmation
- /nano/get_robot_status: Get current robot status

This replaces the topic-based /cmd_vel with service-based communication.
"""

import rospy
from geometry_msgs.msg import Twist
from jetbot_nano_bringup.srv import SetMotorVelocity, SetMotorVelocityResponse
from jetbot_nano_bringup.srv import GetRobotStatus, GetRobotStatusResponse


class MotorServiceServer(object):
    """
    Service server that handles motor velocity commands via XML-RPC.
    """

    def __init__(self):
        self.node_name = "nano_motor_service_server"
        rospy.init_node(self.node_name)

        # Parameters for differential drive kinematics
        self.wheel_base = rospy.get_param("~wheel_base", 0.2)   # meters
        self.wheel_radius = rospy.get_param("~wheel_radius", 0.03)  # meters

        # Current velocity state
        self.current_velocity = Twist()
        self.motors_active = False

        # Advertise services
        self.set_velocity_service = rospy.Service(
            '/nano/set_motor_velocity',
            SetMotorVelocity,
            self.handle_set_velocity
        )

        self.get_status_service = rospy.Service(
            '/nano/get_robot_status',
            GetRobotStatus,
            self.handle_get_status
        )

        rospy.loginfo("[%s] Started. Services available:" % self.node_name)
        rospy.loginfo("[%s]   - /nano/set_motor_velocity" % self.node_name)
        rospy.loginfo("[%s]   - /nano/get_robot_status" % self.node_name)

    def handle_set_velocity(self, req):
        """
        Handle SetMotorVelocity service request.
        
        Args:
            req: SetMotorVelocity request containing Twist velocity command
            
        Returns:
            SetMotorVelocityResponse with success status and message
        """
        try:
            # Store current velocity
            self.current_velocity = req.velocity
            self.motors_active = True

            v = req.velocity.linear.x      # forward velocity (m/s)
            omega = req.velocity.angular.z  # angular velocity (rad/s)

            # Simple differential drive inverse kinematics
            v_left = v - (omega * self.wheel_base / 2.0)
            v_right = v + (omega * self.wheel_base / 2.0)

            # Optionally convert to wheel angular speed (rad/s)
            if self.wheel_radius > 0:
                w_left = v_left / self.wheel_radius
                w_right = v_right / self.wheel_radius
            else:
                w_left = v_left
                w_right = v_right

            rospy.loginfo("[nano_motor_service] Velocity command received: v=%.3f, omega=%.3f", v, omega)
            rospy.loginfo("[nano_motor_service] Left wheel: v=%.3f m/s (%.3f rad/s), Right wheel: v=%.3f m/s (%.3f rad/s)",
                          v_left, w_left, v_right, w_right)

            # TODO: Here you would send actual commands to motor hardware
            # For now, this is a placeholder that logs the command

            # Return success response
            return SetMotorVelocityResponse(
                success=True,
                message=f"Velocity set: linear={v:.3f} m/s, angular={omega:.3f} rad/s"
            )

        except Exception as e:
            rospy.logerr("[nano_motor_service] Error setting velocity: %s", str(e))
            return SetMotorVelocityResponse(
                success=False,
                message=f"Error: {str(e)}"
            )

    def handle_get_status(self, req):
        """
        Handle GetRobotStatus service request.
        
        Args:
            req: GetRobotStatus request
            
        Returns:
            GetRobotStatusResponse with current robot status
        """
        try:
            return GetRobotStatusResponse(
                motors_active=self.motors_active,
                current_linear_velocity=self.current_velocity.linear.x,
                current_angular_velocity=self.current_velocity.angular.z,
                status_message="Robot operational",
                success=True
            )
        except Exception as e:
            rospy.logerr("[nano_motor_service] Error getting status: %s", str(e))
            return GetRobotStatusResponse(
                motors_active=False,
                current_linear_velocity=0.0,
                current_angular_velocity=0.0,
                status_message=f"Error: {str(e)}",
                success=False
            )


if __name__ == "__main__":
    try:
        server = MotorServiceServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

