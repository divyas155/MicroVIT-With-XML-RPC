cmake_minimum_required(VERSION 3.0.2)
project(jetbot_nano_bringup)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
  message_generation
)

add_service_files(
  FILES
  SetMotorVelocity.srv
  GetRobotStatus.srv
  GetLidarData.srv
  GetOdometryData.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  sensor_msgs
  nav_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs sensor_msgs nav_msgs std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  nodes/motor_driver_node.py
  nodes/motor_service_server.py
  nodes/sensor_service_server.py
  nodes/xmlrpc_server_nano.py
  nodes/lidar_dummy_node.py
  nodes/odom_dummy_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


