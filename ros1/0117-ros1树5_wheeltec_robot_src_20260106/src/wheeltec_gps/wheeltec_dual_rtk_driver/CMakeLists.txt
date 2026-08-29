cmake_minimum_required(VERSION 3.0.2)
project(wheeltec_dual_rtk_driver)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf
)

catkin_python_setup()
catkin_package()
