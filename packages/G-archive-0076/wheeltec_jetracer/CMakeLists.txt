cmake_minimum_required(VERSION 2.8.3)
project(wheeltec_jetracer)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp rospy)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS roscpp
  DEPENDS
)



