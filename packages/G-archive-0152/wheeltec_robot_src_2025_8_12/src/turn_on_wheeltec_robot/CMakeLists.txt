cmake_minimum_required(VERSION 2.8.3)
project(turn_on_wheeltec_robot)
set(PCL_H "/usr/include/pcl-1.8")
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  nav_msgs
  nav_msgs
  roscpp
  serial
  roslib
  rospy
  sensor_msgs
  std_msgs
  tf
  message_filters
  moveit_msgs
  moveit_ros_perception
  moveit_ros_planning_interface
  industrial_robot_client
)
find_package(Boost REQUIRED)


add_message_files(FILES supersonic.msg)

add_service_files(FILES SetRgb.srv)

generate_messages(DEPENDENCIES std_msgs)
# ulimit -c unlimited
# ssh firefly@192.168.1.76
add_compile_options(-g -std=c++11)    #-g -> core dumped

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES turn_on_wheeltec_robot
  CATKIN_DEPENDS geometry_msgs message_generation nav_msgs nav_msgs roscpp roslib rospy sensor_msgs std_msgs tf message_runtime   moveit_msgs  moveit_ros_perception  moveit_ros_planning_interface  industrial_robot_client
  #DEPENDS system_lib
)

# defind robot status massage
# command atkin_make -DCATKIN_WHITELIST_PACKAGES="new_msg"
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
include_directories(include/${PROJECT_NAME})
include_directories(${catkin_INCLUDE_DIRS} ${PCL_H})

add_executable(wheeltec_robot_node src/wheeltec_robot.cpp src/Quaternion_Solution.cpp)
target_link_libraries(wheeltec_robot_node ${catkin_LIBRARIES})

add_executable(arm_uplift src/arm_uplift.cpp)
target_link_libraries(arm_uplift ${catkin_LIBRARIES})

add_executable(action_nav src/action_nav.cpp)
target_link_libraries(action_nav ${catkin_LIBRARIES})

add_executable(self_check_node src/self_check.cpp src)
target_link_libraries(self_check_node ${catkin_LIBRARIES})