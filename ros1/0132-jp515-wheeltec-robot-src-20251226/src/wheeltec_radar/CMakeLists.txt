cmake_minimum_required(VERSION 2.8.3)
project(wheeltec_radar)


# find dependencies
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  pcl_ros
  pcl_conversions
  roscpp
  rospy
  serial
  roslib
  rospy
  sensor_msgs
  std_msgs
  tf
  geometry_msgs
)

add_message_files(FILES
  RadarDetection.msg
  RadarDetectionArray.msg 
  RadarDetectionStamped.msg
  RadarTrack.msg
  RadarTrackArray.msg
  RadarTrackStamped.msg  
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
# ulimit -c unlimited
# ssh firefly@192.168.1.76
# add_compile_options(-g -std=c++14)    #-g -> core dumped

catkin_package(
  #INCLUDE_DIRS include
  #LIBRARIES turn_on_wheeltec_robot
  CATKIN_DEPENDS geometry_msgs roscpp rospy sensor_msgs std_msgs message_runtime
  #DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(include include/${PROJECT_NAME})
include_directories(include ${PCL_H})

add_executable(wheeltec_radar_node src/wheeltec_radar.cpp)
add_dependencies(wheeltec_radar_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wheeltec_radar_node
  ${catkin_LIBRARIES}
)

add_executable(wheeltec_radar_net_node src/wheeltec_radar_net.cpp)
add_dependencies(wheeltec_radar_net_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(wheeltec_radar_net_node
  ${catkin_LIBRARIES}
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

