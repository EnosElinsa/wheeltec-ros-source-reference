cmake_minimum_required(VERSION 3.8)
project(wheeltec_ultrasonic)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(supersonic_converter_node src/supersonic_converter_node.cpp)
ament_target_dependencies(supersonic_converter_node rclcpp sensor_msgs std_msgs robot_interfaces tf2 tf2_ros tf2_geometry_msgs)

install(TARGETS supersonic_converter_node
  DESTINATION lib/${PROJECT_NAME})


add_executable(scan_fixer src/scan_fixer.cpp)
ament_target_dependencies(scan_fixer rclcpp sensor_msgs   )

install(TARGETS scan_fixer
  DESTINATION lib/${PROJECT_NAME})


install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

