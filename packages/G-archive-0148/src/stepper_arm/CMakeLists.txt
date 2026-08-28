cmake_minimum_required(VERSION 3.8)
project(stepper_arm)

# 默认使用C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenCV REQUIRED)

# 生成接口
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Position.msg"
  DEPENDENCIES std_msgs
)

# 包含目录
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# 链接目录
# link_directories(${OpenCV_LIB_DIR})

# 添加可执行文件
# add_executable(arm_2_movit_visualization 
#   src/arm_2_movit_visualization.cpp 
#   src/arm_2_control.cpp
# )
# ament_target_dependencies(arm_2_movit_visualization
#   rclcpp
#   moveit_msgs
#   moveit_ros_planning_interface
#   geometry_msgs
#   tf2
#   tf2_ros
# )
# rosidl_target_interfaces(arm_2_movit_visualization
#   ${PROJECT_NAME} "rosidl_typesupport_cpp"
# )

add_executable(arm_2_keyboard_control 
  src/arm_2_keyboard_control.cpp 
)
target_link_libraries(arm_2_keyboard_control
 "${cpp_typesupport_target}"
 ${cv_bridge_LIBRARIES}
 ${OpenCV_LIBRARIES}

 )

ament_target_dependencies(arm_2_keyboard_control
  rclcpp
  geometry_msgs
  sensor_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
${PROJECT_NAME} rosidl_typesupport_cpp)



add_executable(auto_pick_colorBlock 
  src/auto_pick_colorBlock.cpp 
)
ament_target_dependencies(auto_pick_colorBlock
  rclcpp
  cv_bridge
  image_transport
  sensor_msgs
  geometry_msgs
  tf2
  tf2_ros
  nav2_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(auto_pick_colorBlock
 "${cpp_typesupport_target}"
 ${cv_bridge_LIBRARIES}
 ${OpenCV_LIBRARIES}

 )

add_executable(car_avoidance src/car_avoidance.cpp)
ament_target_dependencies(car_avoidance
  rclcpp
  sensor_msgs
  geometry_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
${PROJECT_NAME} rosidl_typesupport_cpp)

target_link_libraries(car_avoidance
 "${cpp_typesupport_target}"

 )


# 安装可执行文件
install(TARGETS
  # arm_2_movit_visualization
  # arm_2_keyboard_control
  arm_2_keyboard_control
  auto_pick_colorBlock
  car_avoidance
  DESTINATION lib/${PROJECT_NAME}
)

# 安装启动文件
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# 安装配置文件
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# 导出依赖
ament_export_dependencies(
  rclcpp
  std_msgs
  moveit_msgs
  moveit_ros_planning_interface
  geometry_msgs
  tf2
  tf2_ros
  cv_bridge
  image_transport
  sensor_msgs
)

# 导出接口
# ament_export_include_directories(include)
# ament_export_libraries(${PROJECT_NAME})
# ament_export_targets(${PROJECT_NAME})

ament_package()