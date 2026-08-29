# Copyright (c) 2024，D-Robotics.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.5)
project(palm_detection_mediapipe)

# For Debug
set(CMAKE_BUILD_TYPE Debug)
add_definitions(-DDEBUG)
add_compile_options(
    -fdebug-prefix-map=/mnt/workspace/cc_ws=/mnt/deyu.li/cc_ws
)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ai_msgs REQUIRED)
find_package(dnn_node REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(hobot_cv REQUIRED)
find_package(rclcpp_components REQUIRED)


find_package(hbm_img_msgs REQUIRED)
add_definitions(-DSHARED_MEM_ENABLED)

if(DEFINED ENV{ROS_DISTRO})
    set(PREFERRED_ROS_DISTRO $ENV{ROS_DISTRO})
else()
    # 设置默认优先级，jazzy优先于humble
    if(EXISTS "/opt/ros/jazzy")
        set(PREFERRED_ROS_DISTRO jazzy)
    elseif(EXISTS "/opt/ros/humble")
        set(PREFERRED_ROS_DISTRO humble)
    else()
        set(PREFERRED_ROS_DISTRO humble)
    endif()
endif()

if(DEFINED PREFERRED_ROS_DISTRO)
    message(STATUS "Preferred ROS distribution: ${PREFERRED_ROS_DISTRO}")
endif()

message("PREFERRED_ROS_DISTRO is " ${PREFERRED_ROS_DISTRO})

if("${PREFERRED_ROS_DISTRO}" STREQUAL "jazzy")
    add_definitions(-DCV_BRIDGE_CPP)
    message("CV_BRIDGE_CPP111 is " ${CV_BRIDGE_CPP})
endif()

if(PLATFORM_X5)
  message("build ${PROJECT_NAME} for platform X5")
  add_definitions(-DPLATFORM_X5)
  set(PREFIX_PATH x5)
elseif(PLATFORM_S100)
  message("build ${PROJECT_NAME} for platform S100")
  add_definitions(-DPLATFORM_S100)
  set(PREFIX_PATH s100)
elseif(PLATFORM_S600)
  message("build ${PROJECT_NAME} for platform S600")
  add_definitions(-DPLATFORM_S600)
  set(PREFIX_PATH s600)
else()
  message("invalid platform, build platform X5 default")
  add_definitions(-DPLATFORM_X5)
  set(PREFIX_PATH x5)
endif()

set(SYS_ROOT ${CMAKE_SYSROOT})
message("CMAKE_SYSROOT is " ${CMAKE_SYSROOT})
message("CV_BRIDGE_CPP is " ${CV_BRIDGE_CPP})
include_directories(include
  ${PROJECT_SOURCE_DIR}
)

include_directories(
  ${SYS_ROOT}/usr/include/
  ${SYS_ROOT}/usr/hobot/include/
  ${SYS_ROOT}/usr/include/hobot/
  ${SYS_ROOT}/usr/include/opencv/
)

link_directories(
  ${SYS_ROOT}/usr/lib/hbbpu/
  ${SYS_ROOT}/usr/hobot/lib/
  ${SYS_ROOT}/usr/lib/hbmedia/
)

add_library(${PROJECT_NAME}_component SHARED
  src/main.cpp
  src/mono2d_palm_det_node.cpp
  src/post_process/palm_det_parser.cpp
  src/post_process/anchors.cpp
)

target_compile_definitions(${PROJECT_NAME}_component
  PRIVATE "COMPOSITION_BUILDING_DLL")
  ament_target_dependencies(
    ${PROJECT_NAME}_component
  )

ament_target_dependencies(
  ${PROJECT_NAME}_component
  rclcpp
  rclcpp_components
  dnn_node
  sensor_msgs
  ai_msgs
  cv_bridge
  hobot_cv
  hbm_img_msgs
)

rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "Mono2dPalmDetNode"
  EXECUTABLE ${PROJECT_NAME}
)

include_directories(include
${PROJECT_SOURCE_DIR}
)

# Install executables
install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
    TARGETS ${PROJECT_NAME}_component
    DESTINATION lib/
  )

install(DIRECTORY
${PROJECT_SOURCE_DIR}/config/${PREFIX_PATH}/
DESTINATION lib/${PROJECT_NAME}/config/
)

install(FILES
  ${PROJECT_SOURCE_DIR}/config/example.jpg
  DESTINATION lib/${PROJECT_NAME}/config/
)

install(DIRECTORY
${PROJECT_SOURCE_DIR}/launch/
DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
