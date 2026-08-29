# // Copyright (c) 2025，D-Robotics.
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
project(mono_edgesam)

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
find_package(ai_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(dnn_node REQUIRED)
find_package(hobot_cv REQUIRED)
find_package(hbm_img_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

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

if("${PREFERRED_ROS_DISTRO}" STREQUAL "jazzy")
    add_definitions(-DCV_BRIDGE_CPP)
endif()

if(PLATFORM_X5)
  message("build platform X5")
  add_definitions(-DPLATFORM_X5)
  set(PREFIX_PATH x5)
elseif(PLATFORM_S100)
  message("build platform S100")
  add_definitions(-DPLATFORM_S100)
  set(PREFIX_PATH s100)
elseif(PLATFORM_S600)
  message("build platform S600")
  add_definitions(-DPLATFORM_S600)
  set(PREFIX_PATH s600)
else()
  message("invalid platform, build platform X5 default")
  add_definitions(-DPLATFORM_X5)
  set(PREFIX_PATH x5)
endif()

message("CMAKE_SYSROOT is " ${CMAKE_SYSROOT})

include_directories(
  include
  ${PROJECT_SOURCE_DIR}
)

include_directories(
  ${CMAKE_SYSROOT}/usr/include/
  ${CMAKE_SYSROOT}/usr/include/hobot/
  ${CMAKE_SYSROOT}/usr/include/opencv/
  ${CMAKE_SYSROOT}/usr/include/eigen/
)

link_directories(
  ${CMAKE_SYSROOT}/usr/lib/hbbpu/
  ${CMAKE_SYSROOT}/usr/lib/hbmedia/
  ${CMAKE_SYSROOT}/usr/hobot/lib/
)

add_executable(${PROJECT_NAME}
  src/main.cpp
  src/ai_msg_manage.cpp
  src/${PREFIX_PATH}/edgesam_node.cpp
  src/edgesam_output_parser.cpp
  src/${PREFIX_PATH}/data_preprocess.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  ai_msgs
  cv_bridge
  dnn_node
  hobot_cv
  hbm_img_msgs
  sensor_msgs
)

include_directories(include
  ${PROJECT_SOURCE_DIR}
)

# Install executables
install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/config/${PREFIX_PATH}/
  DESTINATION lib/${PROJECT_NAME}/config/
)

install(FILES
  ${PROJECT_SOURCE_DIR}/config/4.jpg
  DESTINATION lib/${PROJECT_NAME}/config/
)

install(DIRECTORY
  ${PROJECT_SOURCE_DIR}/launch/${PREFIX_PATH}/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
