cmake_minimum_required(VERSION 3.14)
project(point_cloud_transformer)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Ignore -Wnonportable-include-path in Clang for mussp
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-nonportable-include-path)
endif()

### Find PCL Dependencies
find_package(PCL REQUIRED)

find_package(pcl_conversions REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(
  include
  SYSTEM
    ${PCL_COMMON_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)


ament_auto_add_library(point_cloud_transformer_node SHARED
  src/point_cloud_transformer.cpp
)

target_link_libraries(point_cloud_transformer_node
  ${PCL_LIBRARIES}
)

rclcpp_components_register_node(point_cloud_transformer_node
  PLUGIN "PointCloudTransformer"
  EXECUTABLE point_cloud_transformer
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
