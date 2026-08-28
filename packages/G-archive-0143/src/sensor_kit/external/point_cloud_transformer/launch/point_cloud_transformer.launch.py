#!/usr/bin/python3


import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import AnonName
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
import os
import yaml


def generate_launch_description():

    my_component = ComposableNode(
        package="point_cloud_transformer",
        plugin="PointCloudTransformer",
        name="point_cloud_transformer_node",
    )

    # set container to run all required components in the same process
    container = ComposableNodeContainer(
        name="pointcloud_container",
        namespace='',
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[my_component],
        output="screen",
    )

    return launch.LaunchDescription(
        [
            container,
        ]
    )
