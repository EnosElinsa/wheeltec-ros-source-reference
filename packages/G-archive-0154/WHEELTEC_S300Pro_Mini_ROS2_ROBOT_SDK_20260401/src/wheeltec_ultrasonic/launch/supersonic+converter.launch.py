#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    robot_type_env = os.getenv('ROBOT_TYPE', 's300_mini').lower()
    return LaunchDescription([
        DeclareLaunchArgument('robot_type', default_value=robot_type_env),
        DeclareLaunchArgument('input_topic', default_value='Distance'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument('publish_pointcloud', default_value='true'),
        Node(
            package='wheeltec_ultrasonic',
            executable='supersonic_converter_node',
            name='supersonic_converter',
            parameters=[{
                'robot_type': LaunchConfiguration('robot_type'),
                'input_topic': LaunchConfiguration('input_topic'),
                'base_frame': LaunchConfiguration('base_frame'),
                'publish_pointcloud': LaunchConfiguration('publish_pointcloud'),
                'min_range': 0.02, 'max_range': 0.8, 'field_of_view': 0.5
            }]
        )
    ])
