#!/usr/bin/env python3
"""
robot_bridge.launch.py — 一键启动 action_service + action_bridge
可选参数:
  params_file  — 参数文件路径（默认使用包内 config/param.yaml）

用法:
  ros2 launch largemodel robot_bridge.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    pkg_share = get_package_share_directory('largemodel')
    default_params = os.path.join(pkg_share, 'config', 'param.yaml')

    cyclonedds_config = os.path.join(pkg_share, 'config', 'cyclonedds.xml')
    cyclone_env = SetEnvironmentVariable(
        'CYCLONEDDS_URI', f'file://{cyclonedds_config}')

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='参数文件路径',
    )

    action_server = Node(
        package='largemodel',
        executable='action_service',
        name='action_service',
        parameters=[LaunchConfiguration('params_file')],
        output='screen',
    )

    action_bridge = Node(
        package='largemodel',
        executable='action_bridge',
        name='action_bridge',
        output='screen',
    )
    wheeltec_robot_dir = get_package_share_directory('turn_on_wheeltec_robot')
    wheeltec_sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wheeltec_robot_dir,'launch','wheeltec_sensors.launch.py')
            ),
        )

    wheeltec_mic_aiui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('wheeltec_mic_aiui'),
                                                        'launch',
                                                        'mic_start.launch.py')
            ),
        )
    lasertracker = Node(
        package="simple_follower_ros2", 
        executable="lasertracker", 
        name='lasertracker'
    )

    return LaunchDescription([
        params_arg,
        cyclone_env,
        action_server,
        action_bridge,
        wheeltec_sensors,
        wheeltec_mic_aiui,
        lasertracker
    ])