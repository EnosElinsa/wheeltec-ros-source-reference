import os
from pathlib import Path
import launch
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace
import launch_ros.actions
from launch.conditions import UnlessCondition


def generate_launch_description():
    lidar_dir = get_package_share_directory('lslidar_driver')
    wheeltec_lidar = IncludeLaunchDescription(
          PythonLaunchDescriptionSource(
               os.path.join(lidar_dir, 'launch', 'lslidar_double_launch.py')
          ),
     )
    
    lidar_fusion_dir= get_package_share_directory('double_lidar_fusion')
    lidar_fusion= IncludeLaunchDescription(
          PythonLaunchDescriptionSource(
               os.path.join(lidar_fusion_dir, 'launch', 'double_lidar_fusion.launch.py')
          ),
     )

    ld = LaunchDescription()
    ld.add_action(wheeltec_lidar)
    ld.add_action(lidar_fusion)
    return ld