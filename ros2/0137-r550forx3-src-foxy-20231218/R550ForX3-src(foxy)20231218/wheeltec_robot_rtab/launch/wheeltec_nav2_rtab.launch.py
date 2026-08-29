
import os
from ament_index_python import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')
    
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    nav2_dir = get_package_share_directory('wheeltec_nav2')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    rtab_bringup = get_package_share_directory('wheeltec_robot_rtab')

    
    nav_config = LaunchConfiguration('params_file', default=os.path.join(nav2_dir, 'param', 'wheeltec_rtab_params.yaml'))
    map_path = LaunchConfiguration('map', default=os.path.join(nav2_dir, 'map', 'WHEELTEC.yaml'))
    
    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch','turn_on_wheeltec_robot.launch.py')),
    )
    wheeltec_lidar = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'wheeltec_lidar.launch.py')),
    )
    wheeltec_camera = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir,'launch', 'wheeltec_camera.launch.py')),
    )


    return LaunchDescription([
        wheeltec_robot,wheeltec_lidar,wheeltec_camera,
        # Launch arguments

        DeclareLaunchArgument('use_sim_time', default_value='false',    description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('map',          default_value=map_path,   description='Full path to map file to load'),
        DeclareLaunchArgument('nav_config',   default_value=nav_config, description='Full path to param file to load'),
        DeclareLaunchArgument('localization', default_value='true',     description='Launch in localization mode.'),
 
         # Nodes to launch
            
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')),
                launch_arguments={      
                    'map': map_path,
                    'use_sim_time': use_sim_time,
                    'nav_config': nav_config}.items(),
        ),
        IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(rtab_bringup, 'launch', 'wheeltec_slam_rtab.launch.py')),
                launch_arguments={      
                    'localization': localization}.items(),
       )
    ])
