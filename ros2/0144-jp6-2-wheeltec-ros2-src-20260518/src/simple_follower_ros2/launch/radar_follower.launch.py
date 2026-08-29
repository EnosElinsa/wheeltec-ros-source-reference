import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions

#def launch(launch_descriptor, argv):
def generate_launch_description():

    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')
    wheeltec_robot = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'turn_on_wheeltec_robot.launch.py')),
    )
    radar_dir = get_package_share_directory('wheeltec_radar')
    radarlaunch_dir = os.path.join(radar_dir, 'launch')
    wheeltec_radar = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(radarlaunch_dir, 'wheeltec_radar.launch.py')),
    )
    return LaunchDescription([
        wheeltec_robot,wheeltec_radar,
        launch_ros.actions.Node(
            package='simple_follower_ros2', 
            executable='radarTracker', 
            name='radarTracker',
            parameters=[
            {'disable_distance_x': 2.0},   #屏蔽左右方向距离超出disable_distance_x的目标
            ]
             ),
        launch_ros.actions.Node(
            package='simple_follower_ros2', 
            executable='radarfollower', 
            parameters=[
            {'targetDist': 1.2},            #跟随距离
            ]
            ),]
    )

