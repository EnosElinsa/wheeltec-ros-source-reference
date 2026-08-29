from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取包路径
    pkg_dir = get_package_share_directory('stepper_arm')
    # 参数文件路径
    params_file = os.path.join(pkg_dir, 'config', 'auto_pick_colorBlock.yaml')
    
    # 声明参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    return LaunchDescription([
        # 声明启动参数
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'),
        
        Node(
            package='stepper_arm',
            executable='car_avoidance',
            name='car_avoidance',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            output='screen'),
        # 启动色块抓取节点
        Node(
            package='stepper_arm',
            executable='auto_pick_colorBlock',
            name='auto_pick_Block',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            output='screen'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('stepper_arm'),
                    'launch',
                    'usb_cam.launch.py'
                ])
            ])
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('turn_on_wheeltec_robot'),
                    'launch',
                    'turn_on_wheeltec_robot.launch.py'
                ])
            ])
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('turn_on_wheeltec_robot'),
                    'launch',
                    'wheeltec_lidar.launch.py'
                ])
            ])
        ),

      
        
    ])