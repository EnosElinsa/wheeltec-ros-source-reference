#!/usr/bin/env python3
# coding: utf-8

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    # 获取包路径
    wheeltec_jetracer_dir = get_package_share_directory('wheeltec_jetracer')
    
    # 设置环境变量（可选，用于显示输出）
    set_env = SetEnvironmentVariable(
        name='RCUTILS_CONSOLE_OUTPUT_FORMAT',
        value='[{severity}][{time}][{name}]: {message}'
    )
    
    # 选择使用v550_akm/v550_mec
    # config_file = LaunchConfiguration('config_file', 
    #     default=PathJoinSubstitution([
    #         wheeltec_jetracer_dir, 'params', 'v550_mec.yaml'
    #     ])
    # )
    params_file = os.path.join(
        get_package_share_directory('wheeltec_jetracer'),
        'param',
        'v550_mec.yaml'
    )
    
    # 创建激光检测节点
    laser_detect_node = Node(
        package='wheeltec_jetracer',
        executable='laser_detect',
        name='laser_detect',
        output='screen'
    )
    
    # 创建路径跟随节点
    road_following_node = Node(
        package='wheeltec_jetracer',
        executable='road_following',
        name='wheeltec_jetracer',
        output='screen',
        parameters=[params_file]
    )
    
    # 包含激光跟随节点
    # simple_follower_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             get_package_share_directory('simple_follower_ros2'),
    #             'launch',
    #             'laserTracker.launch.py'  # 注意：ROS2 launch文件通常使用.py扩展名
    #         ])
    #     ]),
    #     condition=IfCondition(LaunchConfiguration('launch_simple_follower', default='true'))
    # )
    
    # 包含机器人底层节点
    turn_on_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('turn_on_wheeltec_robot'),
                'launch',
                'turn_on_wheeltec_robot.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('launch_robot', default='true'))
    )
    
    # 包含雷达启动节点
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory('turn_on_wheeltec_robot'),
                'launch',
                'wheeltec_lidar.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('launch_lidar', default='true'))
    )
    
    
    # 创建启动描述
    ld = LaunchDescription()
    
    # 添加环境变量设置
    ld.add_action(set_env)
    
    # 添加节点
    ld.add_action(laser_detect_node)
    ld.add_action(road_following_node)
    
    # 添加包含的launch文件
    ld.add_action(turn_on_robot_launch)
    ld.add_action(lidar_launch)
    
    return ld
