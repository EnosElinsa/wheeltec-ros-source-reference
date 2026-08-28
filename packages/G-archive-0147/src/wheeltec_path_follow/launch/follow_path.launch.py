import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import launch_ros.actions


def generate_launch_description():
    #路径跟踪文件路径
    pathfilename = DeclareLaunchArgument('pathfilename',default_value='/home/wheeltec/wheeltec_ros2/src/wheeltec_path_follow/path/wheeltec_path')
    #是否循环进行路径跟踪
    run_in_loop = DeclareLaunchArgument('run_in_loop',default_value='False')
    #路径跟踪中是否绕开障碍物：
    #True：绕开障碍物，实际路径会变化
    #False:停障，适合复杂路径、复杂环境
    avoid = DeclareLaunchArgument('avoid',default_value='True')
    
    #以下为pure_pursuit路径跟踪节点（只停障的运动控制节点）参数
    #lookahead_distance_前瞻点（即，车辆一直跟踪的点）：m
    lookahead_distance_ = DeclareLaunchArgument('lookahead_distance_',default_value='0.4')
    #最大角速度
    w_max = DeclareLaunchArgument('w_max',default_value='2.0')
    #最大线速度
    v_max = DeclareLaunchArgument('v_max',default_value='0.2')
    #位置容忍度：m
    position_tolerance = DeclareLaunchArgument('position_tolerance',default_value='0.05')
    #障碍物停障距离：m
    avoid_distance = DeclareLaunchArgument('avoid_distance',default_value='0.5')

    follow_path = launch_ros.actions.Node(
            package='wheeltec_path_follow', 
            executable='follow_path.py', 
            name='follow_path',
            output='screen',
            parameters=[{'pathfilename': LaunchConfiguration('pathfilename')},
                    {'run_in_loop': LaunchConfiguration('run_in_loop')},
                    {'avoid': LaunchConfiguration('avoid')},]
    )

    pure_pursuit = launch_ros.actions.Node(
            condition=UnlessCondition(LaunchConfiguration('avoid')),
            package='wheeltec_path_follow', 
            executable='pure_pursuit', 
            name='pure_pursuit',
            output='screen',
            parameters=[{'lookahead_distance_': LaunchConfiguration('lookahead_distance_')},
                    {'w_max': LaunchConfiguration('w_max')},
                    {'v_max': LaunchConfiguration('v_max')},
                    {'position_tolerance': LaunchConfiguration('position_tolerance')},
                    {'avoid_distance': LaunchConfiguration('avoid_distance')},]
    )
    #提取最近的障碍物距离信息
    laser_tracker = launch_ros.actions.Node(
            condition=UnlessCondition(LaunchConfiguration('avoid')),
            package='simple_follower_ros2', 
            executable='lasertracker', 
            name='lasertracker'
            )

    return LaunchDescription([
        pathfilename,run_in_loop,avoid,lookahead_distance_,w_max,v_max,position_tolerance,avoid_distance,
        follow_path,pure_pursuit,laser_tracker
    ])

