#!/usr/bin/env python3

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_type = os.getenv('ROBOT_TYPE', 's300_pro').lower()
    
    #超声波避障开关 - 只需要修改这里的True/False
    ENABLE_ULTRASONIC = False  # True=启用超声波, False=禁用超声波
    
    # 获取包路径
    wheeltec_nav_dir = get_package_share_directory('wheeltec_nav2')
    wheeltec_robot_dir = get_package_share_directory('turn_on_wheeltec_robot')
    wheeltec_launch_dir = os.path.join(wheeltec_robot_dir, 'launch')
    wheeltec_nav_launch_dir = os.path.join(wheeltec_nav_dir, 'launch')
    ultra_pkg_dir = get_package_share_directory('wheeltec_ultrasonic')
    ultra_launch_dir = os.path.join(ultra_pkg_dir, 'launch')
    # 参数文件处理
    param_dir = os.path.join(wheeltec_nav_dir, 'param')
    original_param_file = os.path.join(param_dir, f'nav_param_{robot_type}.yaml')
    
    if ENABLE_ULTRASONIC:
        # 生成带超声波配置的参数文件
        param_file_path = create_ultrasonic_param_file(original_param_file, param_dir, robot_type)
        print("超声波避障: Enable")
        print(f"生成超声波参数文件: {param_file_path}")
    else:
        # 使用原始参数文件
        param_file_path = original_param_file
        print("超声波避障: Disable")
    
    # 地图文件路径
    map_dir = os.path.join(wheeltec_nav_dir, 'map')
    map_file = LaunchConfiguration('map', default=os.path.join(map_dir, 'WHEELTEC.yaml'))
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    actions = [
    DeclareLaunchArgument('map', default_value=map_file, description='Full path to map file to load'),
    DeclareLaunchArgument('params', default_value=param_file_path, description='Full path to param file to load'),
    DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation time if true'),
    Node(name='waypoint_cycle', package='nav2_waypoint_cycle', executable='nav2_waypoint_cycle'),
    IncludeLaunchDescription(PythonLaunchDescriptionSource([wheeltec_launch_dir, '/turn_on_wheeltec_robot.launch.py'])),
    IncludeLaunchDescription(PythonLaunchDescriptionSource([wheeltec_launch_dir, '/wheeltec_lidar.launch.py'])),
    IncludeLaunchDescription(PythonLaunchDescriptionSource([wheeltec_nav_launch_dir, '/bringup_launch.py']),
        launch_arguments={'map': map_file, 'use_sim_time': use_sim_time, 'params_file': param_file_path}.items()),
    ]

    if ENABLE_ULTRASONIC:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([ultra_launch_dir, '/supersonic+converter.launch.py'])
            )
        )

    return LaunchDescription(actions)

def create_ultrasonic_param_file(original_file, param_dir, robot_type):
    try:
        with open(original_file, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)

        local_params = data.get('local_costmap', {}).get('local_costmap', {}).get('ros__parameters')
        global_params = data.get('global_costmap', {}).get('global_costmap', {}).get('ros__parameters')
        
     
        local_obstacle_layer = {
            'plugin': 'nav2_costmap_2d::ObstacleLayer',
            'enabled': True,
            'observation_sources': 'scan ultras',
            'scan': {
                'topic': '/scan',
                'max_obstacle_height': 2.0,
                'clearing': True,
                'marking': True,
                'data_type': 'LaserScan',
                'raytrace_max_range': 4.0,
                'raytrace_min_range': 0.0,
                'obstacle_max_range': 3.5,
                'obstacle_min_range': 0.0,
                'inf_is_valid': True
            },
            'ultras': {
                'topic': '/ultrasonic/points',
                'data_type': 'PointCloud2',
                'marking': True,
                'clearing': False,
                'max_obstacle_height': 2.0,
                'min_obstacle_height': 0.0,
                'obstacle_min_range': 0.03,
                'obstacle_max_range': 0.8,
                'raytrace_min_range': 0.03,
                'raytrace_max_range': 0.8,
                'observation_persistence': 10.0
            }
        }

        global_obstacle_layer = {
            'plugin': 'nav2_costmap_2d::ObstacleLayer',
            'enabled': True,
            'observation_sources': 'scan ultras',
            'scan': {
                'topic': '/scan',
                'max_obstacle_height': 2.0,
                'clearing': True,
                'marking': True,
                'data_type': 'LaserScan',
                'raytrace_max_range': 3.0,
                'raytrace_min_range': 0.0,
                'obstacle_max_range': 2.5,
                'obstacle_min_range': 0.0
            },
            'ultras': {
                'topic': '/ultrasonic/points',
                'data_type': 'PointCloud2',
                'marking': True,
                'clearing': False,
                'max_obstacle_height': 2.0,
                'min_obstacle_height': 0.0,
                'obstacle_min_range': 0.03,
                'obstacle_max_range': 0.8,
                'raytrace_min_range': 0.03,
                'raytrace_max_range': 0.8,
                'observation_persistence': 10.0
            }
        }


        local_params['obstacle_layer'] = local_obstacle_layer
        global_params['obstacle_layer'] = global_obstacle_layer

        new_file_name = f'nav_param_{robot_type}_with_ultrasonic.yaml'
        new_file_path = os.path.join(param_dir, new_file_name)
        with open(new_file_path, 'w', encoding='utf-8') as f:
            yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)

        print(f"超声波参数文件已生成: {new_file_path}")
        return new_file_path

    except Exception as e:
        print(f"创建超声波参数文件失败: {e}")
        print(f"使用原始参数文件: {original_file}")
        return original_file
