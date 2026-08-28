from launch import LaunchDescription
import launch_ros
import os
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def create_tf_publisher(name, params, parent_frame, child_frame):
    """Helper function to create static transform publisher node with new-style arguments"""
    return launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=f'base_to_{name}',
        arguments=[
            '--x', str(params[0]),
            '--y', str(params[1]),
            '--z', str(params[2]),
            '--roll', str(params[3]),
            '--pitch', str(params[4]),
            '--yaw', str(params[5]),
            '--frame-id', parent_frame,
            '--child-frame-id', child_frame
        ]
    )

def generate_launch_description():
    robot_type = os.getenv('ROBOT_TYPE', 's300_pro').lower()
    enable_arm = os.getenv('ENABLE_ARM', 'false').lower() == 'true'
    
    robot_configs = {
        's300_pro': {
            'package': 'rm_description',
            'urdf': 's300_pro.xacro',
            'static_tf': {
                'laser': ['0.0', '0.00', '0.19', '0', '0', '0'],  
                'laser1': ['0.3', '0.235', '0.19', '0', '0', '0.785'],  
                'laser2': ['-0.3', '-0.235', '0.19', '0', '0', '-2.355'], 
                'ultrasonic_A': ['0.34', '0.275', '0.09', '0', '0', '0.707'],  
                'ultrasonic_B': ['0.34', '0.17', '0.09', '0', '0', '0'],  
                'ultrasonic_C': ['0.34', '0.055', '0.09', '0', '0', '0'],  
                'ultrasonic_D': ['0.34', '-0.06', '0.09', '0', '0', '0'], 
                'ultrasonic_E': ['0.34', '-0.17', '0.09', '0', '0', '0'],  
                'ultrasonic_F': ['0.34', '-0.275', '0.09', '0', '0', '-0.707'], 
                'gyro_link': ['0', '0', '0', '0', '0', '0']  
            } 
        },
        's300_mini': {
            'package': 'rm_description',
            'urdf': 's300_mini.xacro',
            'static_tf': {
                'laser': ['0', '0', '0.16', '0', '0', '0'],
                'laser1': ['0.215', '0.175', '0.155', '0', '0', '0.7854'],
                'laser2': ['-0.215', '-0.175', '0.155', '0', '0', '-2.355'],
                'camera_link': ['0.06', '0', '0.22', '0', '0', '0'],
                'ultrasonic_A': ['0.26', '0.20', '0.09', '0', '0', '0.7854'],
                'ultrasonic_B': ['0.26', '0.11', '0.09', '0', '0', '0'],
                'ultrasonic_C': ['0.26', '0', '0.09', '0', '0', '0'],
                'ultrasonic_D': ['0.26', '-0.11', '0.09', '0', '0', '0'],
                'ultrasonic_E': ['0.26', '-0.20', '0.09', '0', '0', '-0.7854'],
                'gyro_link':    ['0', '0' ,'0', '0' ,'0' ,'0']
            }
        },
        's300': {
            'package': 'rm_description',
            'urdf': 's300.xacro',
            'static_tf': {
                'laser': ['0.0', '0.00', '0.19', '0', '0', '0'],  
                'laser1': ['-0.3', '-0.235', '0.19', '0', '0', '-2.355'],  
                'laser2': ['0.3', '0.235', '0.19', '0', '0', '0.785'], 
                'ultrasonic_A': ['0.34', '0.275', '0.05', '0', '0', '0'],  
                'ultrasonic_B': ['0.34', '0.165', '0.05', '0', '0', '0'],  
                'ultrasonic_C': ['0.34', '0.055', '0.05', '0', '0', '0'],  
                'ultrasonic_D': ['0.34', '-0.055', '0.05', '0', '0', '0'], 
                'ultrasonic_E': ['0.34', '-0.165', '0.05', '0', '0', '0'],  
                'ultrasonic_F': ['0.34', '-0.275', '0.05', '0', '0', '0'], 
               
                'gyro_link': ['0', '0', '0', '0', '0', '0']  
            }
        },
        
    }

    config = robot_configs[robot_type]
    
    # 创建所有静态TF发布节点
    nodes = []

    # Create TF publishers for all static transforms
    for frame_name, params in config['static_tf'].items():
        nodes.append(
            create_tf_publisher(
                name=frame_name.replace('_link', ''),  # Remove '_link' for node naming
                params=params,
                parent_frame='base_footprint',
                child_frame=frame_name
            )
        )

    # 如果不启用机械臂，添加机器人状态发布器
    if not enable_arm:
        nodes.append(
            # Robot State Publisher
            launch_ros.actions.Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[{
                    'robot_description': Command([
                        'xacro ' if config['urdf'].endswith('.xacro') else 'cat ',
                        PathJoinSubstitution([
                            FindPackageShare(config['package']),
                            'urdf',
                            config['urdf']
                        ])
                    ])
                }]
            )  
        )
        nodes.append(
        # Joint State Publisher
            launch_ros.actions.Node(
                package='joint_state_publisher',
                executable='joint_state_publisher',
                name='joint_state_publisher'
            )
        )
        nodes.append(
            launch_ros.actions.Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='car_camera_link',
                arguments=['0.27', '0', '0.23','0', '0','0','base_footprint','camera_link'],
            )
        )

    return LaunchDescription(nodes)
