import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition
def generate_launch_description():
    # 获取包的共享目录
    params_file=os.path.join(get_package_share_directory('largemodel'), "config", "param.yaml")
    wheeltec_robot_dir = get_package_share_directory('turn_on_wheeltec_robot')
    wheeltec_sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(wheeltec_robot_dir,'launch','wheeltec_sensors.launch.py')
            ),
        )
    
    wheeltec_mic = Node(
        package="wheeltec_mic_aiui",
        executable="wheeltec_mic",
        output='screen',
        parameters=[{"usart_port_name": "/dev/wheeltec_mic",
                    "serial_baud_rate": 115200}]
    )

    wheeltec_mic_aiui = Node(
        package="wheeltec_mic_aiui",
        executable="wheeltec_mic_aiui",
        output='screen',
    )

    # 定义节点
    model_server = Node(
        package='largemodel',
        executable='model_service',
        name='model_service',
        parameters=[params_file],
        output='screen'
    )
    action_server = Node(
        package='largemodel',
        executable='action_service',
        name='action_service',
        parameters=[params_file ],
        output='screen'
    )
    lasertracker = Node(
        package="simple_follower_ros2", 
        executable="lasertracker", 
        name='lasertracker'
    )
    return LaunchDescription([
        model_server,          #启动模型服务节点
        action_server,         #启动动作服务节点
        wheeltec_mic,
        wheeltec_mic_aiui,
        lasertracker,
        wheeltec_sensors
    ])





