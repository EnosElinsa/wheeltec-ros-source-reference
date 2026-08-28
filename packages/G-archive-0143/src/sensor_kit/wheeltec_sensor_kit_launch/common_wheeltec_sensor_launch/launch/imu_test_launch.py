import os 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction,IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from launch.conditions import IfCondition,UnlessCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


#def launch(launch_descriptor, argv):
def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('common_wheeltec_sensor_launch'),
        'params',
        'yesense_config.yaml',
    )
    
    # Declare the launch arguments
    use_wheeltec_imu_declare = DeclareLaunchArgument(
        'use_wheeltec_imu',
        default_value='false',  
        description='If true, use wheeltec_imu'
    )
    #declare_use_imu = LaunchConfiguration('use_wheeltec_imu')

    

    # Extract common parameters
    common_params = {
        'usart_port_name': '/dev/wheeltec_controller',
        'serial_baud_rate': 115200,
        'robot_frame_id': 'base_footprint',
        'odom_frame_id': 'odom_combined',
        'cmd_vel': 'cmd_vel',
        'akm_cmd_vel': 'none',
        'product_number': 0,
        'odom_x_scale': 1.0,
        'odom_y_scale': 1.0,
        'odom_z_scale_positive': 1.0,
        'odom_z_scale_negative': 1.0
    }
    
    remappings=[('imu/data_raw', 'imu/data_board')]

    turn_on_robot_use_imu = Node(
        condition=UnlessCondition(LaunchConfiguration('use_wheeltec_imu')),
        package='turn_on_wheeltec_robot', 
        executable='wheeltec_robot_node', 
        output='screen',
        remappings=[('imu/data_raw', '/sensing/imu/imu_data')],
        parameters=[common_params],
    )

    turn_on_robot = GroupAction(
        condition=IfCondition(LaunchConfiguration('use_wheeltec_imu')),
        actions=[
            Node(
                package='turn_on_wheeltec_robot', 
                executable='wheeltec_robot_node', 
                output='screen',
                parameters=[common_params],
                remappings=remappings,),
            Node(
            package='yesense_std_ros2',
            executable='yesense_node_publisher',
            name='yesense_pub',
            parameters=[config],
            output='screen',
            ),
        ]
    )
    tf1 = Node(
        package='tf2_ros', 
        executable='static_transform_publisher', 
        name='base_to_odom',
        arguments=['0', '0', '0','0', '0','0','base_link','odom_combined'],
        )
    tf2 = Node(
        package='tf2_ros', 
        executable='static_transform_publisher', 
        name='base_to_gyro',
        arguments=['0', '0', '0','0', '0','0','base_link','gyro_link'],
        )

    ld = LaunchDescription()
    ld.add_action(use_wheeltec_imu_declare)  # Ìí¼ÓÉùÃ÷µ½ld
    ld.add_action(turn_on_robot_use_imu)
    ld.add_action(turn_on_robot)
    ld.add_action(tf1)
    ld.add_action(tf2)

    return ld
