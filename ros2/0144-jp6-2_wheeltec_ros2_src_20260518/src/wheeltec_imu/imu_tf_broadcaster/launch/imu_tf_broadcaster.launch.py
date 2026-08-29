from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments with default values
        DeclareLaunchArgument(
            'imu_topic',
            default_value='/imu/data_raw',
            description='Topic name for IMU data'
        ),
        DeclareLaunchArgument(
            'position_x',
            default_value='1',
            description='X position offset for the IMU frame'
        ),
        DeclareLaunchArgument(
            'position_y',
            default_value='1',
            description='Y position offset for the IMU frame'
        ),
        DeclareLaunchArgument(
            'position_z',
            default_value='0',
            description='Z position offset for the IMU frame'
        ),
        DeclareLaunchArgument(
            'world_frame_id',
            default_value='world',
            description='Parent frame ID for the transform'
        ),
        DeclareLaunchArgument(
            'imu_frame_id',
            default_value='gyro_link',
            description='Child frame ID for the transform'
        ),
        
        # Node configuration
        Node(
            package='imu_tf_broadcaster',
            executable='imu_tf_broadcaster',
            name='imu_tf_broadcaster',
            output='screen',
            parameters=[{
                'imu_topic': LaunchConfiguration('imu_topic'),
                'position_x': LaunchConfiguration('position_x'),
                'position_y': LaunchConfiguration('position_y'),
                'position_z': LaunchConfiguration('position_z'),
                'world_frame_id': LaunchConfiguration('world_frame_id'),
                'imu_frame_id': LaunchConfiguration('imu_frame_id'),
            }]
        )
    ])