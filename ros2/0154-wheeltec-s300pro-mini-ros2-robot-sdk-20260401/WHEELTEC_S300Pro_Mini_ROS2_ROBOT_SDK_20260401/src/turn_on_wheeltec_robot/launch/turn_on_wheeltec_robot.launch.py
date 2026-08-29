import os
from pathlib import Path
import launch
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import PushRosNamespace
import launch_ros.actions
from launch.conditions import UnlessCondition

def generate_launch_description():
        # Get the launch directory
        bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
        launch_dir = os.path.join(bringup_dir, 'launch')
        ekf_config = Path(get_package_share_directory('turn_on_wheeltec_robot'), 'config', 'ekf.yaml')
        imu_config = Path(get_package_share_directory('turn_on_wheeltec_robot'), 'config', 'imu.yaml')

        carto_slam = LaunchConfiguration('carto_slam', default='false')
        carto_slam_dec = DeclareLaunchArgument('carto_slam',default_value='false')

        wheeltec_robot = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, 'base_serial.launch.py')),
                 launch_arguments={'Ultrasonic_Avoid': 'false'}.items(),)
        #choose your car,the default car is mini_mec 
        choose_car = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, 'robot_mode_description.launch.py')),
        )
        robot_ekf = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, 'wheeltec_ekf.launch.py')),
                launch_arguments={'carto_slam':carto_slam}.items(),            
        )
        choose_car = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, 'robot_mode_description.launch.py')),
        )


        robot_type = os.getenv('ROBOT_TYPE', 's300_pro').lower()

        if   robot_type == 's300_pro':
                transform_args = ['0.173573593143821', '0', '0.208299999999995', '0', '0', '0', 'base_footprint', 'base_link']
        elif robot_type == 's300_mini':
                transform_args = ['0.0839713612660109', '0', '0.1904000000000005', '0', '0', '0', 'base_footprint', 'base_link']
        else:
                transform_args = ['0.0839713612660109', '0', '0.21000000000005', '0', '0', '0', 'base_footprint', 'base_link']

        baselink_to_base_footprint = launch_ros.actions.Node(
        package='tf2_ros', 
        executable='static_transform_publisher', 
        name='base_to_link',
        arguments=transform_args
        )           
        

        imu_filter_node=launch_ros.actions.Node(
                package='imu_filter_madgwick',
                executable='imu_filter_madgwick_node',
                parameters=[imu_config]
        )

                                
        ld = LaunchDescription()

        ld.add_action(carto_slam_dec)
        ld.add_action(wheeltec_robot)
        ld.add_action(robot_ekf)
        ld.add_action(baselink_to_base_footprint)
        ld.add_action(choose_car)
        ld.add_action(imu_filter_node)

    

        return ld

