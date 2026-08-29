import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    
    bringup_dir = get_package_share_directory('turn_on_wheeltec_robot')
    launch_dir = os.path.join(bringup_dir, 'launch')

    wheeltec_camera = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir,'launch', 'wheeltec_camera.launch.py')),
    )

    return LaunchDescription([
    launch_ros.actions.Node(
            package='aruco_ros', 
            executable='single', 
            parameters=[
                {'image_is_rectified': False},
                {'marker_size': 0.07},
                {'marker_id': 8},
                {'reference_frame':'camera_arm_link'},
                {'camera_frame': 'camera_arm_link'},
                {'marker_frame': 'aruco_marker_frame'},
                {'corner_refinement':'LINES'}
                ],
       	    remappings=[('/camera_info', '/gemini_info'),
                    ('/image', '/camera_arm/color/image_raw')],
            output='screen',
            ),]
            
    )
    
    
