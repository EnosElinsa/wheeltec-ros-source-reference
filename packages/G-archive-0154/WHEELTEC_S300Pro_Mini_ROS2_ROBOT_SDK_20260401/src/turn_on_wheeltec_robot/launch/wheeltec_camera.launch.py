import os
from pathlib import Path
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources import AnyLaunchDescriptionSource

def generate_launch_description():
	astra_dir = get_package_share_directory('orbbec_camera')
	astra_launch_dir = os.path.join(astra_dir,'launch')
 
	
	Gemini_car = IncludeLaunchDescription(
	AnyLaunchDescriptionSource(os.path.join(astra_launch_dir,'gemini_car.launch.py')),)
	ld = LaunchDescription()
	ld.add_action(Gemini_car)

	return ld
