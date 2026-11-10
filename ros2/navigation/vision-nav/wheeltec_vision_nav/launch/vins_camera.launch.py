"""Acquisition camera only. No chassis, TF model, mapping, cleanup or commands."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from pathlib import Path


def generate_launch_description():
    share = Path(get_package_share_directory("wheeltec_vision_nav"))
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
        Path(get_package_share_directory("realsense2_camera")) / "launch/rs_launch.py")),
        launch_arguments={"camera_namespace": "/", "camera_name": "camera",
            "enable_color": "false", "enable_infra1": "true", "enable_infra2": "true",
            "enable_depth": "true", "enable_sync": "true", "align_depth.enable": "false",
            "depth_module.depth_profile": "848x480x30", "depth_module.infra_profile": "848x480x30",
            "spatial_filter.enable": "true", "spatial_filter.filter_magnitude": "2",
            "temporal_filter.enable": "true", "pointcloud.enable": "false",
            "publish_tf": "true", "initial_reset": "false",
            "config_file": str(share / "config/realsense_d435c.yaml")}.items())])
