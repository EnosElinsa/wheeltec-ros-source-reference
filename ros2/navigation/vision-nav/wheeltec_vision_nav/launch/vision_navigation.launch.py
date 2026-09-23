import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_utils import (
    capture_runtime_node,
    delayed_camera_stack,
    launch_flag,
    model_actions,
    package_file,
    visual_runtime_actions,
    write_visual_nav_params,
)


def generate_launch_description():
    car_mode = LaunchConfiguration("car_mode")
    wheeltec_param = LaunchConfiguration("wheeltec_param_yaml")
    robot_model = LaunchConfiguration("robot_model_yaml")
    database = LaunchConfiguration("database_path")
    nav_params = LaunchConfiguration("nav_params_file")
    start_base = LaunchConfiguration("start_base")
    autostart = LaunchConfiguration("autostart")
    hold_odom = LaunchConfiguration("hold_odom")
    capture = LaunchConfiguration("capture")
    capture_dir = LaunchConfiguration("capture_dir")
    camera_xyz = LaunchConfiguration("camera_xyz")

    def build_model(context, *args, **kwargs):
        return model_actions(context, car_mode, wheeltec_param, robot_model, camera_xyz)

    def build_runtime(context, *args, **kwargs):
        return visual_runtime_actions(
            database.perform(context),
            localization=True,
            start_base=launch_flag(start_base.perform(context)),
            hold_odom=launch_flag(hold_odom.perform(context)),
        )

    def build_nav(context, *args, **kwargs):
        generated = write_visual_nav_params(nav_params.perform(context))
        nav_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                package_file("nav2_bringup", "launch", "navigation_launch.py")
            ),
            launch_arguments={
                "namespace": "",
                "use_sim_time": "false",
                "autostart": autostart.perform(context),
                "params_file": generated,
                "use_composition": "False",
                "use_respawn": "False",
            }.items(),
        )
        return [nav_launch]

    def build_capture(context, *args, **kwargs):
        if not launch_flag(capture.perform(context)):
            return []
        return [capture_runtime_node(capture_dir.perform(context))]

    return LaunchDescription(
        [
            DeclareLaunchArgument("car_mode", default_value=""),
            DeclareLaunchArgument(
                "wheeltec_param_yaml",
                default_value=package_file("turn_on_wheeltec_robot", "config", "wheeltec_param.yaml"),
            ),
            DeclareLaunchArgument(
                "robot_model_yaml",
                default_value=package_file("turn_on_wheeltec_robot", "config", "robot_model.yaml"),
            ),
            DeclareLaunchArgument(
                "database_path",
                default_value=os.path.expanduser("~/.ros/vision_rtabmap_mapping.db"),
            ),
            DeclareLaunchArgument(
                "nav_params_file",
                default_value=package_file("wheeltec_nav2", "param", "wheeltec_params", "param_senior_akm.yaml"),
            ),
            DeclareLaunchArgument("start_base", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="false"),
            DeclareLaunchArgument("hold_odom", default_value="false"),
            DeclareLaunchArgument("capture", default_value="false"),
            DeclareLaunchArgument(
                "capture_dir",
                default_value=os.path.expanduser("~/vision_captures"),
            ),
            # D435C left-infrared optical centre in base_footprint, metres.
            # Empty uses launch_utils.DEFAULT_CAMERA_XYZ (CAD-derived).
            DeclareLaunchArgument("camera_xyz", default_value=""),
            *delayed_camera_stack(
                [
                    OpaqueFunction(function=build_model),
                    OpaqueFunction(function=build_runtime),
                    OpaqueFunction(function=build_nav),
                    OpaqueFunction(function=build_capture),
                ]
            ),
        ]
    )
