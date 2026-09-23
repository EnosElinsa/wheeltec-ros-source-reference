import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration

from launch_utils import (
    capture_runtime_node,
    delayed_camera_stack,
    launch_flag,
    model_actions,
    package_file,
    visual_runtime_actions,
)


def generate_launch_description():
    car_mode = LaunchConfiguration("car_mode")
    wheeltec_param = LaunchConfiguration("wheeltec_param_yaml")
    robot_model = LaunchConfiguration("robot_model_yaml")
    database = LaunchConfiguration("database_path")
    start_base = LaunchConfiguration("start_base")
    hold_odom = LaunchConfiguration("hold_odom")
    use_wheel_guess = LaunchConfiguration("use_wheel_guess")
    wipe_db = LaunchConfiguration("wipe_db")
    capture = LaunchConfiguration("capture")
    capture_dir = LaunchConfiguration("capture_dir")
    camera_xyz = LaunchConfiguration("camera_xyz")

    def build_model(context, *args, **kwargs):
        return model_actions(context, car_mode, wheeltec_param, robot_model, camera_xyz)

    def build_runtime(context, *args, **kwargs):
        return visual_runtime_actions(
            database.perform(context),
            localization=False,
            start_base=launch_flag(start_base.perform(context)),
            hold_odom=launch_flag(hold_odom.perform(context)),
            use_wheel_guess=launch_flag(use_wheel_guess.perform(context)),
            wipe_db=launch_flag(wipe_db.perform(context)),
        )

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
            DeclareLaunchArgument("start_base", default_value="false"),
            DeclareLaunchArgument("hold_odom", default_value="false"),
            DeclareLaunchArgument("use_wheel_guess", default_value="false"),
            DeclareLaunchArgument("wipe_db", default_value="true"),
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
                    OpaqueFunction(function=build_capture),
                ]
            ),
        ]
    )
