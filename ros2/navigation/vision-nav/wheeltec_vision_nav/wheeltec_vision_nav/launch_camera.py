"""RealSense startup, camera cleanup, and robot-model launch actions."""

from pathlib import Path

import yaml
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from wheeltec_vision_nav.camera_model import parse_rpy, parse_xyz, write_camera_urdf
from wheeltec_vision_nav.launch_common import package_file


def realsense_launch(stereo="false", camera_profile="848x480x30"):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            package_file("realsense2_camera", "launch", "rs_launch.py")
        ),
        launch_arguments={
            "camera_namespace": "/",
            "camera_name": "camera",
            # This D435C currently reports zero Depth↔Color extrinsics, so
            # aligned RGB-D is empty. Left IR shares the depth optical center
            # (identity extrinsics) and is the mapping stream until RGB
            # calibration is restored.
            "enable_color": "false",
            "enable_depth": "true",
            "enable_infra1": "true",
            "enable_infra2": stereo,
            "enable_sync": "true",
            "align_depth.enable": "false",
            "pointcloud.enable": "false",
            # D435 native stereo size at 30 fps. The emitter interleave halves
            # the usable rate, so paired RGB-D arrives at ~15 Hz. If the Jetson
            # drops below ~20 Hz on the raw infra topic, fall back to
            # 640x480x30 using `ros2 topic hz`, not a one-time guess.
            "depth_module.depth_profile": camera_profile,
            "depth_module.infra_profile": camera_profile,
            # Humble rs_launch.py does not declare emitter_on_off. That key is
            # in realsense_d435c.yaml. The TF root is camera_link regardless.
            "config_file": package_file(
                "wheeltec_vision_nav", "config", "realsense_d435c.yaml"
            ),
            "spatial_filter.enable": "true",
            "spatial_filter.filter_magnitude": "2",
            "temporal_filter.enable": "true",
            "initial_reset": "false",
            "publish_tf": "true",
            "tf_publish_rate": "10.0",
            "wait_for_device_timeout": "30.0",
            "reconnect_timeout": "6.0",
        }.items(),
    )


def model_actions(context, car_mode_arg, wheeltec_param_arg, robot_model_arg, camera_xyz_arg=None, camera_rpy_arg=None):
    """Build the robot model and static TF without starting radar or chassis.

    The URDF camera_joint is rewritten so that camera_link (the RealSense TF
    root) sits at the D435C instead of the factory Orbbec position. The chain
    is base_footprint -> base_link -> camera_link -> camera_*_optical_frame.
    """
    param_path = Path(wheeltec_param_arg.perform(context))
    model_path = Path(robot_model_arg.perform(context))
    cfg = yaml.safe_load(param_path.read_text(encoding="utf-8"))
    model_cfg = yaml.safe_load(model_path.read_text(encoding="utf-8"))
    car_mode = car_mode_arg.perform(context) or cfg["car_mode"]
    if car_mode != "senior_akm":
        raise RuntimeError(f"Only the fixed senior_akm hardware is supported, got {car_mode!r}")
    if car_mode not in model_cfg["robot_model"]:
        raise RuntimeError(f"Unknown WHEELTEC car_mode: {car_mode}")
    values = model_cfg["robot_model"][car_mode]
    urdf_path = package_file("wheeltec_robot_urdf", "urdf", f"{car_mode}_robot.urdf")
    camera_xyz = parse_xyz(camera_xyz_arg.perform(context) if camera_xyz_arg is not None else "")
    camera_rpy = parse_rpy(camera_rpy_arg.perform(context) if camera_rpy_arg is not None else "")
    urdf_path = write_camera_urdf(urdf_path, camera_xyz, float(values["base_to_link"][2]), camera_rpy)

    actions = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="vision_robot_state_publisher",
            arguments=[urdf_path],
            output="screen",
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="vision_joint_state_publisher",
            parameters=[{"rate": 10.0}],
            output="screen",
        ),
    ]
    static_specs = [
        ("base_to_laser_vision", values["base_to_laser"], "laser"),
        ("base_to_link_vision", values["base_to_link"], "base_link"),
        ("base_to_gyro_vision", values["base_to_gyro"], "gyro_link"),
        ("base_to_radar_vision", values["base_to_radar"], "radar"),
    ]
    # The URDF owns base_link -> camera_link (rewritten above). Do not add a
    # second base_footprint -> camera_link publisher from the legacy launch.
    for name, transform, child in static_specs:
        actions.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name=name,
                arguments=[*map(str, transform), "base_footprint", child],
                output="screen",
            )
        )
    actions[0].vision_generated_files = [urdf_path]
    return actions
