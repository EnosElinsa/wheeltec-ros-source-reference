"""Compose visual pipeline actions with explicit odometry and safety ownership."""
from wheeltec_vision_nav.camera_model import DEFAULT_CAMERA_XYZ, parse_rpy, parse_xyz, rewrite_camera_joint, write_camera_urdf
from wheeltec_vision_nav.launch_camera import model_actions, realsense_launch
from wheeltec_vision_nav.launch_common import ackermann_bt_xml, launch_flag, package_file
from wheeltec_vision_nav.launch_nodes import (
    capture_runtime_node, depth_obstacle_nodes, ekf_inputs_node, ekf_node,
    emitter_pair_node, hold_odom_tf_node, isolated_base_node, navigation_nodes,
    rgbd_sync_node, rtabmap_node, runtime_monitor_node, visual_odometry_node,
    vo_watchdog_node, wheel_odom_tf_node,
)
from wheeltec_vision_nav.nav_params import bundled_runtime_config
from wheeltec_vision_nav.nav_params import write_visual_nav_params as _write_visual_nav_params
from wheeltec_vision_nav.runtime_config import resolve_odom_source, safety_parameters


def camera_stack(extra_actions, stereo="false", camera_profile="848x480x30"):
    """Start sensor consumers together after leased preflight; no fixed delay."""
    return [realsense_launch(stereo=stereo, camera_profile=camera_profile),
            emitter_pair_node(), rgbd_sync_node(), *extra_actions]


def visual_runtime_actions(database_path, localization, start_base, hold_odom=False,
                           use_wheel_guess=False, wipe_db=False, odom_source="auto",
                           runtime=None, supervised_reverse=None):
    cfg = runtime if runtime is not None else bundled_runtime_config()
    source = resolve_odom_source(odom_source, start_base=start_base, hold_odom=hold_odom)
    perception = cfg["perception"]
    safe = safety_parameters(cfg, localization=localization, odom_source=source,
                             supervised_reverse=supervised_reverse)
    actions = [
        visual_odometry_node(use_wheel_guess=use_wheel_guess, use_imu=False,
                             max_features=perception["max_features"]),
        rtabmap_node(database_path, localization=localization, wipe_db=wipe_db,
                     odom_tf_linear_variance=perception["odom_tf_linear_variance"],
                     odom_tf_angular_variance=perception["odom_tf_angular_variance"]),
    ]
    # Wheel guessing is a prior only. Actuation always requires start_base.
    if start_base:
        actions.append(isolated_base_node(serial_port=cfg["robot"]["serial_port"]))
    if use_wheel_guess:
        actions.append(wheel_odom_tf_node())
    if source == "ekf":
        actions.extend([ekf_inputs_node(wheel_prior_used=use_wheel_guess, safety=safe), ekf_node()])
    elif source == "vo_only":
        actions.append(hold_odom_tf_node())
    actions.extend(depth_obstacle_nodes(decimation=perception["cloud_decimation"], safety=safe))
    actions.append(vo_watchdog_node(wheel_backup=safe["wheel_backup"], safety=safe))
    return actions


def write_visual_nav_params(source_path, *, supervised_reverse=False, runtime=None):
    return _write_visual_nav_params(source_path, ackermann_bt_xml(supervised_reverse),
        supervised_reverse=supervised_reverse, runtime=runtime,
        through_tree_path=ackermann_bt_xml(supervised_reverse, through_poses=True))
