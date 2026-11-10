"""Node configurations for pairing, odometry, localization, and navigation."""

from launch_ros.actions import Node

from wheeltec_vision_nav.launch_common import (
    DEPTH_CLOUD_TOPIC,
    OBSTACLES_TOPIC,
    RAW_DEPTH_INFO_TOPIC,
    RAW_DEPTH_TOPIC,
    ekf_visual_config,
    rtabmap_visual_config,
)


def emitter_pair_node():
    return Node(
        package="wheeltec_vision_nav",
        executable="emitter_pair",
        name="vision_emitter_pair",
        output="screen",
        parameters=[
            {
                "max_interval": 0.05,
                "min_interval": 0.02,
                "buffer_size": 12,
                "infra_topic": "/camera/infra1/image_rect_raw",
                "infra_info_topic": "/camera/infra1/camera_info",
                "depth_topic": "/camera/depth/image_rect_raw",
                "depth_info_topic": "/camera/depth/camera_info",
                "infra_out_topic": "/vision/paired/infra1/image_rect_raw",
                "infra_info_out_topic": "/vision/paired/infra1/camera_info",
                "depth_out_topic": "/vision/paired/depth/image_rect_raw",
                "depth_info_out_topic": "/vision/paired/depth/camera_info",
            }
        ],
    )


def rgbd_sync_node():
    return Node(
        package="rtabmap_sync",
        executable="rgbd_sync",
        name="vision_rgbd_sync",
        output="screen",
        parameters=[
            {
                "approx_sync": True,
                # Pairing node already matches opposite emitter frames. Keep a
                # 1–2 frame window at 15 Hz rather than a quarter-second slop.
                "approx_sync_max_interval": 0.10,
                "qos": 2,
            }
        ],
        remappings=[
            ("rgb/image", "/vision/paired/infra1/image_rect_raw"),
            ("rgb/camera_info", "/vision/paired/infra1/camera_info"),
            ("depth/image", "/vision/paired/depth/image_rect_raw"),
            ("rgbd_image", "/rtabmap/rgbd_image"),
        ],
    )


def visual_odometry_node(use_wheel_guess=False, use_imu=False, max_features=1500):
    params = {
        "frame_id": "base_footprint",
        "odom_frame_id": "odom",
        # Humble rgbd_odometry with guess_frame_id may publish odom→wheel_odom
        # instead of odom→base_footprint. The robot frame gets its odom parent
        # from the EKF (chassis running) or hold_odom_tf (camera only).
        "publish_tf": False,
        "subscribe_rgbd": True,
        "approx_sync": False,
        "qos": 2,
        "wait_for_transform": 0.5,
        "Vis/MaxFeatures": str(max_features),
    }
    if use_wheel_guess:
        # wheel_odom_tf publishes base_footprint -> wheel_odom (inverse wheel
        # pose). lookup(wheel_odom, base_footprint) therefore returns the wheel
        # pose, which rgbd_odometry uses as the motion prior between frames
        # and as the pose to continue from after a reset.
        params["guess_frame_id"] = "wheel_odom"
        params["guess_min_translation"] = 0.0
        params["guess_min_rotation"] = 0.0
    if use_imu:
        params["imu_topic"] = "/imu/data"
        params["wait_imu_to_init"] = False
    return Node(
        package="rtabmap_odom",
        executable="rgbd_odometry",
        name="vision_rgbd_odometry",
        output="screen",
        parameters=[rtabmap_visual_config(), params],
        remappings=[
            ("rgbd_image", "/rtabmap/rgbd_image"),
            ("odom", "/odom_vo"),
        ],
    )


# Variance given to every odometry link between two map nodes. RTAB-Map reads
# odometry from TF (odom -> base_footprint) instead of the /odom covariance:
# the chassis reports a yaw variance of 1e3 rad^2, which left the pose graph
# with no heading constraint and let a few wrong loop closures twist the map.
ODOM_TF_LINEAR_VARIANCE = 0.0005
ODOM_TF_ANGULAR_VARIANCE = 0.0005


def rtabmap_node(database_path, localization=False, wipe_db=False,
                 odom_tf_linear_variance=ODOM_TF_LINEAR_VARIANCE,
                 odom_tf_angular_variance=ODOM_TF_ANGULAR_VARIANCE):
    if wipe_db:
        raise ValueError("Database wipe requires the leased startup backup; RTAB-Map never receives -d")
    params = {
        "frame_id": "base_footprint",
        "odom_frame_id": "odom",
        "odom_tf_linear_variance": odom_tf_linear_variance,
        "odom_tf_angular_variance": odom_tf_angular_variance,
        "subscribe_rgbd": True,
        "subscribe_scan": False,
        "subscribe_depth": False,
        "publish_tf": True,
        "qos_image": 2,
        "qos_imu": 2,
        "qos_odom": 2,
        "wait_for_transform": 0.5,
        "database_path": database_path,
        "Mem/IncrementalMemory": "true" if not localization else "false",
        "Mem/InitWMWithAllNodes": "false" if not localization else "true",
    }
    arguments = []
    return Node(
        package="rtabmap_slam",
        executable="rtabmap",
        name="vision_rtabmap",
        output="screen",
        arguments=arguments,
        parameters=[rtabmap_visual_config(), params],
        remappings=[
            ("rgbd_image", "/rtabmap/rgbd_image"),
            ("odom", "/odom"),
            ("initialpose", "/initialpose"),
        ],
    )


def isolated_base_node(condition=None, connect_base_footprint=True, cmd_vel_topic="/cmd_vel_safe",
                       serial_port="/dev/wheeltec_controller"):
    """Optional actuator node. Wheel odometry is remapped to /wheel/odom.

    The topic uses wheel_odom as parent and base_footprint as child. TF is
    published separately as the inverse (base_footprint→wheel_odom) so
    base_footprint keeps a single parent.
    Isolated mode keeps a disconnected child name when guess is not used.
    """
    if cmd_vel_topic != "/cmd_vel_safe":
        raise ValueError("Actuation requires the gated /cmd_vel_safe topic")
    if connect_base_footprint:
        robot_frame_id = "base_footprint"
        odom_frame_id = "wheel_odom"
    else:
        robot_frame_id = "wheel_base_footprint"
        odom_frame_id = "wheel_odom"
    kwargs = {
        "package": "turn_on_wheeltec_robot",
        "executable": "wheeltec_robot_node",
        "name": "vision_wheeltec_robot",
        "output": "screen",
        "parameters": [
            {
                "usart_port_name": serial_port,
                "serial_baud_rate": 115200,
                "robot_frame_id": robot_frame_id,
                "odom_frame_id": odom_frame_id,
                "gyro_frame_id": "gyro_link",
                "cmd_vel": cmd_vel_topic,
                "cmd_vel_timeout_s": 0.3,
                "akm_cmd_vel": "none",
                "product_number": 0,
                "car_mode": "senior_akm",
                "ranger_avoid_flag": False,
                "ultrasonic_avoid": False,
            }
        ],
        "remappings": [
            ("odom", "/wheel/odom"),
            ("mobile_base/sensors/imu_data", "/imu/data_raw"),
            # The chassis node subscribes to "/cmd_vel" in source and does
            # not read the cmd_vel parameter. Remap so navigation can feed
            # /cmd_vel_safe from the watchdog.
            ("/cmd_vel", cmd_vel_topic if str(cmd_vel_topic).startswith("/") else f"/{cmd_vel_topic}"),
        ],
    }
    if condition is not None:
        kwargs["condition"] = condition
    return Node(**kwargs)


def wheel_odom_tf_node(condition=None):
    """Publish inverse TF from /wheel/odom. The chassis node does not broadcast TF."""
    kwargs = {
        "package": "wheeltec_vision_nav",
        "executable": "wheel_odom_tf",
        "name": "vision_wheel_odom_tf",
        "output": "screen",
        "parameters": [
            {
                "odom_topic": "/wheel/odom",
                "parent_frame": "base_footprint",
                "child_frame": "wheel_odom",
            }
        ],
    }
    if condition is not None:
        kwargs["condition"] = condition
    return Node(**kwargs)


def ekf_inputs_node(wheel_prior_used=False, safety=None):
    """Gyro without bias on /vision/imu, VO velocity with realistic variance on /vision/odom_vo."""
    return Node(
        package="wheeltec_vision_nav",
        executable="ekf_inputs",
        name="vision_ekf_inputs",
        output="screen",
        parameters=[{"wheel_prior_used": bool(wheel_prior_used), **{
            target: safety[source] for source, target in (
                ("min_inliers", "min_inliers"), ("vo_timeout_s", "quality_timeout_s"),
                ("wheel_fresh_s", "wheel_timeout_s")) if safety and source in safety}}],
    )


def ekf_node(condition=None):
    """Fuse wheel speed, gyro yaw rate and VO velocity into /odom and odom -> base_footprint."""
    kwargs = {
        "package": "robot_localization",
        "executable": "ekf_node",
        "name": "vision_ekf",
        "output": "screen",
        "parameters": [ekf_visual_config()],
        "remappings": [("odometry/filtered", "/odom")],
    }
    if condition is not None:
        kwargs["condition"] = condition
    return Node(**kwargs)


def depth_obstacle_nodes(decimation=4, safety=None):
    """Obstacle points from raw depth, independent of paired VO image health.

    The validity gate forwards usable raw depth with its exact CameraInfo.
    point_cloud_xyz turns that depth into a decimated,
    voxel-filtered cloud. obstacles_detection splits it into ground and
    obstacles in base_footprint with the same height limits as the map grid.
    """
    gate = Node(
        package="wheeltec_vision_nav",
        executable="depth_validity_gate",
        name="vision_depth_validity",
        output="screen",
        parameters=[{key: value for key, value in (safety or {}).items()
                     if key in ("sensor_timeout_s", "future_tolerance_s")}],
        remappings=[
            ("depth/image", RAW_DEPTH_TOPIC),
            ("depth/camera_info", RAW_DEPTH_INFO_TOPIC),
            ("depth_valid/image", "/vision/valid_depth/image"),
            ("depth_valid/camera_info", "/vision/valid_depth/camera_info"),
        ],
    )
    cloud = Node(
        package="rtabmap_util",
        executable="point_cloud_xyz",
        name="vision_depth_cloud",
        output="screen",
        parameters=[
            {
                "decimation": decimation,
                "voxel_size": 0.05,
                "min_depth": 0.20,
                "max_depth": 3.0,
                "approx_sync": False,
                "qos": 2,
                "qos_camera_info": 2,
            }
        ],
        remappings=[
            ("depth/image", "/vision/valid_depth/image"),
            ("depth/camera_info", "/vision/valid_depth/camera_info"),
            ("cloud", DEPTH_CLOUD_TOPIC),
        ],
    )
    obstacles = Node(
        package="rtabmap_util",
        executable="obstacles_detection",
        name="vision_obstacles_detection",
        output="screen",
        parameters=[
            {
                "frame_id": "base_footprint",
                "wait_for_transform": 0.2,
                "Grid/NormalsSegmentation": "true",
                "Grid/MaxGroundAngle": "30",
                "Grid/MaxGroundHeight": "0.05",
                "Grid/MaxObstacleHeight": "1.80",
                "Grid/MinClusterSize": "5",
            }
        ],
        remappings=[
            ("cloud", DEPTH_CLOUD_TOPIC),
            ("obstacles", OBSTACLES_TOPIC),
            ("ground", "/vision/ground"),
        ],
    )
    return [gate, cloud, obstacles]


def hold_odom_tf_node(condition=None):
    kwargs = {
        "package": "wheeltec_vision_nav",
        "executable": "hold_odom_tf",
        "name": "vision_hold_odom_tf",
        "output": "screen",
    }
    if condition is not None:
        kwargs["condition"] = condition
    return Node(**kwargs)


def capture_runtime_node(output_dir: str):
    return Node(
        package="wheeltec_vision_nav",
        executable="capture_runtime",
        name="vision_capture_runtime",
        output="screen",
        parameters=[
            {
                "output_dir": output_dir,
                "max_images": 8,
                "image_period": 5.0,
            }
        ],
    )


def vo_watchdog_node(condition=None, wheel_backup=False, safety=None):
    kwargs = {
        "package": "wheeltec_vision_nav",
        "executable": "vo_watchdog",
        "name": "vision_vo_watchdog",
        "output": "screen",
        "parameters": [
            {
                "cmd_vel_in": "/cmd_vel",
                "cmd_vel_out": "/cmd_vel_safe",
                "wheel_backup": bool(wheel_backup),
                **(safety or {}),
            }
        ],
    }
    if condition is not None:
        kwargs["condition"] = condition
    return Node(**kwargs)


def runtime_monitor_node(output_dir, runtime_config, nav_params_file, database_path):
    import os
    return Node(package="wheeltec_vision_nav", executable="runtime_monitor",
                name="vision_runtime_monitor", output="screen", parameters=[{
                    "output_dir": output_dir, "resource_root_pid": os.getpid(),
                    "runtime_config": runtime_config, "nav_params_file": nav_params_file,
                    "database_path": database_path,
                }])


def navigation_nodes(params_file, autostart=False):
    """Explicit Humble Nav2 processes with a scoped, single command chain."""
    specs = [
        ("nav2_controller", "controller_server"),
        ("nav2_smoother", "smoother_server"),
        ("nav2_planner", "planner_server"),
        ("nav2_behaviors", "behavior_server"),
        ("nav2_bt_navigator", "bt_navigator"),
        ("nav2_waypoint_follower", "waypoint_follower"),
        ("nav2_velocity_smoother", "velocity_smoother"),
    ]
    actions = []
    for package, name in specs:
        remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]
        if name in ("controller_server", "behavior_server"):
            remappings.append(("cmd_vel", "/cmd_vel_nav"))
        elif name == "velocity_smoother":
            remappings.extend([("cmd_vel", "/cmd_vel_nav"), ("cmd_vel_smoothed", "/cmd_vel")])
        actions.append(Node(package=package, executable=name, name=name, output="screen",
                            parameters=[params_file], remappings=remappings))
    actions.append(Node(package="nav2_lifecycle_manager", executable="lifecycle_manager",
                        name="lifecycle_manager_navigation", output="screen", parameters=[{
                            "use_sim_time": False, "autostart": bool(autostart),
                            "node_names": [name for _, name in specs],
                        }]))
    return actions
