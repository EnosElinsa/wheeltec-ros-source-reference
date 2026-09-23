"""Shared launch helpers for the isolated WHEELTEC visual path."""

import os
import re
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# D435C left-infrared optical centre in base_footprint (m). Derived from the
# sensor-mount-v1 CAD: top plate upper face ~0.104 m above base_footprint,
# bracket M3 holes 28 mm above the plate, camera front face 9.5 mm behind the
# plate front edge (~0.197 m), left imager 17.5 mm left of the centreline.
# The RealSense driver roots its TF tree at camera_link (= depth/infra1
# sensor origin), so this is the pose that base_link -> camera_link must give.
DEFAULT_CAMERA_XYZ = (0.187, 0.0175, 0.132)


def package_file(package: str, *parts: str) -> str:
    return str(Path(get_package_share_directory(package), *parts))


def launch_flag(value: str) -> bool:
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def parse_xyz(value, default=DEFAULT_CAMERA_XYZ) -> tuple[float, float, float]:
    """Parse 'x y z' or 'x,y,z' (metres). Empty string -> default."""
    text = str(value or "").strip()
    if not text:
        return tuple(default)
    parts = [p for p in re.split(r"[\s,]+", text) if p]
    if len(parts) != 3:
        raise ValueError(f"camera_xyz needs three numbers, got {value!r}")
    return tuple(float(p) for p in parts)


def rewrite_camera_joint(urdf_text: str, xyz_in_base_link: tuple[float, float, float]) -> str:
    """Move the URDF camera_joint origin so camera_link sits at the D435C.

    The factory senior_akm URDF places camera_link at the former Orbbec camera.
    Only the <origin xyz> of the joint named camera_joint is changed; the
    parent (base_link), child (camera_link) and rpy stay as authored.
    """
    pattern = re.compile(
        r'(<joint\s+name="camera_joint"[^>]*>\s*<origin\s+xyz=")([^"]*)(")',
        re.DOTALL,
    )
    new_xyz = " ".join(f"{v:.6f}" for v in xyz_in_base_link)
    text, count = pattern.subn(lambda m: m.group(1) + new_xyz + m.group(3), urdf_text, count=1)
    if count != 1:
        raise RuntimeError("camera_joint <origin xyz> not found in URDF")
    return text


def write_camera_urdf(urdf_path: str, camera_xyz_in_footprint, base_to_link_z: float) -> str:
    """Write a temp URDF whose camera_link is at camera_xyz (base_footprint frame)."""
    x, y, z = camera_xyz_in_footprint
    text = Path(urdf_path).read_text(encoding="utf-8")
    text = rewrite_camera_joint(text, (x, y, z - base_to_link_z))
    handle, generated = tempfile.mkstemp(prefix="wheeltec_vision_nav_", suffix=".urdf")
    os.close(handle)
    Path(generated).write_text(text, encoding="utf-8")
    return generated


def rtabmap_visual_config() -> str:
    return package_file("wheeltec_vision_nav", "config", "rtabmap_visual.yaml")


def ekf_visual_config() -> str:
    return package_file("wheeltec_vision_nav", "config", "ekf_visual.yaml")


def ackermann_bt_xml() -> str:
    return package_file(
        "wheeltec_vision_nav",
        "behavior_trees",
        "navigate_w_replanning_ackermann.xml",
    )


def realsense_launch():
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
            "enable_infra2": "false",
            "enable_sync": "true",
            "align_depth.enable": "false",
            "pointcloud.enable": "false",
            # D435 native stereo size at 30 fps. The emitter interleave halves
            # the usable rate, so paired RGB-D arrives at ~15 Hz. If the Jetson
            # drops below ~20 Hz on the raw infra topic, fall back to
            # 640x480x30 using `ros2 topic hz`, not a one-time guess.
            "depth_module.depth_profile": "848x480x30",
            "depth_module.infra_profile": "848x480x30",
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


def model_actions(context, car_mode_arg, wheeltec_param_arg, robot_model_arg, camera_xyz_arg=None):
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
    if car_mode not in model_cfg["robot_model"]:
        raise RuntimeError(f"Unknown WHEELTEC car_mode: {car_mode}")
    values = model_cfg["robot_model"][car_mode]
    urdf_path = package_file("wheeltec_robot_urdf", "urdf", f"{car_mode}_robot.urdf")
    camera_xyz = parse_xyz(camera_xyz_arg.perform(context) if camera_xyz_arg is not None else "")
    urdf_path = write_camera_urdf(urdf_path, camera_xyz, float(values["base_to_link"][2]))

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
    return actions


def camera_cleanup_action():
    """End leftover camera and vision nodes so USB and /odom have a single owner.

    Character classes keep ``pkill -f`` from matching this cleanup shell.
    """
    return ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            "pkill -f '/opt/ros/humble/lib/realsense2_camera/realsense2_camera_nod[e]' || true; "
            "pkill -f '/opt/ros/humble/lib/rtabmap_odom/rgbd_odometr[y]' || true; "
            "pkill -f '/opt/ros/humble/lib/rtabmap_slam/rtabma[p]' || true; "
            "pkill -f '/opt/ros/humble/lib/rtabmap_sync/rgbd_syn[c]' || true; "
            "pkill -f 'lib/wheeltec_vision_nav/hold_odom_t[f]' || true; "
            "pkill -f 'lib/wheeltec_vision_nav/emitter_pai[r]' || true; "
            "pkill -f 'lib/wheeltec_vision_nav/vo_watchdo[g]' || true; "
            "pkill -f 'lib/wheeltec_vision_nav/wheel_odom_t[f]' || true; "
            "pkill -f 'lib/wheeltec_vision_nav/capture_runtim[e]' || true; "
            "pkill -f '/opt/ros/humble/lib/robot_localization/ekf_nod[e]' || true; "
            "pkill -f 'lib/wheeltec_vision_nav/ekf_input[s]' || true; "
            "pkill -f '/opt/ros/humble/lib/rtabmap_util/point_cloud_xy[z]' || true; "
            "pkill -f '/opt/ros/humble/lib/rtabmap_util/obstacles_detectio[n]' || true; "
            "pkill -f '__node:=vision_joint_state_publishe[r]' || true; "
            "pkill -f '__node:=vision_imu_filte[r]' || true; "
            "pkill -f '__node:=vision_robot_state_publishe[r]' || true; "
            "pkill -f '__node:=base_to_laser_visio[n]' || true; "
            "pkill -f '__node:=base_to_link_visio[n]' || true; "
            "pkill -f '__node:=base_to_gyro_visio[n]' || true; "
            "pkill -f '__node:=base_to_radar_visio[n]' || true; "
            "pkill -f '__node:=vision_wheeltec_robo[t]' || true; "
            "pkill -f 'lib/nav2_controller/controller_serve[r]' || true; "
            "pkill -f 'lib/nav2_planner/planner_serve[r]' || true; "
            "pkill -f 'lib/nav2_bt_navigator/bt_navigato[r]' || true; "
            "pkill -f 'lib/nav2_behaviors/behavior_serve[r]' || true; "
            "pkill -f 'lib/nav2_smoother/smoother_serve[r]' || true; "
            "pkill -f 'lib/nav2_waypoint_follower/waypoint_followe[r]' || true; "
            "pkill -f '__node:=lifecycle_manager_navigatio[n]' || true; "
            "true",
        ],
        name="vision_camera_cleanup",
        output="screen",
    )


def delayed_camera_stack(extra_actions):
    """Start D435C after leftover nodes exit, then pairing, sync, and extras."""
    return [
        camera_cleanup_action(),
        TimerAction(
            period=2.0,
            actions=[
                realsense_launch(),
                emitter_pair_node(),
                rgbd_sync_node(),
                *extra_actions,
            ],
        ),
    ]


def emitter_pair_node():
    return Node(
        package="wheeltec_vision_nav",
        executable="emitter_pair",
        name="vision_emitter_pair",
        output="screen",
        parameters=[
            {
                "max_interval": 0.12,
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


def visual_odometry_node(use_wheel_guess=False, use_imu=False):
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


def rtabmap_node(database_path, localization=False, wipe_db=True):
    params = {
        "frame_id": "base_footprint",
        "odom_frame_id": "odom",
        "odom_tf_linear_variance": ODOM_TF_LINEAR_VARIANCE,
        "odom_tf_angular_variance": ODOM_TF_ANGULAR_VARIANCE,
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
    arguments = ["-d"] if (not localization and wipe_db) else []
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


def isolated_base_node(condition=None, connect_base_footprint=True, cmd_vel_topic="cmd_vel"):
    """Optional actuator node. Wheel odometry is remapped to /wheel/odom.

    The topic uses wheel_odom as parent and base_footprint as child. TF is
    published separately as the inverse (base_footprint→wheel_odom) so
    base_footprint keeps a single parent.
    Isolated mode keeps a disconnected child name when guess is not used.
    """
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
                "usart_port_name": "/dev/wheeltec_controller",
                "serial_baud_rate": 115200,
                "robot_frame_id": robot_frame_id,
                "odom_frame_id": odom_frame_id,
                "gyro_frame_id": "gyro_link",
                "cmd_vel": cmd_vel_topic,
                "akm_cmd_vel": "none",
                "product_number": 0,
                "car_mode": "senior_akm",
                "ranger_avoid_flag": False,
                "ultrasonic_avoid": False,
            }
        ],
        "remappings": [
            ("odom", "/wheel/odom"),
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


def ekf_inputs_node():
    """Gyro without bias on /vision/imu, VO velocity with realistic variance on /vision/odom_vo."""
    return Node(
        package="wheeltec_vision_nav",
        executable="ekf_inputs",
        name="vision_ekf_inputs",
        output="screen",
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


PAIRED_DEPTH_TOPIC = "/vision/paired/depth/image_rect_raw"
PAIRED_DEPTH_INFO_TOPIC = "/vision/paired/depth/camera_info"
DEPTH_CLOUD_TOPIC = "/vision/depth_cloud"
OBSTACLES_TOPIC = "/vision/obstacles"


def depth_obstacle_nodes():
    """Obstacle points for the local costmap straight from each paired depth image.

    point_cloud_xyz turns the projected (emitter-on) depth into a decimated,
    voxel-filtered cloud. obstacles_detection splits it into ground and
    obstacles in base_footprint with the same height limits as the map grid.
    """
    cloud = Node(
        package="rtabmap_util",
        executable="point_cloud_xyz",
        name="vision_depth_cloud",
        output="screen",
        parameters=[
            {
                "decimation": 4,
                "voxel_size": 0.05,
                "min_depth": 0.20,
                "max_depth": 3.0,
                "approx_sync": False,
                "qos": 2,
                "qos_camera_info": 2,
            }
        ],
        remappings=[
            ("depth/image", PAIRED_DEPTH_TOPIC),
            ("depth/camera_info", PAIRED_DEPTH_INFO_TOPIC),
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
    return [cloud, obstacles]


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


def vo_watchdog_node(condition=None, wheel_backup=False):
    kwargs = {
        "package": "wheeltec_vision_nav",
        "executable": "vo_watchdog",
        "name": "vision_vo_watchdog",
        "output": "screen",
        "parameters": [
            {
                "cmd_vel_in": "/cmd_vel",
                "cmd_vel_out": "/cmd_vel_safe",
                "status_topic": "/vision/vo_status",
                "min_inliers": 10,
                "consecutive_bad": 5,
                # With the chassis running, /odom keeps flowing from wheel
                # odometry while VO is lost, so a VO loss alone does not stop
                # the car. The car stops only when the map correction
                # (map->odom) has not been refreshed for this long.
                "wheel_backup": bool(wheel_backup),
                "wheel_fresh_s": 0.3,
                "map_stale_s": 5.0,
            }
        ],
    }
    if condition is not None:
        kwargs["condition"] = condition
    return Node(**kwargs)


def visual_runtime_actions(
    database_path,
    localization,
    start_base,
    hold_odom,
    use_wheel_guess=None,
    wipe_db=True,
):
    """Odometry, RTAB-Map, chassis and the single owner of odom -> base_footprint.

    With the chassis running, the EKF fuses wheel speed, gyro yaw rate and VO
    velocity and owns odom -> base_footprint. Camera only, hold_odom_tf relays
    the visual pose instead.
    """
    _ = hold_odom
    if use_wheel_guess is None:
        use_wheel_guess = False
    start_chassis = bool(start_base) or bool(use_wheel_guess)
    use_wheel_guess = start_chassis
    actions = [
        visual_odometry_node(use_wheel_guess=use_wheel_guess, use_imu=False),
        rtabmap_node(database_path, localization=localization, wipe_db=wipe_db),
    ]
    if start_chassis:
        cmd_vel_topic = "cmd_vel_safe" if localization else "cmd_vel"
        actions.append(
            isolated_base_node(connect_base_footprint=True, cmd_vel_topic=cmd_vel_topic)
        )
        actions.append(wheel_odom_tf_node())
        actions.append(ekf_inputs_node())
        actions.append(ekf_node())
    else:
        actions.append(hold_odom_tf_node())
    if localization:
        actions.extend(depth_obstacle_nodes())
        actions.append(vo_watchdog_node(wheel_backup=start_chassis))
    return actions


# Costmap inflation shared by the local costmap and the MPPI obstacle critic.
# The two must agree: MPPI reads the local costmap and turns cost into a
# repulsion term, so a critic tuned for a different inflation radius stops the
# car well before the footprint touches anything.
INFLATION_RADIUS = 0.25
COST_SCALING_FACTOR = 3.0
# Camera-only mapping is sparse and the D435 sees 4 m at most. Keep the car
# slow so the ~15 Hz RGB-D stream and the 1 Hz replanner can follow it.
VX_MAX = 0.3
VX_MIN = -0.15
WZ_MAX = 1.0

# The Jetson also runs the camera, VO and RTAB-Map; at 20 Hz the controller
# missed its rate. Humble MPPI requires the control period to equal model_dt,
# so the horizon keeps 2.8 s with 28 steps of 0.1 s.
CONTROLLER_FREQUENCY = 10.0
MPPI_MODEL_DT = 0.1
MPPI_TIME_STEPS = 28
MPPI_BATCH_SIZE = 1000
# Ackermann can reverse, but reversing is the exception: weight forward motion
# in the controller and penalise reverse segments in the planner.
PREFER_FORWARD_WEIGHT = 5.0
SMAC_REVERSE_PENALTY = 3.0

DEPTH_CLOUD_SOURCE = {
    "topic": OBSTACLES_TOPIC,
    "data_type": "PointCloud2",
    "clearing": True,
    "marking": True,
    "min_obstacle_height": 0.05,
    "max_obstacle_height": 1.8,
    "obstacle_min_range": 0.20,
    "obstacle_max_range": 3.0,
    "raytrace_min_range": 0.20,
    "raytrace_max_range": 3.0,
    # Obstacle points arrive with every paired depth image (~15 Hz).
    "observation_persistence": 0.0,
}


def _tune_mppi(follow: dict) -> None:
    """Align the MPPI controller with the visual costmaps and slow the car down."""
    follow["vx_max"] = VX_MAX
    follow["vx_min"] = VX_MIN
    follow["wz_max"] = WZ_MAX
    follow["model_dt"] = MPPI_MODEL_DT
    follow["time_steps"] = MPPI_TIME_STEPS
    follow["batch_size"] = MPPI_BATCH_SIZE
    obstacles = follow.setdefault("ObstaclesCritic", {})
    if isinstance(obstacles, dict):
        obstacles["inflation_radius"] = INFLATION_RADIUS
        obstacles["cost_scaling_factor"] = COST_SCALING_FACTOR
        obstacles["critical_weight"] = 20.0
        obstacles["repulsion_weight"] = 1.5
        obstacles["collision_margin_distance"] = 0.05
        obstacles["consider_footprint"] = True
    forward = follow.setdefault("PreferForwardCritic", {})
    if isinstance(forward, dict):
        forward["cost_weight"] = PREFER_FORWARD_WEIGHT


def write_visual_nav_params(source_path: str) -> str:
    cfg = yaml.safe_load(Path(source_path).read_text(encoding="utf-8"))
    for key in ("amcl", "amcl_map_client", "amcl_rclcpp_node", "map_server", "map_saver"):
        cfg.pop(key, None)

    bt = cfg.setdefault("bt_navigator", {}).setdefault("ros__parameters", {})
    bt["odom_topic"] = "/odom"
    bt["default_nav_to_pose_bt_xml"] = ackermann_bt_xml()

    controller = cfg.setdefault("controller_server", {}).setdefault("ros__parameters", {})
    controller["odom_topic"] = "/odom"
    controller["controller_frequency"] = CONTROLLER_FREQUENCY
    follow = controller.setdefault("FollowPath", {})
    if isinstance(follow, dict):
        if "MPPI" in str(follow.get("plugin", "")):
            _tune_mppi(follow)
        else:
            # Ackermann often needs reverse to clear a blocked heading.
            try:
                current = float(follow.get("vx_min", 0.0))
            except (TypeError, ValueError):
                current = 0.0
            if current >= 0.0:
                follow["vx_min"] = VX_MIN
    # A camera-only map leaves the car creeping around unknown cells. Give the
    # progress checker more room before it declares the controller stuck.
    progress = controller.setdefault("progress_checker", {})
    if isinstance(progress, dict):
        progress["required_movement_radius"] = 0.25
        progress["movement_time_allowance"] = 20.0

    # The global planner runs on a map with large unknown regions next to the
    # walls. Let it cross unknown cells; the inflation layer still keeps the
    # path away from marked obstacles.
    planner = cfg.setdefault("planner_server", {}).setdefault("ros__parameters", {})
    grid_based = planner.get("GridBased")
    if isinstance(grid_based, dict):
        grid_based["allow_unknown"] = True
        if "Smac" in str(grid_based.get("plugin", "")):
            grid_based["reverse_penalty"] = SMAC_REVERSE_PENALTY

    static_layer_cfg = {
        "plugin": "nav2_costmap_2d::StaticLayer",
        "map_topic": "/map",
        "subscribe_to_updates": True,
        "track_unknown_space": True,
        "map_subscribe_transient_local": True,
    }

    for costmap_name in ("local_costmap", "global_costmap"):
        block = cfg.get(costmap_name, {}).get(costmap_name, {}).get("ros__parameters", {})
        if not block:
            continue
        block["robot_base_frame"] = "base_footprint"
        inflation = dict(block.get("inflation_layer") or {})
        inflation.setdefault("plugin", "nav2_costmap_2d::InflationLayer")
        if costmap_name == "local_costmap":
            # Local costmap: only what the camera currently sees, in the smooth
            # odom frame. No static layer: the static /map carries unknown
            # cells next to every wall and MPPI treats unknown as lethal, which
            # is what stopped the car short of obstacles.
            block["global_frame"] = "odom"
            block["rolling_window"] = True
            block["track_unknown_space"] = False
            block.pop("static_layer", None)
            block["plugins"] = ["voxel_layer", "inflation_layer"]
            voxel = dict(block.get("voxel_layer") or {})
            voxel.setdefault("plugin", "nav2_costmap_2d::VoxelLayer")
            voxel["observation_sources"] = "depth_points"
            voxel.pop("scan", None)
            voxel.pop("scan2", None)
            voxel["depth_points"] = dict(DEPTH_CLOUD_SOURCE)
            block["voxel_layer"] = voxel
            inflation["inflation_radius"] = INFLATION_RADIUS
            inflation["cost_scaling_factor"] = COST_SCALING_FACTOR
        else:
            block["global_frame"] = "map"
            block["track_unknown_space"] = True
            merged_static = dict(block.get("static_layer") or {})
            merged_static.update(static_layer_cfg)
            block["static_layer"] = merged_static
            plugins = [p for p in list(block.get("plugins") or []) if p not in ("voxel_layer", "obstacle_layer")]
            if "static_layer" not in plugins:
                plugins.insert(0, "static_layer")
            if "inflation_layer" not in plugins:
                plugins.append("inflation_layer")
            block["plugins"] = plugins
            block.pop("obstacle_layer", None)
            block.pop("voxel_layer", None)
            inflation.setdefault("inflation_radius", INFLATION_RADIUS)
        block["inflation_layer"] = inflation

    handle, generated = tempfile.mkstemp(prefix="wheeltec_vision_nav_", suffix=".yaml")
    os.close(handle)
    Path(generated).write_text(yaml.safe_dump(cfg, allow_unicode=True, sort_keys=False), encoding="utf-8")
    return generated
