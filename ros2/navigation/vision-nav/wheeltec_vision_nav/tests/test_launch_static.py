from pathlib import Path


ROOT = Path(__file__).parents[1]


def test_visual_launches_do_not_start_radar_or_livox():
    for path in (
        ROOT / "launch" / "vision_mapping.launch.py",
        ROOT / "launch" / "vision_navigation.launch.py",
        ROOT / "launch" / "baseline_record.launch.py",
        ROOT / "launch" / "capture_runtime.launch.py",
    ):
        text = path.read_text(encoding="utf-8").lower()
        assert "wheeltec_lidar" not in text, path
        assert "wheeltec_radar" not in text, path
        assert "livox" not in text, path
        assert "lslidar" not in text, path


def test_visual_launches_keep_actuation_opt_in():
    for path in (ROOT / "launch" / "vision_mapping.launch.py", ROOT / "launch" / "vision_navigation.launch.py"):
        text = path.read_text(encoding="utf-8")
        assert 'DeclareLaunchArgument("start_base", default_value="false")' in text
        assert 'DeclareLaunchArgument("autostart", default_value="false")' in text or path.name == "vision_mapping.launch.py"
        assert 'DeclareLaunchArgument("hold_odom", default_value="false")' in text


def test_rgbd_uses_left_ir_and_raw_depth_until_color_extrinsics_exist():
    helpers = (ROOT / "wheeltec_vision_nav" / "launch_utils.py").read_text(encoding="utf-8")
    navigation = (ROOT / "launch" / "vision_navigation.launch.py").read_text(encoding="utf-8")
    package_xml = (ROOT / "package.xml").read_text(encoding="utf-8")
    setup = (ROOT / "setup.py").read_text(encoding="utf-8")
    assert "/camera/infra1/image_rect_raw" in helpers
    assert "/camera/depth/image_rect_raw" in helpers
    assert "/camera/aligned_depth_to_color/image_raw" not in helpers
    assert '"enable_infra1": "true"' in helpers
    assert '"enable_color": "false"' in helpers
    assert '"align_depth.enable": "false"' in helpers
    assert '"pointcloud.enable": "false"' in helpers
    assert "stereo_odometry" not in helpers
    assert 'package_file("wheeltec_nav2"' in navigation
    assert "<exec_depend>wheeltec_nav2</exec_depend>" in package_xml
    assert "hold_odom_tf = wheeltec_vision_nav.hold_odom_tf:main" in setup
    assert "wheel_odom_tf = wheeltec_vision_nav.wheel_odom_tf:main" in setup
    assert "check_baseline = wheeltec_vision_nav.check_baseline:main" in setup
    assert "emitter_pair = wheeltec_vision_nav.emitter_pair:main" in setup
    assert "vo_watchdog = wheeltec_vision_nav.vo_watchdog:main" in setup
    assert "capture_runtime = wheeltec_vision_nav.capture_runtime:main" in setup


def test_wheel_guess_keeps_visual_tf_and_uses_wheel_odom():
    helpers = (ROOT / "wheeltec_vision_nav" / "launch_utils.py").read_text(encoding="utf-8")
    mapping = (ROOT / "launch" / "vision_mapping.launch.py").read_text(encoding="utf-8")
    package_xml = (ROOT / "package.xml").read_text(encoding="utf-8")
    assert 'params["guess_frame_id"] = "wheel_odom"' in helpers
    assert 'params["guess_min_translation"] = 0.0' in helpers
    assert '"publish_tf": not use_wheel_guess' not in helpers
    assert '"publish_tf": False,\n        "subscribe_rgbd": True' in helpers
    assert 'params["imu_topic"] = "/imu/data"' in helpers
    assert 'params["imu_topic"] = "/imu/data_raw"' not in helpers
    assert 'odom_frame_id = "wheel_odom"' in helpers
    assert 'robot_frame_id = "base_footprint"' in helpers
    assert "<exec_depend>robot_localization</exec_depend>" in package_xml
    assert "<exec_depend>rtabmap_util</exec_depend>" in package_xml
    assert "<exec_depend>joint_state_publisher</exec_depend>" in package_xml
    assert "joint_state_publisher" in helpers
    assert 'visual_odometry_node(use_wheel_guess=use_wheel_guess, use_imu=False)' in helpers
    assert "planar_wheel_pose" in (ROOT / "wheeltec_vision_nav" / "wheel_odom_tf.py").read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("use_wheel_guess", default_value="false")' in mapping
    assert 'follow["vx_min"] = VX_MIN' in helpers
    assert "VX_MIN = -0.15" in helpers
    assert 'follow["vx_min"] = 0.0' not in helpers
    assert '"map_topic": "/map"' in helpers
    assert "/camera/depth/color/points" not in helpers
    assert '"topic": OBSTACLES_TOPIC' in helpers
    assert "hold_odom and not start_base and not localization" not in helpers
    assert "actions.append(hold_odom_tf_node())" in helpers
    assert '("odom", "/odom_vo")' in helpers
    assert '"wait_for_transform": 0.5' in helpers
    assert '"parent_frame": "base_footprint"' in helpers
    assert '"child_frame": "wheel_odom"' in helpers
    assert "use_wheel_guess = start_chassis" in helpers
    assert "capture_runtime_node" in helpers
    assert 'DeclareLaunchArgument("capture", default_value="false")' in mapping
    assert 'DeclareLaunchArgument("capture", default_value="false")' in (ROOT / "launch" / "vision_navigation.launch.py").read_text(encoding="utf-8")


def test_emitter_interleave_resolution_and_sync_window():
    helpers = (ROOT / "wheeltec_vision_nav" / "launch_utils.py").read_text(encoding="utf-8")
    mapping = (ROOT / "launch" / "vision_mapping.launch.py").read_text(encoding="utf-8")
    navigation = (ROOT / "launch" / "vision_navigation.launch.py").read_text(encoding="utf-8")
    camera_yaml = (ROOT / "config" / "realsense_d435c.yaml").read_text(encoding="utf-8")
    assert '"depth_module.depth_profile": "848x480x30"' in helpers
    assert '"depth_module.infra_profile": "848x480x30"' in helpers
    assert "depth_module.emitter_on_off: true" in camera_yaml
    assert '"spatial_filter.enable": "true"' in helpers
    assert '"temporal_filter.enable": "true"' in helpers
    assert '"approx_sync_max_interval": 0.10' in helpers
    assert "/vision/paired/infra1/image_rect_raw" in helpers
    assert "/vision/paired/depth/image_rect_raw" in helpers
    assert "emitter_pair_node()" in helpers
    assert "delayed_camera_stack" in mapping
    assert "delayed_camera_stack" in navigation
    assert "lib/wheeltec_vision_nav/hold_odom_t[f]" in helpers
    assert '"tf_publish_rate": "10.0"' in helpers
    assert '"wait_for_device_timeout": "30.0"' in helpers
    assert '"align_depth.enable": "false"' in helpers
    assert '"enable_color": "false"' in helpers


def test_rtabmap_params_live_in_yaml_and_wipe_db_is_explicit():
    helpers = (ROOT / "wheeltec_vision_nav" / "launch_utils.py").read_text(encoding="utf-8")
    yaml_text = (ROOT / "config" / "rtabmap_visual.yaml").read_text(encoding="utf-8")
    mapping = (ROOT / "launch" / "vision_mapping.launch.py").read_text(encoding="utf-8")
    setup = (ROOT / "setup.py").read_text(encoding="utf-8")
    assert "rtabmap_visual.yaml" in helpers
    assert "rtabmap_visual.yaml" in setup
    assert 'Vis/MaxFeatures: "1500"' in yaml_text
    assert 'Vis/FeatureType: "6"' in yaml_text
    assert 'Vis/MinInliers: "10"' in yaml_text
    assert 'Odom/ResetCountdown: "1"' in yaml_text
    assert 'Rtabmap/DetectionRate: "2"' in yaml_text
    assert 'Grid/NormalsSegmentation: "true"' in yaml_text
    assert 'GridGlobal/Eroded: "true"' in yaml_text
    assert "Grid/FootprintLength" in yaml_text
    assert 'Odom/Strategy: "0"' in yaml_text
    assert 'Vis/EstimationType: "1"' in yaml_text
    assert 'RGBD/LinearUpdate: "0.05"' in yaml_text
    assert 'RGBD/AngularUpdate: "0.05"' in yaml_text
    assert 'Reg/Strategy: "0"' in yaml_text
    assert 'RGBD/NeighborLinkRefining: "false"' in yaml_text
    assert 'Reg/Force3DoF: "true"' in yaml_text
    assert "RGBD/OptimizeMaxError" in yaml_text
    assert 'Grid/RayTracing: "true"' in yaml_text
    assert "Grid/NoiseFilteringRadius" in yaml_text
    assert 'Grid/MaxObstacleHeight: "1.80"' in yaml_text
    assert 'Grid/MaxGroundHeight: "0.10"' in yaml_text
    assert 'DeclareLaunchArgument("wipe_db", default_value="true")' in mapping
    assert 'arguments = ["-d"] if (not localization and wipe_db) else []' in helpers


def test_navigation_chassis_remaps_cmd_vel_to_watchdog_topic():
    helpers = (ROOT / "wheeltec_vision_nav" / "launch_utils.py").read_text(encoding="utf-8")
    assert 'cmd_vel_topic = "cmd_vel_safe" if localization else "cmd_vel"' in helpers
    assert '("odom", "/wheel/odom")' in helpers
    assert '("/cmd_vel",' in helpers
    assert '"cmd_vel": cmd_vel_topic' in helpers


def test_realsense_undeclared_params_go_through_config_file():
    helpers = (ROOT / "wheeltec_vision_nav" / "launch_utils.py").read_text(encoding="utf-8")
    setup = (ROOT / "setup.py").read_text(encoding="utf-8")
    yaml_text = (ROOT / "config" / "realsense_d435c.yaml").read_text(encoding="utf-8")
    assert "realsense_d435c.yaml" in helpers
    assert '"config_file"' in helpers
    assert "realsense_d435c.yaml" in setup
    assert "depth_module.emitter_on_off" in yaml_text
    assert "true" in yaml_text
    # The driver ignores base_frame_id; its TF root is <camera_name>_link.
    assert "base_frame_id" not in yaml_text
    assert "d435c_link" not in yaml_text


def test_camera_pose_comes_from_rewritten_urdf_not_static_tf():
    helpers = (ROOT / "wheeltec_vision_nav" / "launch_utils.py").read_text(encoding="utf-8")
    mapping = (ROOT / "launch" / "vision_mapping.launch.py").read_text(encoding="utf-8")
    navigation = (ROOT / "launch" / "vision_navigation.launch.py").read_text(encoding="utf-8")
    assert "camera_d435c_link" not in helpers
    assert "base_to_d435c_vision" not in helpers
    assert "def rewrite_camera_joint" in helpers
    assert "write_camera_urdf(urdf_path, camera_xyz" in helpers
    assert "DEFAULT_CAMERA_XYZ = (0.187, 0.0175, 0.132)" in helpers
    assert 'DeclareLaunchArgument("camera_xyz", default_value="")' in mapping
    assert 'DeclareLaunchArgument("camera_xyz", default_value="")' in navigation


def test_rewrite_camera_joint_moves_only_camera_origin():
    import importlib.util
    import sys
    import types

    # launch_utils imports ROS launch packages at module import; stub them so
    # the pure helpers can be exercised on a machine without ROS.
    for name in ("ament_index_python", "ament_index_python.packages", "launch", "launch.actions",
                 "launch.launch_description_sources", "launch_ros", "launch_ros.actions"):
        sys.modules.setdefault(name, types.ModuleType(name))
    sys.modules["ament_index_python.packages"].get_package_share_directory = lambda p: "/tmp/" + p
    for attr in ("ExecuteProcess", "IncludeLaunchDescription", "TimerAction"):
        setattr(sys.modules["launch.actions"], attr, object)
    sys.modules["launch.launch_description_sources"].PythonLaunchDescriptionSource = object
    sys.modules["launch_ros.actions"].Node = object
    spec = importlib.util.spec_from_file_location("lu", ROOT / "wheeltec_vision_nav" / "launch_utils.py")
    lu = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(lu)

    urdf = (
        '<robot><joint name="lf_joint" type="continuous"><origin xyz="0.1 0.2 0.3" rpy="0 0 0"/></joint>'
        '<joint\n    name="camera_joint"\n    type="fixed">\n    <origin\n      xyz="0.165778518918356 0.000945202408300414 0.089704"\n      rpy="0 0 0" />'
        '<parent link="base_link"/><child link="camera_link"/></joint></robot>'
    )
    out = lu.rewrite_camera_joint(urdf, (0.187, 0.0175, 0.0897))
    assert 'xyz="0.187000 0.017500 0.089700"' in out
    assert 'xyz="0.1 0.2 0.3"' in out
    assert "0.165778518918356" not in out
    assert lu.parse_xyz("") == lu.DEFAULT_CAMERA_XYZ
    assert lu.parse_xyz("0.2, 0.0, 0.1") == (0.2, 0.0, 0.1)


def test_rtabmap_grid_sensor_is_depth():
    yaml_text = (ROOT / "config" / "rtabmap_visual.yaml").read_text(encoding="utf-8")
    assert 'Grid/Sensor: "1"' in yaml_text


def test_camera_cleanup_ends_leftover_model_and_imu():
    helpers = (ROOT / "wheeltec_vision_nav" / "launch_utils.py").read_text(encoding="utf-8")
    assert "vision_joint_state_publishe[r]" in helpers
    assert "vision_imu_filte[r]" in helpers
    assert "vision_wheeltec_robo[t]" in helpers


def test_vo_watchdog_ackermann_bt_and_initialpose():
    helpers = (ROOT / "wheeltec_vision_nav" / "launch_utils.py").read_text(encoding="utf-8")
    bt = (ROOT / "behavior_trees" / "navigate_w_replanning_ackermann.xml").read_text(encoding="utf-8")
    setup = (ROOT / "setup.py").read_text(encoding="utf-8")
    assert "vo_watchdog_node" in helpers
    assert 'cmd_vel_topic = "cmd_vel_safe" if localization else "cmd_vel"' in helpers
    assert '("initialpose", "/initialpose")' in helpers
    assert "default_nav_to_pose_bt_xml" in helpers
    assert "navigate_w_replanning_ackermann.xml" in helpers
    assert "navigate_w_replanning_ackermann.xml" in setup
    assert "<Spin" not in bt
    assert "<BackUp" in bt
    assert "Wait" in bt
    assert "ClearEntireCostmap" in bt
    assert "if localization:" in helpers
    assert "hold_odom and not start_base and not localization" not in helpers
    assert "actions.append(hold_odom_tf_node())" in helpers


def test_hold_odom_tf_relays_vo_at_image_stamp_only():
    text = (ROOT / "wheeltec_vision_nav" / "hold_odom_tf.py").read_text(encoding="utf-8")
    assert 'self.create_subscription(Odometry, "/odom_vo"' in text
    # Camera-only relay: the EKF owns /odom while the chassis runs.
    assert '"/wheel/odom"' not in text
    assert "t.transform.translation.z = 0.0" in text
    assert 'self.create_publisher(Odometry, "/odom", odom_pub_qos)' in text
    assert "ReliabilityPolicy.BEST_EFFORT" in text
    assert "self.create_timer(0.05, self._tick)" in text
    # The odom -> base_footprint transform is never re-stamped with the current time.
    assert "_broadcast_tf" not in text
    assert "TFMessage(transforms=[_tf_from_sample(sample)])" in text
    assert "stitch_odom_sample" in text
    assert 'self.create_subscription(OccupancyGrid, "/map"' in text
    assert 't.header.frame_id = "map"' in text
    assert 't.child_frame_id = "odom"' in text
    assert "if self.last_valid is not None and self._now() - self.last_valid_time < 0.25" not in text
    assert 't.header.frame_id = "odom"' in text
    assert 't.child_frame_id = "base_footprint"' in text


def test_rviz_config_uses_visual_topics_and_map_frame():
    rviz = (ROOT / "config" / "vision_runtime.rviz").read_text(encoding="utf-8")
    setup = (ROOT / "setup.py").read_text(encoding="utf-8")
    assert "vision_runtime.rviz" in setup
    assert "Fixed Frame: map" in rviz
    assert "/camera/infra1/image_rect_raw" in rviz
    assert "Value: /map" in rviz
    assert "Value: /map_updates" in rviz
    assert "Value: /odom" in rviz
    assert "/camera/depth/color/points" not in rviz
    assert "/scan" not in rviz
    assert "Value: /vision/obstacles" in rviz
    assert "senior_akm_robot.urdf" not in rviz
    assert "Value: /robot_description" in rviz
    assert "Description Source: Topic" in rviz
    assert "/global_costmap/costmap" in rviz
    assert "/local_costmap/costmap" in rviz
    assert "Value: /plan" in rviz
    assert "/camera/depth/image_rect_raw" in rviz


def test_visual_nav_params_local_costmap_has_no_static_layer_and_mppi_matches_inflation():
    helpers = (ROOT / "wheeltec_vision_nav" / "launch_utils.py").read_text(encoding="utf-8")
    assert 'block["plugins"] = ["voxel_layer", "inflation_layer"]' in helpers
    assert 'block.pop("static_layer", None)' in helpers
    assert 'grid_based["allow_unknown"] = True' in helpers
    assert 'obstacles["inflation_radius"] = INFLATION_RADIUS' in helpers
    assert 'inflation["inflation_radius"] = INFLATION_RADIUS' in helpers
    assert 'obstacles["consider_footprint"] = True' in helpers
    assert '"observation_persistence": 0.0' in helpers
    assert 'progress["movement_time_allowance"] = 20.0' in helpers
    assert "vo_watchdog_node(wheel_backup=start_chassis)" in helpers
    assert '"map_stale_s": 5.0' in helpers


class _RecordedNode:
    def __init__(self, **kwargs):
        self.kwargs = kwargs


def _launch_utils():
    """Import launch_utils with ROS launch packages stubbed; Node records its kwargs."""
    import importlib.util
    import sys
    import types

    for name in ("ament_index_python", "ament_index_python.packages", "launch", "launch.actions",
                 "launch.launch_description_sources", "launch_ros", "launch_ros.actions"):
        sys.modules.setdefault(name, types.ModuleType(name))
    sys.modules["ament_index_python.packages"].get_package_share_directory = lambda p: str(ROOT)
    for attr in ("ExecuteProcess", "IncludeLaunchDescription", "TimerAction"):
        setattr(sys.modules["launch.actions"], attr, _RecordedNode)
    sys.modules["launch.launch_description_sources"].PythonLaunchDescriptionSource = _RecordedNode
    sys.modules["launch_ros.actions"].Node = _RecordedNode
    spec = importlib.util.spec_from_file_location("lu_runtime", ROOT / "wheeltec_vision_nav" / "launch_utils.py")
    lu = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(lu)
    return lu


def _executables(actions):
    return [a.kwargs.get("executable") for a in actions]


def test_chassis_run_gives_odom_to_the_ekf():
    lu = _launch_utils()
    actions = lu.visual_runtime_actions("/tmp/x.db", localization=True, start_base=True, hold_odom=False)
    names = _executables(actions)
    assert "ekf_node" in names
    assert "ekf_inputs" in names
    assert "hold_odom_tf" not in names
    assert "imu_filter_madgwick_node" not in names
    assert {"point_cloud_xyz", "obstacles_detection", "vo_watchdog"} <= set(names)
    ekf = actions[names.index("ekf_node")].kwargs
    assert ("odometry/filtered", "/odom") in ekf["remappings"]
    rtabmap = actions[names.index("rtabmap")].kwargs["parameters"][1]
    assert rtabmap["odom_frame_id"] == "odom"
    assert 0.0 < rtabmap["odom_tf_angular_variance"] < 0.01
    assert 0.0 < rtabmap["odom_tf_linear_variance"] < 0.01


def test_camera_only_mapping_relays_vo_without_ekf():
    lu = _launch_utils()
    names = _executables(lu.visual_runtime_actions("/tmp/x.db", localization=False, start_base=False, hold_odom=False))
    assert "hold_odom_tf" in names
    assert "ekf_node" not in names
    assert "point_cloud_xyz" not in names


def test_ekf_config_fuses_wheel_gyro_and_vo_velocity():
    import yaml

    cfg = yaml.safe_load((ROOT / "config" / "ekf_visual.yaml").read_text(encoding="utf-8"))
    p = cfg["vision_ekf"]["ros__parameters"]
    assert p["two_d_mode"] is True and p["publish_tf"] is True
    assert p["world_frame"] == "odom" and p["base_link_frame"] == "base_footprint"
    vx, vy, vyaw = 6, 7, 11
    assert p["odom0"] == "/wheel/odom"
    assert p["odom0_config"][vx] and p["odom0_config"][vy] and p["odom0_config"][vyaw]
    assert p["imu0"] == "/vision/imu" and p["imu0_config"][vyaw]
    assert sum(p["imu0_config"]) == 1
    assert p["odom1"] == "/vision/odom_vo" and p["odom1_config"][vx] and p["odom1_config"][vyaw]
    assert p["odom1_twist_rejection_threshold"] > 0
    assert not any(p["odom0_config"][:6]) and not any(p["odom1_config"][:6])
    assert len(p["process_noise_covariance"]) == 225


def test_generated_nav_params_follow_the_visual_pipeline(tmp_path):
    import yaml

    lu = _launch_utils()
    source = {
        "controller_server": {"ros__parameters": {
            "controller_frequency": 20.0,
            "FollowPath": {"plugin": "nav2_mppi_controller::MPPIController", "model_dt": 0.05,
                           "time_steps": 56, "batch_size": 2000, "vx_min": -0.5,
                           "PreferForwardCritic": {"cost_weight": 1.5}},
        }},
        "planner_server": {"ros__parameters": {"GridBased": {
            "plugin": "nav2_smac_planner/SmacPlannerHybrid", "allow_unknown": False, "reverse_penalty": 1.5}}},
        "local_costmap": {"local_costmap": {"ros__parameters": {
            "plugins": ["static_layer", "voxel_layer", "inflation_layer"], "static_layer": {}, "voxel_layer": {}}}},
        "global_costmap": {"global_costmap": {"ros__parameters": {
            "plugins": ["static_layer", "obstacle_layer", "inflation_layer"], "obstacle_layer": {}}}},
    }
    path = tmp_path / "nav.yaml"
    path.write_text(yaml.safe_dump(source), encoding="utf-8")
    out = yaml.safe_load(Path(lu.write_visual_nav_params(str(path))).read_text(encoding="utf-8"))
    controller = out["controller_server"]["ros__parameters"]
    follow = controller["FollowPath"]
    assert abs(1.0 / controller["controller_frequency"] - follow["model_dt"]) < 1e-9
    assert follow["time_steps"] * follow["model_dt"] >= 2.5
    assert follow["batch_size"] == 1000
    assert follow["vx_min"] == -0.15
    assert follow["PreferForwardCritic"]["cost_weight"] == 5.0
    grid = out["planner_server"]["ros__parameters"]["GridBased"]
    assert grid["allow_unknown"] is True and grid["reverse_penalty"] == 3.0
    local = out["local_costmap"]["local_costmap"]["ros__parameters"]
    assert local["plugins"] == ["voxel_layer", "inflation_layer"]
    assert local["voxel_layer"]["depth_points"]["topic"] == "/vision/obstacles"
    glob = out["global_costmap"]["global_costmap"]["ros__parameters"]
    assert glob["plugins"][0] == "static_layer" and "obstacle_layer" not in glob["plugins"]
