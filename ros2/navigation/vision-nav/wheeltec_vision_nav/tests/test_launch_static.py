"""Exercise launch action callbacks with scoped substitutes, without ROS."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest
import yaml

from wheeltec_vision_nav.startup import ResourceConflict, ResourceLeases, database_resource

ROOT = Path(__file__).resolve().parents[1]


def nodes_by_executable(actions):
    return {a.kwargs["executable"]: a.kwargs for a in actions if "executable" in a.kwargs}


@pytest.mark.parametrize("localization", [False, True])
@pytest.mark.parametrize("start_base", [False, True])
@pytest.mark.parametrize("use_guess", [False, True])
@pytest.mark.parametrize("odom_source", ["auto", "vo_only", "external", "ekf"])
def test_runtime_has_one_odometry_owner_and_always_gates_actuation(launch_utils, localization, start_base, use_guess, odom_source):
    nodes = nodes_by_executable(launch_utils.visual_runtime_actions(
        "map.db", localization, start_base, use_wheel_guess=use_guess, odom_source=odom_source))
    resolved = ("ekf" if start_base else "vo_only") if odom_source == "auto" else odom_source
    assert ("wheeltec_robot_node" in nodes) is start_base
    assert ("ekf_node" in nodes) is (resolved == "ekf")
    assert ("hold_odom_tf" in nodes) is (resolved == "vo_only")
    assert not {"ekf_node", "hold_odom_tf"} <= nodes.keys()
    assert {"depth_validity_gate", "point_cloud_xyz", "obstacles_detection", "vo_watchdog"} <= nodes.keys()
    guard = nodes["vo_watchdog"]["parameters"][0]
    assert guard["mode"] == ("navigation" if localization else "mapping")
    assert guard["wheel_backup"] is (resolved == "ekf")
    assert guard["require_depth"] and not guard["supervised_reverse"]
    assert guard["cmd_vel_in"] == "/cmd_vel" and guard["cmd_vel_out"] == "/cmd_vel_safe"
    vo = nodes["rgbd_odometry"]["parameters"][1]
    assert ("guess_frame_id" in vo) is use_guess and vo["publish_tf"] is False
    if resolved == "ekf":
        assert nodes["ekf_inputs"]["parameters"][0]["wheel_prior_used"] is use_guess
    if start_base:
        base = nodes["wheeltec_robot_node"]
        assert ("/cmd_vel", "/cmd_vel_safe") in base["remappings"]
        assert ("mobile_base/sensors/imu_data", "/imu/data_raw") in base["remappings"]
        assert base["parameters"][0]["cmd_vel_timeout_s"] == .3


def test_only_safe_topic_can_reach_the_actuator(launch_utils):
    with pytest.raises(ValueError, match="cmd_vel_safe"):
        launch_utils.isolated_base_node(cmd_vel_topic="/cmd_vel")
    with pytest.raises(ValueError, match="backup"):
        launch_utils.rtabmap_node("map.db", wipe_db=True)
    assert launch_utils.rtabmap_node("map.db").kwargs["arguments"] == []


def test_raw_obstacle_path_does_not_depend_on_paired_vo_depth(launch_utils):
    gate, cloud, obstacles = [a.kwargs for a in launch_utils.depth_obstacle_nodes()]
    assert gate["executable"] == "depth_validity_gate"
    assert ("depth/image", "/camera/depth/image_rect_raw") in gate["remappings"]
    assert ("depth/camera_info", "/camera/depth/camera_info") in gate["remappings"]
    assert ("depth/image", "/vision/valid_depth/image") in cloud["remappings"]
    assert ("depth/camera_info", "/vision/valid_depth/camera_info") in cloud["remappings"]
    assert cloud["parameters"][0]["approx_sync"] is False
    assert ("cloud", "/vision/depth_cloud") in cloud["remappings"]
    assert ("obstacles", "/vision/obstacles") in obstacles["remappings"]
    assert not any("paired" in topic for _, topic in cloud["remappings"])


def test_nav2_command_remaps_are_scoped_and_recoveries_share_the_guard(launch_utils):
    nodes = nodes_by_executable(launch_utils.navigation_nodes("params.yaml"))
    for name in ("controller_server", "behavior_server"):
        assert ("cmd_vel", "/cmd_vel_nav") in nodes[name]["remappings"]
    smoother = nodes["velocity_smoother"]
    assert ("cmd_vel", "/cmd_vel_nav") in smoother["remappings"]
    assert ("cmd_vel_smoothed", "/cmd_vel") in smoother["remappings"]
    assert ("cmd_vel_smoothed", "/cmd_vel_nav") not in smoother["remappings"]
    assert nodes["lifecycle_manager"]["parameters"][0]["autostart"] is False


def test_custom_safety_deadlines_reach_raw_gate_and_ekf_inputs(launch_utils):
    safety = {"sensor_timeout_s": .4, "future_tolerance_s": .02, "min_inliers": 20,
              "vo_timeout_s": .6, "wheel_fresh_s": .2}
    gate = launch_utils.depth_obstacle_nodes(safety=safety)[0].kwargs
    assert gate["parameters"] == [{"sensor_timeout_s": .4, "future_tolerance_s": .02}]
    ekf = launch_utils.ekf_inputs_node(safety=safety).kwargs
    assert ekf["parameters"] == [{"wheel_prior_used": False, "min_inliers": 20,
                                   "quality_timeout_s": .6, "wheel_timeout_s": .2}]


@pytest.mark.parametrize("reverse", [False, True])
@pytest.mark.parametrize("through", [False, True])
def test_behavior_trees_apply_explicit_reverse_policy(launch_utils, reverse, through):
    path = Path(launch_utils.ackermann_bt_xml(reverse, through))
    tree = ET.parse(path)
    assert not tree.findall(".//Spin")
    backups = tree.findall(".//BackUp")
    assert bool(backups) is reverse
    if reverse:
        assert backups[0].attrib == {"backup_dist": "0.25", "backup_speed": "0.08"}
    assert tree.findall(".//Wait") and tree.findall(".//ClearEntireCostmap")
    assert bool(tree.findall(".//ComputePathThroughPoses")) is through
    if through:
        sequence = tree.find(".//RateController/RecoveryNode/ReactiveSequence")
        assert sequence is not None
        assert [child.tag for child in sequence] == ["RemovePassedGoals", "ComputePathThroughPoses"]
        remove, compute = list(sequence)
        assert remove.attrib == {"input_goals": "{goals}", "output_goals": "{goals}", "radius": "0.7"}
        assert compute.attrib["goals"] == remove.attrib["output_goals"]
    else:
        assert not tree.findall(".//RemovePassedGoals")


@pytest.fixture
def build_launch(launch_utils, monkeypatch, tmp_path):
    from wheeltec_vision_nav import launch_common
    robot = tmp_path / "robot"
    (robot / "urdf").mkdir(parents=True)
    (robot / "config").mkdir()
    (robot / "urdf/senior_akm_robot.urdf").write_text(
        '<robot><joint name="camera_joint"><origin xyz="0 0 0" rpy="0 0 1"/></joint></robot>', encoding="utf-8")
    (robot / "config/wheeltec_param.yaml").write_text("car_mode: mini_mec\n", encoding="utf-8")
    values = {key: [0., 0., .04, 0., 0., 0.] for key in ("base_to_link", "base_to_laser", "base_to_gyro", "base_to_radar")}
    (robot / "config/robot_model.yaml").write_text(yaml.safe_dump({"robot_model": {"senior_akm": values}}), encoding="utf-8")
    monkeypatch.setattr(launch_common, "get_package_share_directory", lambda package: str(ROOT if package == "wheeltec_vision_nav" else robot))
    def build(localization, **overrides):
        filename = "vision_navigation.launch.py" if localization else "vision_mapping.launch.py"
        spec = importlib.util.spec_from_file_location("launch_under_test", ROOT / "launch" / filename)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        actions = module.generate_launch_description().args[0]
        defaults = {a.args[0]: a.kwargs["default_value"] for a in actions if a.__class__.__name__ == "DeclareLaunchArgument"}
        defaults.update(database_path=str(tmp_path / "map.db"), lease_dir=str(tmp_path / "leases"))
        defaults.update(overrides)
        if localization:
            Path(defaults["database_path"]).touch()
        opaque = next(a for a in actions if a.__class__.__name__ == "OpaqueFunction")
        return defaults, opaque.execute(defaults)
    return build


def handler(actions, name, target=False):
    choices = [a.args[0] for a in actions if a.__class__.__name__ == "RegisterEventHandler"]
    return next(h for h in choices if h.__class__.__name__ == name and ("target_action" in h.kwargs) is target)


def event(action=None, code=0):
    return SimpleNamespace(action=action, returncode=code)


def finish(actions):
    # Lease release follows exit of the last child, even during launch shutdown.
    handler(actions, "OnShutdown").handle(event(), {})


@pytest.mark.parametrize("localization", [False, True])
def test_launch_preflight_starts_consumers_immediately_and_monitor_by_default(build_launch, localization):
    defaults, actions = build_launch(localization, camera_rpy="0 .1 .2", max_features="750", mppi_batch_size="500")
    try:
        assert defaults["wipe_db"] == "false" and defaults["start_base"] == "false"
        assert not any(a.__class__.__name__ in ("TimerAction", "ExecuteProcess") for a in actions)
        nodes = nodes_by_executable(actions)
        assert {"vo_watchdog", "runtime_monitor", "wait_ready", "depth_validity_gate", "point_cloud_xyz", "rgbd_odometry"} <= nodes.keys()
        assert "controller_server" not in nodes
        assert nodes["wait_ready"]["parameters"][0]["timeout_s"] == 30.
        assert nodes["emitter_pair"]["parameters"][0]["max_interval"] == .05
        assert nodes["rgbd_odometry"]["parameters"][1]["Vis/MaxFeatures"] == "750"
        monitor = nodes["runtime_monitor"]["parameters"][0]
        assert monitor["resource_root_pid"] > 0
        runtime = yaml.safe_load(Path(monitor["runtime_config"]).read_text(encoding="utf-8"))
        assert runtime["robot"]["odom_source"] == "vo_only"
        assert runtime["robot"]["camera_rpy"] == "0 .1 .2"
        urdf = Path(nodes["robot_state_publisher"]["arguments"][0]).read_text(encoding="utf-8")
        assert 'rpy="0.000000 0.100000 0.200000"' in urdf
        if localization:
            nav = yaml.safe_load(Path(monitor["nav_params_file"]).read_text(encoding="utf-8"))
            assert nav["controller_server"]["ros__parameters"]["FollowPath"]["batch_size"] == 500
            assert "nav2_remove_passed_goals_action_bt_node" in nav["bt_navigator"]["ros__parameters"]["plugin_lib_names"]
    finally:
        finish(actions)


def test_ready_exit_success_starts_nav2_and_timeout_shuts_down(build_launch):
    _, actions = build_launch(True)
    try:
        ready = handler(actions, "OnProcessExit", target=True)
        waiter = ready.kwargs["target_action"]
        nav = ready.handle(event(waiter, 0), {})
        assert "controller_server" in nodes_by_executable(nav)
        shutdown = ready.handle(event(waiter, 2), {})
        assert shutdown[0].__class__.__name__ == "EmitEvent"
        assert "Readiness timed out" in shutdown[0].kwargs["event"].kwargs["reason"]
    finally:
        finish(actions)


@pytest.mark.parametrize("executable", ["vo_watchdog", "depth_validity_gate"])
def test_critical_exit_stops_launch_and_leases_survive_until_last_child_exit(build_launch, executable):
    defaults, actions = build_launch(False)
    nodes = [a for a in actions if "executable" in a.kwargs]
    process = next(a for a in nodes if a.kwargs["executable"] == executable)
    monitor = next(a for a in nodes if a.kwargs["executable"] == "runtime_monitor")
    start = handler(actions, "OnProcessStart")
    exited = handler(actions, "OnProcessExit")
    start.handle(event(process), {})
    start.handle(event(monitor), {})
    try:
        stopped = exited.handle(event(process, 1), {})
        assert "exited with code 1" in stopped[0].kwargs["event"].kwargs["reason"]
        finish(actions)
        resource = database_resource(defaults["database_path"])
        with pytest.raises(ResourceConflict):
            ResourceLeases(defaults["lease_dir"]).acquire([resource])
        exited.handle(event(monitor), {})
        lease = ResourceLeases(defaults["lease_dir"]).acquire([resource])
        lease.release()
    finally:
        exited.handle(event(monitor), {})


def test_camera_process_inside_include_is_critical(build_launch):
    _, actions = build_launch(False)
    try:
        camera = SimpleNamespace(process_details={"cmd": ["/opt/ros/humble/lib/realsense2_camera/realsense2_camera_node"], "name": "camera"})
        # Actual launch processes are hashable actions.
        action = type("CameraProcess", (), {})()
        action.process_details = camera.process_details
        stopped = handler(actions, "OnProcessExit").handle(event(action, 1), {})
        assert "camera exited" in stopped[0].kwargs["event"].kwargs["reason"]
    finally:
        finish(actions)


def test_preflight_failures_release_ownership(build_launch, tmp_path):
    with pytest.raises(ValueError, match="senior_akm"):
        build_launch(False, car_mode="mini_mec")
    bad = tmp_path / "bad.yaml"
    bad.write_text("amcl: {}", encoding="utf-8")
    with pytest.raises(ValueError, match="critical"):
        build_launch(True, nav_params_file=str(bad))
    _, actions = build_launch(False)
    finish(actions)


def test_runtime_files_are_free_of_process_killing_and_fixed_startup_delays():
    for name in ("launch_camera.py", "launch_pipeline.py", "launch_utils.py"):
        content = (ROOT / "wheeltec_vision_nav" / name).read_text(encoding="utf-8")
        assert "pkill" not in content and "TimerAction" not in content


@pytest.mark.parametrize("localization", [False, True])
@pytest.mark.parametrize("name,value", [("reverse_speed", .04), ("reverse_m", .1), ("reverse_s", 1.0)])
def test_unsupported_reverse_limits_fail_before_launch_side_effects(build_launch, monkeypatch, tmp_path, localization, name, value):
    from wheeltec_vision_nav import launch_pipeline

    runtime = yaml.safe_load((ROOT / "config/runtime.yaml").read_text(encoding="utf-8"))
    runtime["safety"][name] = value
    custom = tmp_path / "custom_runtime.yaml"
    custom.write_text(yaml.safe_dump(runtime), encoding="utf-8")
    database = tmp_path / "map.db"
    database.write_bytes(b"original database must survive rejected preflight")
    def unexpected(*args, **kwargs):
        pytest.fail("Rejected reverse configuration reached a side effect")
    monkeypatch.setattr(launch_pipeline.ResourceLeases, "acquire", unexpected)
    for function in ("model_actions", "write_visual_nav_params", "_write_effective", "prepare_database"):
        monkeypatch.setattr(launch_pipeline, function, unexpected)
    with pytest.raises(ValueError, match=name):
        build_launch(localization, runtime_config=str(custom), supervised_reverse="true", wipe_db="true", start_base="true")
    assert database.read_bytes() == b"original database must survive rejected preflight"
    assert not (tmp_path / "leases").exists()


@pytest.mark.parametrize("reverse", [False, True])
def test_custom_nav_template_gets_pruning_plugin_in_effective_launch_config(build_launch, tmp_path, reverse):
    source = yaml.safe_load((ROOT / "config/visual_nav_baseline.yaml").read_text(encoding="utf-8"))
    original_plugins = source["bt_navigator"]["ros__parameters"]["plugin_lib_names"]
    original_plugins.remove("nav2_remove_passed_goals_action_bt_node")
    custom = tmp_path / "custom_nav.yaml"
    original = yaml.safe_dump(source)
    custom.write_text(original, encoding="utf-8")
    _, actions = build_launch(True, nav_params_file=str(custom), supervised_reverse=str(reverse).lower())
    try:
        monitor = nodes_by_executable(actions)["runtime_monitor"]["parameters"][0]
        effective = yaml.safe_load(Path(monitor["nav_params_file"]).read_text(encoding="utf-8"))
        bt = effective["bt_navigator"]["ros__parameters"]
        assert bt["plugin_lib_names"] == original_plugins + ["nav2_remove_passed_goals_action_bt_node"]
        tree = ET.parse(bt["default_nav_through_poses_bt_xml"])
        assert tree.find(".//ReactiveSequence/RemovePassedGoals") is not None
        assert custom.read_text(encoding="utf-8") == original
    finally:
        finish(actions)
