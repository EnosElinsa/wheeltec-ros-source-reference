"""Shared runtime and strict Ackermann/Nav2 configuration contracts."""
from copy import deepcopy
from dataclasses import asdict
from pathlib import Path
import sys

import pytest
import yaml

from wheeltec_vision_nav.nav_params import build_visual_nav_params
from wheeltec_vision_nav.runtime_config import load_runtime_config, resolve_odom_source, safety_parameters, validate_runtime_config
from wheeltec_vision_nav.runtime_safety import SafetyConfig

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def baseline():
    return yaml.safe_load((ROOT / "config/visual_nav_baseline.yaml").read_text(encoding="utf-8"))


@pytest.fixture
def runtime():
    return load_runtime_config(ROOT / "config/runtime.yaml")


def test_shared_safety_defaults_are_complete_and_match_the_guard(runtime):
    expected = asdict(SafetyConfig())
    expected.pop("mode")
    expected.pop("wheel_backup")
    assert runtime["safety"] == expected
    assert runtime["perception"]["camera_profile"] == "848x480x30"
    assert runtime["perception"]["max_features"] == 1500
    assert runtime["navigation"]["mppi_batch_size"] == 1000
    del runtime["safety"]["collision_margin"]
    with pytest.raises(ValueError, match="SafetyConfig"):
        validate_runtime_config(runtime)


@pytest.mark.parametrize("source,start_base,hold,expected", [
    ("auto", False, False, "vo_only"), ("auto", True, False, "ekf"),
    ("auto", True, True, "vo_only"), ("vo_only", True, True, "vo_only"),
    ("external", False, False, "external"), ("ekf", False, False, "ekf"),
])
def test_explicit_odometry_ownership(source, start_base, hold, expected):
    assert resolve_odom_source(source, start_base=start_base, hold_odom=hold) == expected


@pytest.mark.parametrize("source", ["ekf", "external", "invalid"])
def test_hold_odom_rejects_conflicting_owners(source):
    with pytest.raises(ValueError):
        resolve_odom_source(source, hold_odom=True)


@pytest.mark.parametrize("reverse", [False, True])
def test_reverse_policy_timing_geometry_and_separate_cloud_sources(baseline, runtime, reverse):
    original = deepcopy(baseline)
    out = build_visual_nav_params(baseline, "pose.xml", through_tree_path="poses.xml",
                                  supervised_reverse=reverse, runtime=runtime)
    assert baseline == original
    control = out["controller_server"]["ros__parameters"]
    follow = control["FollowPath"]
    assert 1.0 / control["controller_frequency"] == pytest.approx(follow["model_dt"])
    assert (control["controller_frequency"], follow["time_steps"], follow["batch_size"]) == (10., 28, 1000)
    assert (follow["vx_min"], follow["vx_max"]) == (-.08 if reverse else 0., .3)
    grid = out["planner_server"]["ros__parameters"]["GridBased"]
    assert grid["motion_model_for_search"] == ("REEDS_SHEPP" if reverse else "DUBIN")
    assert grid["allow_unknown"] is False
    assert grid["minimum_turning_radius"] == follow["AckermannConstraints"]["min_turning_r"] == runtime["safety"]["min_turning_radius"]
    local = out["local_costmap"]["local_costmap"]["ros__parameters"]
    voxel = local["voxel_layer"]
    marking, clearing = voxel["obstacle_points"], voxel["depth_clear"]
    assert (marking["topic"], marking["marking"], marking["clearing"]) == ("/vision/obstacles", True, False)
    assert (clearing["topic"], clearing["marking"], clearing["clearing"]) == ("/vision/depth_cloud", False, True)
    assert clearing["min_obstacle_height"] < 0
    assert not local["track_unknown_space"]
    behavior = out["behavior_server"]["ros__parameters"]
    assert "spin" not in behavior
    assert ("backup" in behavior["behavior_plugins"]) is reverse
    assert out["velocity_smoother"]["ros__parameters"]["min_velocity"][0] == follow["vx_min"]
    guard = safety_parameters(runtime, localization=True, odom_source="ekf", supervised_reverse=reverse)
    assert guard["supervised_reverse"] is reverse and guard["wheel_backup"]


def test_custom_profile_keeps_guard_nav_and_reported_tuning_coherent(baseline, runtime):
    runtime["navigation"]["mppi_batch_size"] = 500
    runtime["safety"].update(min_turning_radius=.8, footprint_front=.3)
    out = build_visual_nav_params(baseline, "pose.xml", runtime=runtime)
    follow = out["controller_server"]["ros__parameters"]["FollowPath"]
    assert follow["batch_size"] == 500
    assert follow["AckermannConstraints"]["min_turning_r"] == .8
    assert "0.3" in out["local_costmap"]["local_costmap"]["ros__parameters"]["footprint"]


@pytest.mark.parametrize("path,value,match", [
    (("controller_server", "ros__parameters", "FollowPath", "plugin"), "other", "MPPI"),
    (("controller_server", "ros__parameters", "FollowPath", "motion_model"), "DiffDrive", "Ackermann"),
    (("controller_server", "ros__parameters", "FollowPath", "model_dt"), .05, "period"),
    (("planner_server", "ros__parameters", "GridBased", "plugin"), "navfn", "Smac"),
    (("planner_server", "ros__parameters", "GridBased", "motion_model_for_search"), "MOORE", "motion_model"),
    (("planner_server", "ros__parameters", "GridBased", "minimum_turning_radius"), 0, "turning_radius"),
    (("local_costmap", "local_costmap", "ros__parameters", "footprint"), "[[0,0],[0,0],[0,0]]", "footprint"),
])
def test_unsupported_critical_template_settings_fail(baseline, path, value, match):
    target = baseline
    for key in path[:-1]:
        target = target[key]
    target[path[-1]] = value
    with pytest.raises(ValueError, match=match):
        build_visual_nav_params(baseline, "pose.xml")


def test_missing_critical_template_setting_fails_without_silent_noop(baseline):
    del baseline["planner_server"]["ros__parameters"]["GridBased"]
    with pytest.raises(ValueError, match="Missing critical"):
        build_visual_nav_params(baseline, "pose.xml")


def test_standalone_nav_generation_finds_installed_runtime_share(launch_utils, baseline, monkeypatch, tmp_path):
    from wheeltec_vision_nav import nav_params

    # Simulate a non-symlink install: Python module and share resources are in
    # different directories, and no config exists beside the Python package.
    monkeypatch.setattr(nav_params, "__file__", str(tmp_path / "lib/python3/site-packages/wheeltec_vision_nav/nav_params.py"))
    installed_share = tmp_path / "share/wheeltec_vision_nav"
    (installed_share / "config").mkdir(parents=True)
    runtime = yaml.safe_load((ROOT / "config/runtime.yaml").read_text(encoding="utf-8"))
    runtime["navigation"]["mppi_batch_size"] = 321
    (installed_share / "config/runtime.yaml").write_text(yaml.safe_dump(runtime), encoding="utf-8")
    lookups = []
    def lookup(package):
        lookups.append(package)
        return str(installed_share)
    monkeypatch.setattr(sys.modules["ament_index_python.packages"], "get_package_share_directory", lookup)
    result = nav_params.build_visual_nav_params(baseline, "pose.xml")
    assert lookups == ["wheeltec_vision_nav"]
    assert result["controller_server"]["ros__parameters"]["FollowPath"]["batch_size"] == 321


@pytest.mark.parametrize("reverse", [False, True])
def test_custom_older_bt_plugin_list_is_completed_without_mutating_source(baseline, runtime, reverse):
    supplied = baseline["bt_navigator"]["ros__parameters"]["plugin_lib_names"]
    supplied.remove("nav2_remove_passed_goals_action_bt_node")
    supplied.insert(0, "nav2_goal_reached_condition_bt_node")
    original = deepcopy(baseline)
    result = build_visual_nav_params(baseline, "pose.xml", through_tree_path="poses.xml",
                                     supervised_reverse=reverse, runtime=runtime)
    plugins = result["bt_navigator"]["ros__parameters"]["plugin_lib_names"]
    assert baseline == original
    assert plugins[:-1] == supplied
    assert plugins[-1] == "nav2_remove_passed_goals_action_bt_node"
    assert plugins.count("nav2_remove_passed_goals_action_bt_node") == 1
    # Regenerating an effective configuration must not add duplicate libraries.
    regenerated = build_visual_nav_params(result, "pose.xml", through_tree_path="poses.xml",
                                           supervised_reverse=reverse, runtime=runtime)
    assert regenerated["bt_navigator"]["ros__parameters"]["plugin_lib_names"] == plugins


def test_missing_custom_bt_plugin_list_gets_all_libraries_used_by_the_trees(baseline, runtime):
    required = baseline["bt_navigator"]["ros__parameters"].pop("plugin_lib_names")
    result = build_visual_nav_params(baseline, "pose.xml", through_tree_path="poses.xml", runtime=runtime)
    plugins = result["bt_navigator"]["ros__parameters"]["plugin_lib_names"]
    assert set(required) <= set(plugins)


def test_each_navigation_goal_recreates_tree_node_state_even_with_same_xml(baseline, runtime):
    baseline['bt_navigator']['ros__parameters']['always_reload_bt_xml'] = False
    output = build_visual_nav_params(baseline, 'pose.xml', through_tree_path='poses.xml', runtime=runtime)
    bt = output['bt_navigator']['ros__parameters']
    assert bt['always_reload_bt_xml'] is True
    assert bt['default_nav_to_pose_bt_xml'] == 'pose.xml'
    assert bt['default_nav_through_poses_bt_xml'] == 'poses.xml'
    assert baseline['bt_navigator']['ros__parameters']['always_reload_bt_xml'] is False
