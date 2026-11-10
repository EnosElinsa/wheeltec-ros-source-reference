"""Validated, reproducible Nav2 parameters for visual Ackermann navigation."""
import ast
from copy import deepcopy
import math
import os
from pathlib import Path
import tempfile

import yaml

from wheeltec_vision_nav.runtime_config import load_runtime_config, validate_runtime_config

OBSTACLES_TOPIC = "/vision/obstacles"
DEPTH_CLOUD_TOPIC = "/vision/depth_cloud"
INFLATION_RADIUS, COST_SCALING_FACTOR = 0.25, 3.0
VX_MAX, VX_MIN, WZ_MAX = 0.3, 0.0, 1.0
CONTROLLER_FREQUENCY, MPPI_MODEL_DT = 10.0, 0.1
MPPI_TIME_STEPS, MPPI_BATCH_SIZE = 28, 1000
PREFER_FORWARD_WEIGHT, SMAC_REVERSE_PENALTY = 5.0, 3.0

# These libraries register the nodes used by our installed to-pose and
# through-poses trees. A custom template may predate a tree change, so complete
# its effective plugin list whenever we replace the tree paths.
BT_PLUGIN_LIBRARIES = (
    "nav2_compute_path_to_pose_action_bt_node",
    "nav2_compute_path_through_poses_action_bt_node",
    "nav2_remove_passed_goals_action_bt_node",
    "nav2_follow_path_action_bt_node",
    "nav2_back_up_action_bt_node",
    "nav2_wait_action_bt_node",
    "nav2_clear_costmap_service_bt_node",
    "nav2_goal_updated_condition_bt_node",
    "nav2_rate_controller_bt_node",
    "nav2_recovery_node_bt_node",
    "nav2_pipeline_sequence_bt_node",
    "nav2_round_robin_node_bt_node",
)


def bundled_runtime_config():
    """Find defaults in a source checkout or a normal ament installation."""
    path = Path(__file__).resolve().parents[1] / "config/runtime.yaml"
    if not path.is_file():
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory("wheeltec_vision_nav")) / "config/runtime.yaml"
    return load_runtime_config(path)


def _required(mapping, key, location):
    if not isinstance(mapping, dict) or key not in mapping:
        raise ValueError(f"Missing critical Nav2 setting: {location}.{key}")
    return mapping[key]


def _positive(value, location):
    if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{location} must be finite and positive")
    return value


def validate_nav_template(cfg):
    """Reject unsupported or incomplete templates before applying any override."""
    controller = _required(_required(cfg, "controller_server", "root"), "ros__parameters", "controller_server")
    follow = _required(controller, "FollowPath", "controller_server")
    if _required(controller, "controller_plugins", "controller_server") != ["FollowPath"]:
        raise ValueError("controller_plugins must contain only FollowPath")
    if _required(follow, "plugin", "FollowPath") != "nav2_mppi_controller::MPPIController":
        raise ValueError("Only the MPPIController plugin is supported")
    if _required(follow, "motion_model", "FollowPath") != "Ackermann":
        raise ValueError("MPPI motion_model must be Ackermann")
    frequency = _positive(_required(controller, "controller_frequency", "controller_server"), "controller_frequency")
    period = _positive(_required(follow, "model_dt", "FollowPath"), "model_dt")
    if not math.isclose(1.0 / frequency, period, rel_tol=1e-6):
        raise ValueError("controller period must equal MPPI model_dt")
    _positive(_required(_required(follow, "AckermannConstraints", "FollowPath"), "min_turning_r", "AckermannConstraints"), "min_turning_r")
    planner = _required(_required(cfg, "planner_server", "root"), "ros__parameters", "planner_server")
    if _required(planner, "planner_plugins", "planner_server") != ["GridBased"]:
        raise ValueError("planner_plugins must contain only GridBased")
    grid = _required(planner, "GridBased", "planner_server")
    if _required(grid, "plugin", "GridBased") not in ("nav2_smac_planner/SmacPlannerHybrid", "nav2_smac_planner::SmacPlannerHybrid"):
        raise ValueError("Only SmacPlannerHybrid is supported")
    if _required(grid, "motion_model_for_search", "GridBased") not in ("DUBIN", "REEDS_SHEPP"):
        raise ValueError("Unsupported Smac motion_model_for_search")
    _positive(_required(grid, "minimum_turning_radius", "GridBased"), "minimum_turning_radius")
    for name in ("local_costmap", "global_costmap"):
        block = _required(_required(_required(cfg, name, "root"), name, name), "ros__parameters", name)
        footprint = _required(block, "footprint", name)
        try:
            polygon = ast.literal_eval(footprint) if isinstance(footprint, str) else footprint
            valid = (len(polygon) >= 3 and all(len(p) == 2 and all(math.isfinite(float(v)) for v in p) for p in polygon))
            area = abs(sum(polygon[i][0] * polygon[(i + 1) % len(polygon)][1]
                           - polygon[(i + 1) % len(polygon)][0] * polygon[i][1] for i in range(len(polygon)))) if valid else 0
        except (ValueError, TypeError, SyntaxError, OverflowError):
            area = 0
        if not area:
            raise ValueError(f"{name}.footprint must be a finite nonzero-area polygon")
    return cfg


def build_visual_nav_params(source, behavior_tree_path, *, supervised_reverse=False,
                            runtime=None, through_tree_path=None):
    validate_nav_template(source)
    runtime = deepcopy(runtime) if runtime is not None else bundled_runtime_config()
    validate_runtime_config(runtime)
    safe = runtime["safety"]
    cfg = deepcopy(source)
    for key in ("amcl", "amcl_map_client", "amcl_rclcpp_node", "map_server", "map_saver"):
        cfg.pop(key, None)
    bt = cfg.setdefault("bt_navigator", {}).setdefault("ros__parameters", {})
    bt.update(odom_topic="/odom", default_nav_to_pose_bt_xml=behavior_tree_path,
              default_nav_through_poses_bt_xml=through_tree_path or behavior_tree_path)
    plugins = bt.get("plugin_lib_names", [])
    if not isinstance(plugins, list) or any(not isinstance(name, str) or not name for name in plugins):
        raise ValueError("bt_navigator.plugin_lib_names must be a list of plugin library names")
    bt["plugin_lib_names"] = list(dict.fromkeys([*plugins, *BT_PLUGIN_LIBRARIES]))
    controller = cfg["controller_server"]["ros__parameters"]
    controller.update(odom_topic="/odom", controller_frequency=CONTROLLER_FREQUENCY)
    follow = controller["FollowPath"]
    vx_min = -safe["reverse_speed"] if supervised_reverse else 0.0
    follow.update(vx_min=vx_min, vx_max=safe["max_speed"], wz_max=safe["max_yaw_rate"],
                  model_dt=MPPI_MODEL_DT, time_steps=MPPI_TIME_STEPS,
                  batch_size=runtime["navigation"]["mppi_batch_size"])
    follow["AckermannConstraints"]["min_turning_r"] = safe["min_turning_radius"]
    follow["ObstaclesCritic"] = dict(follow.get("ObstaclesCritic") or {},
        inflation_radius=INFLATION_RADIUS, cost_scaling_factor=COST_SCALING_FACTOR,
        critical_weight=20.0, repulsion_weight=1.5, collision_margin_distance=safe["collision_margin"], consider_footprint=True)
    follow["PreferForwardCritic"] = dict(follow.get("PreferForwardCritic") or {}, cost_weight=PREFER_FORWARD_WEIGHT)
    controller["progress_checker"] = dict(controller.get("progress_checker") or {},
        required_movement_radius=0.25, movement_time_allowance=20.0)
    grid = cfg["planner_server"]["ros__parameters"]["GridBased"]
    grid.update(allow_unknown=False, reverse_penalty=SMAC_REVERSE_PENALTY,
                motion_model_for_search="REEDS_SHEPP" if supervised_reverse else "DUBIN",
                minimum_turning_radius=safe["min_turning_radius"])
    footprint = str([[safe["footprint_front"], safe["footprint_half_width"]],
                     [safe["footprint_front"], -safe["footprint_half_width"]],
                     [-safe["footprint_rear"], -safe["footprint_half_width"]],
                     [-safe["footprint_rear"], safe["footprint_half_width"]]])
    for name in ("local_costmap", "global_costmap"):
        block = cfg[name][name]["ros__parameters"]
        block.update(robot_base_frame="base_footprint", footprint=footprint)
        block["inflation_layer"] = dict(block.get("inflation_layer") or {},
            plugin="nav2_costmap_2d::InflationLayer", inflation_radius=INFLATION_RADIUS, cost_scaling_factor=COST_SCALING_FACTOR)
        if name == "local_costmap":
            block.update(global_frame="odom", rolling_window=True, track_unknown_space=False,
                         plugins=["voxel_layer", "inflation_layer"])
            block.pop("static_layer", None)
            common = dict(data_type="PointCloud2", min_obstacle_height=0.05,
                          max_obstacle_height=1.8, obstacle_min_range=0.20,
                          obstacle_max_range=3.0, raytrace_min_range=0.20,
                          raytrace_max_range=3.0, observation_persistence=0.0)
            block["voxel_layer"] = dict(block.get("voxel_layer") or {},
                plugin="nav2_costmap_2d::VoxelLayer", observation_sources="obstacle_points depth_clear",
                obstacle_points=dict(common, topic=OBSTACLES_TOPIC, marking=True, clearing=False),
                depth_clear=dict(common, topic=DEPTH_CLOUD_TOPIC, marking=False, clearing=True,
                                 min_obstacle_height=-0.15))
            for legacy in ("scan", "scan2", "depth_points"):
                block["voxel_layer"].pop(legacy, None)
        else:
            block.update(global_frame="map", track_unknown_space=True,
                         plugins=["static_layer", "inflation_layer"])
            block["static_layer"] = dict(plugin="nav2_costmap_2d::StaticLayer", map_topic="/map",
                subscribe_to_updates=True, track_unknown_space=True, map_subscribe_transient_local=True)
            block.pop("obstacle_layer", None)
            block.pop("voxel_layer", None)
    behavior = cfg.setdefault("behavior_server", {}).setdefault("ros__parameters", {})
    behavior.update(global_frame="odom", robot_base_frame="base_footprint",
                    behavior_plugins=["backup", "wait"] if supervised_reverse else ["wait"])
    behavior.pop("spin", None)
    behavior["wait"] = {"plugin": "nav2_behaviors/Wait"}
    if supervised_reverse:
        behavior["backup"] = {"plugin": "nav2_behaviors/BackUp"}
    else:
        behavior.pop("backup", None)
    smoother = cfg.setdefault("velocity_smoother", {}).setdefault("ros__parameters", {})
    smoother.update(max_velocity=[safe["max_speed"], 0.0, safe["max_yaw_rate"]],
                    min_velocity=[vx_min, 0.0, -safe["max_yaw_rate"]],
                    max_accel=[safe["braking_deceleration"], 0.0, 1.0],
                    max_decel=[-safe["braking_deceleration"], 0.0, -1.0],
                    odom_topic="/odom")
    validate_nav_template(cfg)
    return cfg


def write_visual_nav_params(source_path, behavior_tree_path, **kwargs):
    source = yaml.safe_load(Path(source_path).read_text(encoding="utf-8"))
    cfg = build_visual_nav_params(source, behavior_tree_path, **kwargs)
    handle, generated = tempfile.mkstemp(prefix="wheeltec_vision_nav_", suffix=".yaml")
    os.close(handle)
    Path(generated).write_text(yaml.safe_dump(cfg, allow_unicode=True, sort_keys=False), encoding="utf-8")
    return generated
