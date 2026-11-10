"""Replay A/B/C in an isolated ROS domain. Nothing here controls the chassis."""
import json
import os
import platform
from pathlib import Path
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, OpaqueFunction,
                            RegisterEventHandler, EmitEvent, TimerAction)
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from wheeltec_vision_nav.vins_experiment import validate_run, playback_command, experiment_nodes, digest, bag_fingerprint
from wheeltec_vision_nav.runtime_config import load_runtime_config


def generate_launch_description():
    def build(context):
        def arg(key):
            return LaunchConfiguration(key).perform(context)
        mode, bag = arg("mode"), Path(arg("bag")).expanduser().resolve()
        config, audit = arg("config"), arg("audit")
        validate_run(mode, bag, config or None, audit or None)
        # A separate domain prevents replay /clock, /odom and /tf reaching a live robot.
        if int(os.environ.get("ROS_DOMAIN_ID", "0")) == 0:
            raise ValueError("Set a dedicated unused ROS_DOMAIN_ID (for example 87) before replay")
        output = Path(arg("output")).expanduser().resolve()
        output.mkdir(parents=True, exist_ok=False)
        share = get_package_share_directory("wheeltec_vision_nav")
        runtime_path = arg("runtime_config")
        runtime = load_runtime_config(runtime_path) if runtime_path else None
        metric_min_inliers = runtime["safety"]["min_inliers"] if mode == "baseline" and runtime else 10
        specs = experiment_nodes(mode, share, str(Path(config).resolve()) if config else "", runtime=runtime)
        cmd = playback_command(bag, float(arg("rate")))
        config_files = [Path(audit)] if audit else []
        if runtime_path:
            config_files.append(Path(runtime_path).expanduser().resolve())
        if config:
            from wheeltec_vision_nav.vins_audit import load_calibration
            cfg = load_calibration(config)
            config_files.extend([Path(config), *(Path(config).resolve().parent / cfg[k]
                                               for k in ("cam0_calib", "cam1_calib"))])
        prefix = Path(get_package_prefix("rtabmap_odom"))
        manifest = {"schema_version": 1, "mode": mode, "bag_uri": str(bag),
            "metric_min_inliers": metric_min_inliers,
            "runtime_config_sha256": digest(Path(runtime_path).expanduser()) if runtime_path else None,
            "runtime_profile_effect": "baseline feature/quality overrides" if mode == "baseline" else
                                      "VINS estimator uses audited config; recorded images are not resampled",
            "bag_metadata_sha256": digest(bag / "metadata.yaml"), "playback_command": cmd,
            "bag_files": bag_fingerprint(bag),
            "ros_domain_id": os.environ["ROS_DOMAIN_ID"], "platform": platform.platform(),
            "python_version": platform.python_version(), "rtabmap_odom_prefix": str(prefix), "nodes": specs,
            "config_sha256": digest(config) if config else None,
            "audit_sha256": digest(audit) if audit else None,
            "config_artifacts_sha256": {str(p.resolve()): digest(p) for p in config_files},
            "runtime_files_sha256": {str(p): digest(p) for p in
                [Path(__file__), *sorted((Path(share) / "config").glob("*.yaml"))]},
            "overlay_evidence_sha256": {p.name: digest(p) for p in
                sorted((prefix.parent / "evidence").glob("*")) if p.is_file()},
            "package_sources_sha256": {p.name: digest(p) for p in
                sorted(Path(__import__("wheeltec_vision_nav").__file__).parent.glob("*.py"))}}
        (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
        nodes = [Node(**spec) for spec in specs]
        nodes.append(Node(package="wheeltec_vision_nav", executable="vins_monitor", name="vins_monitor",
                          output="screen", parameters=[{"use_sim_time": True, "output_dir": str(output),
                            "min_inliers": metric_min_inliers,
                            "resource_root_pid": os.getpid(),
                            "odom_topic": "/odom" if mode == "baseline" else "/odom_vins",
                            "info_topic": "/odom_info_lite" if mode == "baseline" else "/vins/odom_info"}]))
        play = ExecuteProcess(cmd=cmd, output="screen")
        return [*nodes, RegisterEventHandler(OnProcessExit(target_action=play, on_exit=[
            TimerAction(period=2., actions=[EmitEvent(event=Shutdown(reason="bag replay finished"))])])), play]

    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="stereo", choices=["baseline", "stereo", "vio"]),
        DeclareLaunchArgument("bag"), DeclareLaunchArgument("output"),
        DeclareLaunchArgument("config", default_value=""), DeclareLaunchArgument("audit", default_value=""),
        DeclareLaunchArgument("runtime_config", default_value=""),
        DeclareLaunchArgument("rate", default_value="1.0"), OpaqueFunction(function=build)])
