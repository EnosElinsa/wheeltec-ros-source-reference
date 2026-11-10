"""Leased startup, health readiness and scoped teardown of visual navigation."""
import atexit
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart, OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from wheeltec_vision_nav.launch_utils import (
    camera_stack, capture_runtime_node, launch_flag, model_actions, navigation_nodes,
    package_file, runtime_monitor_node, visual_runtime_actions, write_visual_nav_params,
)
from wheeltec_vision_nav.runtime_config import load_runtime_config, resolve_odom_source, validate_runtime_config
from wheeltec_vision_nav.startup import ReadinessDeadline, ResourceLeases, database_resource, prepare_database


def _arg(value):
    return SimpleNamespace(perform=lambda context: value)


def _write_effective(cfg):
    handle, name = tempfile.mkstemp(prefix="wheeltec_runtime_", suffix=".yaml")
    os.close(handle)
    Path(name).write_text(yaml.safe_dump(cfg, sort_keys=False), encoding="utf-8")
    return name


def supervised_processes(leases, critical_actions, ready_node, nav_actions, generated_paths):
    """Keep leases until all started child processes exit, including during shutdown."""
    active = set()
    state = {"shutdown": False}
    critical = set(critical_actions) | set(nav_actions)
    camera_executables = {"realsense2_camera_node", "realsense2_camera_node.exe"}

    def cleanup():
        leases.release()
        for path in generated_paths:
            Path(path).unlink(missing_ok=True)

    atexit.register(cleanup)

    def started(event, context):
        active.add(event.action)
        return []

    def shutdown(event, context):
        state["shutdown"] = True
        if not active:
            cleanup()
        return []

    def exited(event, context):
        active.discard(event.action)
        if state["shutdown"]:
            if not active:
                cleanup()
            return []
        details = getattr(event.action, "process_details", {}) or {}
        command = details.get("cmd", [])
        camera = any(Path(str(part)).name in camera_executables for part in command)
        if event.action in critical or camera:
            name = details.get("name", "critical visual navigation process")
            return [EmitEvent(event=Shutdown(reason=f"{name} exited with code {event.returncode}; motion is stopped"))]
        return []

    def ready_exited(event, context):
        if state["shutdown"] or getattr(context, "is_shutdown", False):
            return []
        if event.returncode == 0:
            return nav_actions
        return [EmitEvent(event=Shutdown(reason="Readiness timed out or failed waiting for /vision/ready; inspect /vision/health/status"))]

    handlers = [RegisterEventHandler(OnProcessStart(on_start=started)),
                RegisterEventHandler(OnProcessExit(on_exit=exited)),
                RegisterEventHandler(OnShutdown(on_shutdown=shutdown))]
    if ready_node is not None:
        handlers.append(RegisterEventHandler(OnProcessExit(target_action=ready_node, on_exit=ready_exited)))
    return handlers


def generate_visual_launch_description(*, localization):
    def build(context, *args, **kwargs):
        def value(name):
            return LaunchConfiguration(name).perform(context)

        cfg = load_runtime_config(value("runtime_config"))
        readiness_timeout = float(value("ready_timeout_s"))
        ReadinessDeadline(readiness_timeout)  # Validate before any database wipe.
        start_base = launch_flag(value("start_base"))
        use_guess = launch_flag(value("use_wheel_guess"))
        source = resolve_odom_source(value("odom_source"), start_base=start_base,
                                     hold_odom=launch_flag(value("hold_odom")))
        reverse_arg = value("supervised_reverse")
        reverse = launch_flag(reverse_arg) if reverse_arg else cfg["safety"]["supervised_reverse"]
        cfg["safety"]["supervised_reverse"] = reverse
        car_mode = value("car_mode") or cfg["robot"]["car_mode"]
        if car_mode != "senior_akm":
            raise ValueError("Only the fixed senior_akm hardware is supported")
        for arg, section, key, convert in (
            ("camera_profile", "perception", "camera_profile", str),
            ("max_features", "perception", "max_features", int),
            ("cloud_decimation", "perception", "cloud_decimation", int),
            ("mppi_batch_size", "navigation", "mppi_batch_size", int),
            ("odom_tf_linear_variance", "perception", "odom_tf_linear_variance", float),
            ("odom_tf_angular_variance", "perception", "odom_tf_angular_variance", float),
        ):
            if value(arg):
                cfg[section][key] = convert(value(arg))
        cfg["robot"].update(odom_source=source, start_base=start_base, use_wheel_guess=use_guess,
                              camera_xyz=value("camera_xyz"), camera_rpy=value("camera_rpy"))
        validate_runtime_config(cfg)
        database = str(Path(value("database_path")).expanduser().resolve())
        resources = ["camera:" + cfg["robot"]["camera_resource"], database_resource(database)]
        if start_base:
            resources.append("chassis:" + cfg["robot"]["serial_port"])
        leases = ResourceLeases(value("lease_dir")).acquire(resources)
        generated = []
        try:
            # Validate files and configuration before any sensor or actuator starts.
            if localization and not Path(database).is_file():
                raise FileNotFoundError(f"Saved RTAB-Map database does not exist: {database}")
            nav_params = ""
            if localization:
                nav_params = write_visual_nav_params(value("nav_params_file"),
                    supervised_reverse=reverse, runtime=cfg)
                generated.append(nav_params)
            models = model_actions(context, _arg(car_mode), _arg(value("wheeltec_param_yaml")),
                                   _arg(value("robot_model_yaml")), _arg(value("camera_xyz")),
                                   _arg(value("camera_rpy")))
            # robot_state_publisher uses our generated, process-owned URDF.
            # Keep it through child termination, then remove only this launch's file.
            if models:
                # The model helper also records its temp file on the action.
                generated.extend(getattr(models[0], "vision_generated_files", []))
            backup = prepare_database(database, localization=localization, wipe=launch_flag(value("wipe_db")))
            if backup:
                cfg["robot"]["database_backup"] = backup
            effective = _write_effective(cfg)
            generated.append(effective)
            runtime = visual_runtime_actions(database, localization, start_base,
                use_wheel_guess=use_guess, odom_source=source, runtime=cfg, supervised_reverse=reverse)
            nav = navigation_nodes(nav_params, autostart=launch_flag(value("autostart"))) if localization else []
            waiter = Node(package="wheeltec_vision_nav", executable="wait_ready", name="vision_wait_ready",
                          output="screen", parameters=[{"timeout_s": readiness_timeout}])
            observers = []
            if launch_flag(value("monitor")):
                observers.append(runtime_monitor_node(value("output_dir"), effective, nav_params, database))
            if launch_flag(value("capture")):
                observers.append(capture_runtime_node(value("capture_dir")))
            stack = camera_stack([*models, *runtime, *observers],
                stereo=value("record_stereo"), camera_profile=cfg["perception"]["camera_profile"])
            critical = [action for action in stack if action not in observers]
            handlers = supervised_processes(leases, critical, waiter, nav, generated)
            return [*handlers, *stack,
                    *([waiter] if waiter is not None else [])]
        except BaseException:
            leases.release()
            for path in generated:
                Path(path).unlink(missing_ok=True)
            raise

    defaults = {
        "runtime_config": package_file("wheeltec_vision_nav", "config", "runtime.yaml"),
        "car_mode": "",
        "wheeltec_param_yaml": package_file("turn_on_wheeltec_robot", "config", "wheeltec_param.yaml"),
        "robot_model_yaml": package_file("turn_on_wheeltec_robot", "config", "robot_model.yaml"),
        "database_path": os.path.expanduser("~/.ros/vision_rtabmap_mapping.db"),
        "nav_params_file": package_file("wheeltec_vision_nav", "config", "visual_nav_baseline.yaml"),
        "start_base": "false", "autostart": "false", "hold_odom": "false",
        "odom_source": "auto", "use_wheel_guess": "false", "wipe_db": "false",
        "supervised_reverse": "", "record_stereo": "false", "camera_xyz": "", "camera_rpy": "",
        "camera_profile": "", "max_features": "", "cloud_decimation": "", "mppi_batch_size": "",
        "odom_tf_linear_variance": "", "odom_tf_angular_variance": "", "ready_timeout_s": "30.0",
        "lease_dir": os.path.expanduser("~/.ros/wheeltec_vision_nav/leases"),
        "monitor": "true", "output_dir": os.path.expanduser("~/vision_runs"),
        "capture": "false", "capture_dir": os.path.expanduser("~/vision_captures"),
    }
    return LaunchDescription([*(DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()),
                              OpaqueFunction(function=build)])
