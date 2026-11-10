"""Scoped launch substitutes for inspecting configuration without a ROS install."""

import importlib
import sys
from pathlib import Path
from types import ModuleType

import pytest


ROOT = Path(__file__).resolve().parents[1]


class RecordedAction:
    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs


class LaunchConfiguration:
    def __init__(self, name):
        self.name = name

    def perform(self, context):
        return context[self.name]


class OpaqueFunction(RecordedAction):
    def execute(self, context):
        return self.kwargs["function"](context)


class RecordedEventHandler(RecordedAction):
    def handle(self, event, context):
        if self.kwargs.get("target_action", event.action) is not event.action:
            return []
        callback = next(self.kwargs[name] for name in ("on_exit", "on_start", "on_shutdown") if name in self.kwargs)
        return callback(event, context)


@pytest.fixture
def launch_utils(monkeypatch):
    """Import the real launch code; record ROS actions and restore imports afterward."""
    names = (
        "ament_index_python", "ament_index_python.packages", "launch", "launch.actions",
        "launch.substitutions", "launch.launch_description_sources", "launch.event_handlers", "launch.events",
        "launch_ros", "launch_ros.actions",
    )
    modules = {name: ModuleType(name) for name in names}
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)
        if "." in name:
            parent, attr = name.rsplit(".", 1)
            setattr(modules[parent], attr, module)
    modules["ament_index_python.packages"].get_package_share_directory = lambda name: str(ROOT)
    modules["launch"].LaunchDescription = type("LaunchDescription", (RecordedAction,), {})
    modules["launch.substitutions"].LaunchConfiguration = LaunchConfiguration
    for name in ("ExecuteProcess", "IncludeLaunchDescription", "TimerAction", "DeclareLaunchArgument", "EmitEvent", "RegisterEventHandler"):
        setattr(modules["launch.actions"], name, type(name, (RecordedAction,), {}))
    modules["launch.actions"].OpaqueFunction = OpaqueFunction
    for name in ("OnProcessExit", "OnProcessStart", "OnShutdown"):
        setattr(modules["launch.event_handlers"], name, type(name, (RecordedEventHandler,), {}))
    modules["launch.events"].Shutdown = type("Shutdown", (RecordedAction,), {})
    modules["launch.launch_description_sources"].PythonLaunchDescriptionSource = type(
        "PythonLaunchDescriptionSource", (RecordedAction,), {},
    )
    modules["launch_ros.actions"].Node = type("Node", (RecordedAction,), {})
    prefix = "wheeltec_vision_nav.launch_"
    for name in list(sys.modules):
        if name.startswith(prefix):
            monkeypatch.delitem(sys.modules, name)
    try:
        yield importlib.import_module("wheeltec_vision_nav.launch_utils")
    finally:
        for name in list(sys.modules):
            if name.startswith(prefix):
                sys.modules.pop(name)
