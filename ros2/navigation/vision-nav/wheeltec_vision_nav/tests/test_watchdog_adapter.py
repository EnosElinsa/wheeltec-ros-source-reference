"""Execute the production ROS callbacks with transport substitutes, without ROS."""
import importlib.util
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace as NS

import numpy as np
import pytest

from wheeltec_vision_nav.runtime_safety import (
    GoalAcceptanceBoundary, READY, SafetyConfig, SafetyController, STOPPED,
)


@pytest.fixture
def adapter(monkeypatch):
    names = ("rclpy", "rclpy.clock", "rclpy.duration", "rclpy.node", "rclpy.qos", "rclpy.time",
             "action_msgs", "action_msgs.msg", "action_msgs.srv", "diagnostic_msgs", "diagnostic_msgs.msg",
             "geometry_msgs", "geometry_msgs.msg", "nav_msgs", "nav_msgs.msg", "rtabmap_msgs", "rtabmap_msgs.msg",
             "sensor_msgs", "sensor_msgs.msg", "sensor_msgs_py", "std_msgs", "std_msgs.msg",
             "std_srvs", "std_srvs.srv", "tf2_ros", "tf2_msgs", "tf2_msgs.msg")
    modules = {name: ModuleType(name) for name in names}
    for name, module in modules.items():
        monkeypatch.setitem(sys.modules, name, module)
    class Message(NS):
        OK, WARN, ERROR = 0, 1, 2
        def __init__(self, **kwargs):
            super().__init__(header=NS(stamp=None), **kwargs)
    class Twist:
        def __init__(self):
            self.linear = NS(x=0., y=0., z=0.)
            self.angular = NS(x=0., y=0., z=0.)
    class Time:
        @staticmethod
        def from_msg(msg):
            return msg
    for package, symbols in {
        "action_msgs.msg": ["GoalStatusArray"], "action_msgs.srv": ["CancelGoal"],
        "diagnostic_msgs.msg": ["DiagnosticArray", "DiagnosticStatus", "KeyValue"],
        "geometry_msgs.msg": ["PoseWithCovarianceStamped"], "nav_msgs.msg": ["Odometry"],
        "rtabmap_msgs.msg": ["OdomInfo"], "sensor_msgs.msg": ["Image", "Imu", "PointCloud2"],
        "std_msgs.msg": ["Bool", "String"], "std_srvs.srv": ["Trigger"], "tf2_msgs.msg": ["TFMessage"],
        "rclpy.clock": ["Clock", "ClockType"], "rclpy.duration": ["Duration"],
        "rclpy.qos": ["DurabilityPolicy", "QoSProfile", "ReliabilityPolicy"],
        "tf2_ros": ["Buffer", "TransformListener"],
    }.items():
        for symbol in symbols:
            setattr(modules[package], symbol, Message)
    modules["geometry_msgs.msg"].Twist = Twist
    modules["rclpy.time"].Time = Time
    modules["rclpy.node"].Node = object
    modules["tf2_ros"].TransformException = type("TransformException", (Exception,), {})
    modules["sensor_msgs_py"].point_cloud2 = NS(read_points=lambda msg, **kwargs: msg.points)
    path = Path(__file__).parents[1] / "wheeltec_vision_nav" / "vo_watchdog.py"
    spec = importlib.util.spec_from_file_location("runtime_watchdog_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def stamp(ns):
    return NS(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def transform(ns, x=0.):
    return NS(header=NS(stamp=stamp(ns)),
              transform=NS(translation=NS(x=x, y=0., z=0.), rotation=NS(x=0., y=0., z=0., w=1.)))


def refresh(node, now, omit=()):
    node.now = now
    node.ros_now = int((1000 + now) * 1e9)
    for name in node.safety._required():
        if name not in omit:
            node.safety.note(name, node.ros_now, now)
    node.safety.note("vo", node.ros_now, now)
    node.safety.note_visual(node.ros_now, now, False, 40)


def node_for(adapter):
    node = adapter.VoWatchdog.__new__(adapter.VoWatchdog)
    node.safety = SafetyController(SafetyConfig(supervised_reverse=True))
    node._cfg = node.safety.cfg
    for index in range(14):
        refresh(node, index / 10)
        node.safety.step(node.now)
    assert node.safety.state == READY
    adapter.time = NS(monotonic=lambda: node.now)
    node.get_clock = lambda: NS(now=lambda: NS(nanoseconds=node.ros_now, to_msg=lambda: stamp(node.ros_now)))
    node.get_publishers_info_by_topic = lambda topic: [object()]
    node._last_ros_time = node._last_wall_time = None
    node._last_graph_check = node._last_status_time = -1.
    node._tf_authors = {}
    node._seen_goals, node._blocked_goals = set(), set()
    node._goal_boundary = GoalAcceptanceBoundary()
    node._fault_episode = False
    node._cancellations, node._cancel_clients, node._cancel_futures = {}, {}, {}
    node._cloud_max_points = 100000
    node._clouds = {
        "cloud": np.array([(x, y / 100., 0.) for x in (1., 1.5, 2.) for y in range(-200, 201, 5)]),
        "obstacles": np.empty((0, 3)),
    }
    node._tf = NS(lookup_transform=lambda *args: transform(node.ros_now))
    for publisher in ("cmd_pub", "detail_pub", "ready_pub", "status_pub", "health_pub"):
        messages = []
        setattr(node, publisher, NS(messages=messages, publish=messages.append))
    return node


def goal(stamp_ns, identifier=1):
    return NS(status_list=[NS(status=2, goal_info=NS(stamp=stamp(stamp_ns),
                           goal_id=NS(uuid=[identifier] * 16)))])


def test_tick_checks_the_clamped_trajectory_before_publishing(adapter):
    node = node_for(adapter)
    node._clouds["obstacles"] = np.array([(.35, -.2, .2)])
    node.safety.note_command(.3, 1., node.now)
    node._tick()
    assert node.safety.state == STOPPED
    assert node.cmd_pub.messages[-1].linear.x == 0.


@pytest.mark.parametrize("reverse", [False, True])
@pytest.mark.parametrize("source", ["odom_tf", "cloud_tf", "cloud_points"])
def test_nonfinite_geometry_invalidates_evidence_before_forward_or_reverse_motion(adapter, reverse, source):
    node = node_for(adapter)
    refresh(node, node.now + .05)
    if source == "odom_tf":
        node._tf.lookup_transform = lambda target, *args: transform(node.ros_now, math.inf if target == "base_footprint" else 0.)
    else:
        if source == "cloud_tf":
            node._tf.lookup_transform = lambda target, *args: transform(node.ros_now, math.inf if target == "odom" else 0.)
        points = [(math.inf if source == "cloud_points" else 1., 0., .2)] * 9
        msg = NS(header=NS(stamp=stamp(node.ros_now), frame_id="camera"), width=9, height=1, points=points)
        node._on_cloud(msg, "cloud")
        assert not node.safety.sources["cloud"].valid
    node.safety.note_command(-.08 if reverse else .2, 0., node.now)
    node._tick()
    assert node.safety.state == STOPPED and node.cmd_pub.messages[-1].linear.x == 0.
    if source == "odom_tf":
        assert not node.safety.sources["tf"].valid


def test_empty_obstacle_cloud_is_valid_and_all_nan_obstacle_cloud_is_invalid(adapter):
    node = node_for(adapter)
    refresh(node, node.now + .05, omit=("obstacles",))
    msg = NS(header=NS(stamp=stamp(node.ros_now), frame_id="base_footprint"), width=0, height=1, points=[])
    node._on_cloud(msg, "obstacles")
    assert node.safety.sources["obstacles"].valid
    assert node._clouds["obstacles"].shape == (0, 3)
    msg.points, msg.width = [(math.nan, math.nan, math.nan)], 1
    node._on_cloud(msg, "obstacles")
    assert not node.safety.sources["obstacles"].valid


def test_frozen_map_tf_cannot_be_refreshed_by_new_odom_tf_or_localization(adapter):
    node = node_for(adapter)
    frozen = node.ros_now
    node._tf.lookup_transform = lambda target, *args: transform(frozen if target == "map" else node.ros_now)
    for _ in range(52):
        refresh(node, node.now + .1, omit=("tf", "map_tf"))
        node.safety.note_command(.2, 0., node.now)
        node._tick()
    assert node.safety.state == STOPPED
    assert node.safety.reason == "map_tf_invalid_or_stale"
    assert node.safety.sources["tf"].stamp_ns > frozen


def test_delayed_unseen_prefault_goal_cannot_rearm_after_recovery(adapter):
    node = node_for(adapter)
    old_acceptance = node.ros_now
    node.safety.stop("depth_lost")
    node._cancel_navigation(node.now)
    # Complete cancellation, then drive actual recovery with healthy fresh samples.
    node.safety.cancel_required = False
    for _ in range(15):
        refresh(node, node.now + .1)
        node._tick()
    assert node.safety.state == READY and node.safety.new_goal_required
    node._on_goals(goal(old_acceptance))
    assert node.safety.new_goal_required
    new_acceptance = node.ros_now + 1
    refresh(node, node.now + .05)
    node._on_goals(goal(new_acceptance, identifier=2))
    assert not node.safety.new_goal_required
    assert node.safety._cmd == (0., 0.)  # A fresh command is still required.


def test_tick_braking_uses_stale_obstacles_despite_fresh_cloud(adapter):
    node = node_for(adapter)
    node.safety.sources.pop("obstacles")
    node.safety.note("obstacles", node.ros_now - 200_000_000, node.now, ros_now_ns=node.ros_now)
    node._clouds["obstacles"] = np.array([(.45, 0., .2)])
    refresh(node, node.now + .09, omit=("obstacles",))
    node.safety.note_command(.3, 0., node.now)
    node._tick()
    age = node.safety.geometry_age(node.now)
    assert age == pytest.approx(.29)
    protected = node.cmd_pub.messages[-1].linear.x
    optimistic = node.safety.step(node.now, clearance_m=.15, observed_range_m=2., observation_age_s=0.)
    assert 0 < protected < optimistic.vx
