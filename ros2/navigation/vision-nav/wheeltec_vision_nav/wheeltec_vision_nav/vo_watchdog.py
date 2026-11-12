"""Monotonic final-command gate for mapping and navigation.

No sensor, node output, TF rebroadcast or ROS clock pause can grant motion by
itself. All motion is constrained by runtime_safety.SafetyController.
"""
from __future__ import annotations

import json
import math
import time
from dataclasses import fields

import numpy as np
import rclpy
from action_msgs.msg import GoalStatusArray
from action_msgs.srv import CancelGoal
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from rtabmap_msgs.msg import OdomInfo
from sensor_msgs.msg import Image, Imu, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from wheeltec_vision_nav.runtime_safety import (
    DEGRADED, READY, Cancellation, GoalAcceptanceBoundary, SafetyConfig, SafetyController,
    covariance_valid, normalized_command, observed_forward_range, quaternion_valid,
    swept_clearance, transform_points, transform_valid,
)
from wheeltec_vision_nav.depth_validity import depth_has_valid_returns


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def pose_valid(pose):
    p, q = pose.position, pose.orientation
    return all(math.isfinite(v) for v in (p.x, p.y, p.z)) and quaternion_valid(q.x, q.y, q.z, q.w)


def odom_valid(msg):
    t = msg.twist.twist
    return (pose_valid(msg.pose.pose) and covariance_valid(msg.pose.covariance)
            and covariance_valid(msg.twist.covariance)
            and all(math.isfinite(v) for v in (t.linear.x, t.linear.y, t.linear.z,
                                               t.angular.x, t.angular.y, t.angular.z)))


class VoWatchdog(Node):
    def __init__(self):
        super().__init__("vision_vo_watchdog")
        defaults = SafetyConfig()
        values = {f.name: self.declare_parameter(f.name, getattr(defaults, f.name)).value
                  for f in fields(SafetyConfig)}
        self.safety = SafetyController(SafetyConfig(**values))
        # Retained declaration for old launch clients; freshness/recovery now
        # replaces frame-count-only loss detection.
        self.declare_parameter("consecutive_bad", 5)
        self._cfg = self.safety.cfg
        self._clouds = {"cloud": np.empty((0, 3)), "obstacles": np.empty((0, 3))}
        # Covers a complete 848x480 cloud decimated by four before voxel filtering.
        self._cloud_max_points = int(self.declare_parameter("cloud_max_points", 100000).value)
        self._last_status_time = -1.0
        self._last_ros_time = None
        self._last_wall_time = None
        self._seen_goals = set()
        self._blocked_goals = set()
        self._fault_episode = False
        self._goal_boundary = GoalAcceptanceBoundary()
        self._last_graph_check = -1.0
        self._ownership_valid = False
        self._input_errors = {}
        self._cancellations = {}
        self._cancel_futures = {}
        self._tf = Buffer(cache_time=Duration(seconds=5.0))
        self._listener = TransformListener(self._tf, self)
        qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        transient = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                               reliability=ReliabilityPolicy.RELIABLE)
        self.cmd_pub = self.create_publisher(Twist, self._param("cmd_vel_out", "/cmd_vel_safe"), 1)
        self.status_pub = self.create_publisher(String, self._param("status_topic", "/vision/vo_status"), 5)
        self.health_pub = self.create_publisher(DiagnosticArray, "/vision/health", 5)
        self.detail_pub = self.create_publisher(String, "/vision/health/status", 5)
        self.ready_pub = self.create_publisher(Bool, "/vision/ready", transient)
        self.create_subscription(Twist, self._param("cmd_vel_in", "/cmd_vel"), self._on_cmd, 1)
        self.create_subscription(Odometry, self._param("vo_topic", "/odom_vo"), self._on_vo, qos)
        self.create_subscription(OdomInfo, self._param("info_topic", "/odom_info_lite"), self._on_info, qos)
        self.create_subscription(Odometry, "/odom", self._on_odom, qos)
        self.create_subscription(Image, "/camera/depth/image_rect_raw", self._on_depth, qos)
        self.create_subscription(PointCloud2, "/vision/depth_cloud", lambda m: self._on_cloud(m, "cloud"), qos)
        self.create_subscription(PointCloud2, "/vision/obstacles", lambda m: self._on_cloud(m, "obstacles"), qos)
        self.create_subscription(DiagnosticArray, "/vision/tf_authority", self._on_tf_authority, qos)
        if self._cfg.wheel_backup or self._cfg.supervised_reverse:
            self.create_subscription(Odometry, "/wheel/odom", self._on_wheel, qos)
        if self._cfg.wheel_backup:
            self.create_subscription(Imu, "/vision/imu", self._on_imu, qos)
        if self._cfg.mode == "navigation":
            self.create_subscription(PoseWithCovarianceStamped,
                                     self._param("localization_topic", "/localization_pose"),
                                     self._on_localization, qos)
            for action in ("navigate_to_pose", "navigate_through_poses", "follow_waypoints"):
                self._cancellations[action] = Cancellation()
                self.create_subscription(GoalStatusArray, f"/{action}/_action/status",
                                         self._on_goals, transient)
            self._cancel_clients = {name: self.create_client(CancelGoal, f"/{name}/_action/cancel_goal")
                                    for name in self._cancellations}
        else:
            self._cancel_clients = {}
        self.create_service(Trigger, "/vision/rearm", self._on_rearm)
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(0.05, self._tick, clock=self._steady_clock)

    def _param(self, name, default):
        return self.declare_parameter(name, default).value

    def _note(self, name, msg, valid=True):
        return self.safety.note(name, stamp_ns(msg.header.stamp), time.monotonic(),
                                valid=valid, ros_now_ns=self.get_clock().now().nanoseconds)

    def _on_vo(self, msg):
        self._note("vo", msg, odom_valid(msg) and msg.child_frame_id == "base_footprint")

    def _on_info(self, msg):
        self.safety.note_visual(stamp_ns(msg.header.stamp), time.monotonic(), bool(msg.lost),
                                int(msg.inliers), self.get_clock().now().nanoseconds)

    def _on_odom(self, msg):
        self._note("odom", msg, odom_valid(msg) and msg.header.frame_id == "odom"
                   and msg.child_frame_id == "base_footprint")

    def _on_wheel(self, msg):
        self.safety.note_wheel(stamp_ns(msg.header.stamp), time.monotonic(),
                              msg.twist.twist.linear.x if odom_valid(msg) else math.nan,
                              ros_now_ns=self.get_clock().now().nanoseconds)

    def _on_imu(self, msg):
        self._note("imu", msg, math.isfinite(msg.angular_velocity.z)
                   and math.isfinite(msg.angular_velocity_covariance[8])
                   and msg.angular_velocity_covariance[8] >= 0)

    def _on_localization(self, msg):
        self._note("localization", msg, msg.header.frame_id == "map"
                   and pose_valid(msg.pose.pose) and covariance_valid(msg.pose.covariance)
                   and max(msg.pose.covariance[i] for i in (0, 7, 35)) < 1000)

    def _on_depth(self, msg):
        try:
            # Interleaved emitter-off frames may have weak/no depth. Only valid
            # returns refresh the .3 s deadline; invalid frames never clear it.
            if depth_has_valid_returns(msg):
                self._note("depth", msg)
        except (ValueError, TypeError, BufferError):
            self._note("depth", msg, False)

    def _on_cloud(self, msg, name):
        try:
            if msg.width * msg.height > self._cloud_max_points:
                raise ValueError("cloud too large for safety processing budget")
            rows = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=False))
            points = np.array([tuple(row) for row in rows], dtype=float).reshape(-1, 3)
            if not np.isfinite(points).all() or (name == "cloud" and len(points) < 9):
                raise ValueError("invalid depth cloud")
            transform = self._tf.lookup_transform("odom", msg.header.frame_id,
                                                  Time.from_msg(msg.header.stamp))
            points = transform_points(points, transform.transform)
            if self._note(name, msg):
                self._clouds[name] = points
                self._input_errors.pop(name, None)
            elif not self.safety.sources[name].valid:
                self._remember_error(name, ValueError(
                    f"rejected acquisition stamp {stamp_ns(msg.header.stamp)}; "
                    f"ROS now {self.get_clock().now().nanoseconds}"))
        except (TransformException, ValueError, TypeError, IndexError, KeyError) as error:
            self._note(name, msg, False)
            self._remember_error(name, error)

    def _remember_error(self, source, error):
        if not hasattr(self, "_input_errors"):
            self._input_errors = {}
        previous = self._input_errors.get(source, {})
        self._input_errors[source] = {"type": type(error).__name__, "message": str(error),
                                      "count": previous.get("count", 0) + 1}

    def _on_cmd(self, msg):
        other = (msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y)
        if any(not math.isfinite(v) or abs(v) > 1e-6 for v in other):
            self.safety.stop("nonplanar_or_invalid_command")
        else:
            self.safety.note_command(msg.linear.x, msg.angular.z, time.monotonic())
        # Geometry runs solely on the independent 20Hz wall timer. A command
        # burst must not multiply cloud transforms or starve sensor/TF receipt.
        # Zero/invalid/blocked input still stops immediately without geometry;
        # nonzero commands are considered on the next tick within 50ms.
        if self.safety._cmd == (0.0, 0.0) or self.safety.state == "STOPPED":
            self.cmd_pub.publish(Twist())

    def _on_tf_authority(self, msg):
        self._note("tf_authority", msg, len(msg.status) == 1
                   and msg.status[0].name == "vision/tf_authority"
                   and msg.status[0].level == DiagnosticStatus.OK)

    def _on_goals(self, msg):
        ros_now = self.get_clock().now().nanoseconds
        self._goal_boundary.update(self.safety.state == READY and not self.safety.cancel_required, ros_now)
        for status in msg.status_list:
            goal = bytes(status.goal_info.goal_id.uuid)
            new = goal not in self._seen_goals
            self._seen_goals.add(goal)
            if (status.status in (1, 2) and new and goal not in self._blocked_goals
                    and self._goal_boundary.accepts(stamp_ns(status.goal_info.stamp), ros_now)):
                if self.safety.rearm():
                    self._fault_episode = False

    def _on_rearm(self, _request, response):
        response.success = self._cfg.mode == "mapping" and self.safety.rearm()
        response.message = "mapping rearmed; send a new command" if response.success else (
            "wait for healthy sensors; navigation requires a new accepted goal after cancellation")
        return response

    def _cancel_navigation(self, now):
        if not self.safety.cancel_required:
            return
        if not self._fault_episode:
            self._fault_episode = True
            self._goal_boundary.update(False, self.get_clock().now().nanoseconds)
            self._blocked_goals.update(self._seen_goals)
            self._cancellations = {name: Cancellation() for name in self._cancel_clients}
            self._cancel_futures.clear()
        for name, attempt in self._cancellations.items():
            future = self._cancel_futures.get(name)
            if future is not None and future.done():
                try:
                    response = future.result()
                    attempt.complete(response.return_code == CancelGoal.Response.ERROR_NONE)
                except Exception as error:
                    self.get_logger().warning(f"cancel {name}: {error}")
                    attempt.complete(False)
                self._cancel_futures.pop(name, None)
            if attempt.due(now):
                old = self._cancel_futures.pop(name, None)
                if old is not None:
                    old.cancel()
                attempt.begin(now)
                client = self._cancel_clients[name]
                if client.service_is_ready():
                    self._cancel_futures[name] = client.call_async(CancelGoal.Request())
                else:
                    attempt.complete(False)
        if self._cancellations and all(a.succeeded for a in self._cancellations.values()):
            self.safety.cancel_required = False

    def _tick(self):
        now = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds
        if self._last_ros_time is not None:
            delta_ros = (ros_now - self._last_ros_time) / 1e9
            delta_wall = now - self._last_wall_time
            if delta_ros < 0 or abs(delta_ros - delta_wall) > .5:
                self.safety.clock_reset()
        self._last_ros_time, self._last_wall_time = ros_now, now
        if now - self._last_graph_check >= .2:
            self._last_graph_check = now
            self._ownership_valid = all(len(self.get_publishers_info_by_topic(topic)) == 1
                                         for topic in ("/odom", "/cmd_vel_safe", "/vision/tf_authority"))
        self.safety.note("ownership", ros_now, now, valid=self._ownership_valid)
        clearance, observed, age = 0.0, 0.0, 0.0
        try:
            transform = self._tf.lookup_transform("base_footprint", "odom", Time())
            if not transform_valid(transform.transform):
                raise ValueError("invalid odom transform")
            self.safety.note("tf", stamp_ns(transform.header.stamp), now, ros_now_ns=ros_now)
            self._input_errors.pop("tf", None)
        except (TransformException, ValueError) as error:
            self.safety.note("tf", 1, now, valid=False)
            self._remember_error("tf", error)
            transform = None
        if self._cfg.mode == "navigation":
            try:
                map_transform = self._tf.lookup_transform("map", "odom", Time())
                self.safety.note("map_tf", stamp_ns(map_transform.header.stamp), now,
                                 valid=transform_valid(map_transform.transform), ros_now_ns=ros_now)
                self._input_errors.pop("map_tf", None)
            except (TransformException, ValueError) as error:
                self.safety.note("map_tf", 1, now, valid=False)
                self._remember_error("map_tf", error)
        if transform is not None:
            geometry = {}
            for name in ("cloud", "obstacles"):
                try:
                    geometry[name] = transform_points(self._clouds[name], transform.transform)
                except ValueError as error:
                    self.safety.note(name, 1, now, valid=False)
                    self._remember_error(name, error)
            if len(geometry) == 2:
                try:
                    vx, wz = normalized_command(self._cfg, *self.safety._cmd)
                    clearance = swept_clearance(geometry["obstacles"], self._cfg, vx, wz)
                    observed = observed_forward_range(geometry["cloud"], self._cfg, vx, wz)
                    age = self.safety.geometry_age(now)
                except ValueError as error:
                    self.safety.note("cloud", 1, now, valid=False)
                    self._remember_error("geometry", error)
        decision = self.safety.step(now, clearance_m=clearance, observed_range_m=observed,
                                    observation_age_s=age)
        output = Twist()
        output.linear.x, output.angular.z = decision.vx, decision.wz
        self.cmd_pub.publish(output)
        self._cancel_navigation(now)
        self._goal_boundary.update(self.safety.state == READY and not self.safety.cancel_required, ros_now)
        if now - self._last_status_time >= .2:
            self._last_status_time = now
            detail = self.safety.summary(now)
            detail.update(reason=decision.reason, output_vx=decision.vx, output_wz=decision.wz,
                          input_errors=dict(self._input_errors), tick_wall_s=time.monotonic() - now,
                          clearance_m=clearance if math.isfinite(clearance) else None,
                          observed_range_m=observed, observation_age_s=age,
                          cancellations={name: {"attempts": a.attempts, "succeeded": a.succeeded}
                                         for name, a in self._cancellations.items()})
            self.detail_pub.publish(String(data=json.dumps(detail, allow_nan=False)))
            self.ready_pub.publish(Bool(data=decision.ready))
            legacy = "ok" if decision.state == READY else (
                "lost_wheel_backup" if decision.state == DEGRADED else "lost")
            self.status_pub.publish(String(data=legacy))
            diagnostic = DiagnosticStatus(name="vision/runtime", hardware_id="R680/D435C",
                level=(DiagnosticStatus.OK if decision.ready else
                       DiagnosticStatus.ERROR if decision.state == "STOPPED" else DiagnosticStatus.WARN),
                message=f"{decision.state}: {decision.reason}",
                values=[KeyValue(key=k, value=json.dumps(v, allow_nan=False)) for k, v in detail.items()])
            message = DiagnosticArray(status=[diagnostic])
            message.header.stamp = self.get_clock().now().to_msg()
            self.health_pub.publish(message)

    def destroy_node(self):
        self.cmd_pub.publish(Twist())
        return super().destroy_node()


def main():
    rclpy.init()
    node = VoWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
