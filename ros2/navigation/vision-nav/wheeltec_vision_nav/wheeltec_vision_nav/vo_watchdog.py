#!/usr/bin/env python3
"""Stop the chassis and cancel Nav2 when the pose the car drives on is unreliable.

Without the chassis, visual odometry is the only pose source: a VO loss stops
the car. With the chassis, /odom keeps flowing from wheel odometry during a
VO loss, so the car keeps driving until RTAB-Map has not localized for
`map_stale_s` seconds. A localization is taken from /localization_pose, which
RTAB-Map publishes only when it recognises a place; map->odom on /tf is
re-broadcast continuously and says nothing about how old the correction is.
"""
from __future__ import annotations

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from wheeltec_vision_nav.vo_watchdog_logic import (
    STATUS_LOST,
    WatchdogConfig,
    WatchdogState,
    command_is_stale,
    evaluate_sample,
    note_map_correction,
    note_wheel,
    should_zero_cmd,
    vo_lost,
)

try:
    from rtabmap_msgs.msg import OdomInfo
except ImportError:  # pragma: no cover
    OdomInfo = None

try:
    from action_msgs.srv import CancelGoal
except ImportError:  # pragma: no cover
    CancelGoal = None


class VoWatchdog(Node):
    def __init__(self) -> None:
        super().__init__("vision_vo_watchdog")
        min_inliers = int(self.declare_parameter("min_inliers", 20).value)
        consecutive_bad = int(self.declare_parameter("consecutive_bad", 5).value)
        wheel_backup = bool(self.declare_parameter("wheel_backup", False).value)
        wheel_fresh_s = float(self.declare_parameter("wheel_fresh_s", 0.3).value)
        map_stale_s = float(self.declare_parameter("map_stale_s", 5.0).value)
        self.cmd_timeout_s = float(self.declare_parameter("cmd_timeout_s", 0.5).value)
        localization_topic = str(self.declare_parameter("localization_topic", "/localization_pose").value)
        cmd_in = str(self.declare_parameter("cmd_vel_in", "/cmd_vel").value)
        cmd_out = str(self.declare_parameter("cmd_vel_out", "/cmd_vel_safe").value)
        status_topic = str(self.declare_parameter("status_topic", "/vision/vo_status").value)
        cancel_service = str(
            self.declare_parameter("cancel_service", "/navigate_to_pose/_action/cancel_goal").value
        )
        self._cfg = WatchdogConfig(
            min_inliers=min_inliers,
            consecutive_bad=consecutive_bad,
            wheel_backup=wheel_backup,
            wheel_fresh_s=wheel_fresh_s,
            map_stale_s=map_stale_s,
        )
        self._state = WatchdogState()
        self._last_cmd = Twist()
        self._last_cmd_time = -1.0
        self._had_cmd = False
        self._zeroing = False
        sensor_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(Odometry, "/odom_vo", self._on_odom, sensor_qos)
        if OdomInfo is not None:
            self.create_subscription(OdomInfo, "/odom_info_lite", self._on_info, sensor_qos)
        if wheel_backup:
            self.create_subscription(Odometry, "/wheel/odom", self._on_wheel, sensor_qos)
            self.create_subscription(
                PoseWithCovarianceStamped, localization_topic, self._on_localization, 10
            )
        self.create_subscription(Twist, cmd_in, self._on_cmd, 20)
        self.cmd_pub = self.create_publisher(Twist, cmd_out, 20)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self._cancel_cli = None
        if CancelGoal is not None:
            self._cancel_cli = self.create_client(CancelGoal, cancel_service)
        self.create_timer(0.05, self._tick)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _zero(self) -> bool:
        return should_zero_cmd(self._state, self._cfg, self._now())

    def _publish_status(self) -> None:
        status = self._state.status
        if vo_lost(self._state) and not self._zero():
            status = "lost_wheel_backup"
        self.status_pub.publish(String(data=status))

    def _on_info(self, msg) -> None:
        evaluate_sample(
            self._state,
            self._cfg,
            lost_flag=bool(getattr(msg, "lost", False)),
            inliers=int(getattr(msg, "inliers", 0)),
        )
        self._publish_status()

    def _on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        evaluate_sample(self._state, self._cfg, quat=(q.x, q.y, q.z, q.w))
        self._publish_status()

    def _on_wheel(self, _msg: Odometry) -> None:
        note_wheel(self._state, self._now())

    def _on_localization(self, _msg: PoseWithCovarianceStamped) -> None:
        note_map_correction(self._state, self._now())

    def _on_cmd(self, msg: Twist) -> None:
        self._last_cmd = msg
        self._last_cmd_time = self._now()
        self._had_cmd = True
        if not self._zero():
            self.cmd_pub.publish(msg)

    def _cancel_navigation(self) -> None:
        if self._cancel_cli is None or CancelGoal is None:
            return
        if not self._cancel_cli.service_is_ready():
            return
        self._cancel_cli.call_async(CancelGoal.Request())

    def _tick(self) -> None:
        if self._zero():
            self._last_cmd = Twist()
            self._had_cmd = False
            self.cmd_pub.publish(Twist())
            if not self._zeroing:
                self._zeroing = True
                self._cancel_navigation()
            if self._state.status != STATUS_LOST:
                self._state.status = STATUS_LOST
            self._publish_status()
            return
        if self._zeroing:
            self._zeroing = False
        if not self._had_cmd:
            return
        if command_is_stale(self._last_cmd_time, self._now(), self.cmd_timeout_s):
            # The controller stopped publishing: never keep driving on an old command.
            self._had_cmd = False
            self.cmd_pub.publish(Twist())
            return
        self.cmd_pub.publish(self._last_cmd)


def main() -> None:
    rclpy.init()
    node = VoWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
