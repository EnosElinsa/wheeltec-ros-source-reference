#!/usr/bin/env python3
"""Relay wheel or visual odometry to /odom and TF."""
from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage

from wheeltec_vision_nav.hold_odom_tf_logic import (
    OdomSample,
    OdomStitch,
    diagonal_covariance,
    gated_odom_sample,
    odom_sample_is_valid,
    stitch_odom_sample,
    wheel_relative_sample,
    wheel_source_is_fresh,
)


MAP_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)


def _sample_from_msg(msg: Odometry) -> OdomSample:
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    return OdomSample(
        stamp_sec=int(msg.header.stamp.sec),
        stamp_nsec=int(msg.header.stamp.nanosec),
        x=p.x,
        y=p.y,
        z=p.z,
        qx=q.x,
        qy=q.y,
        qz=q.z,
        qw=q.w,
        cov0=float(msg.pose.covariance[0]),
        cov5=float(msg.pose.covariance[35]),
    )


def _msg_from_sample(sample: OdomSample, template: Odometry | None = None) -> Odometry:
    msg = Odometry()
    if template is not None:
        msg.twist = template.twist
    msg.header.stamp.sec = sample.stamp_sec
    msg.header.stamp.nanosec = sample.stamp_nsec
    msg.header.frame_id = "odom"
    msg.child_frame_id = "base_footprint"
    msg.pose.pose.position.x = sample.x
    msg.pose.pose.position.y = sample.y
    msg.pose.pose.position.z = 0.0
    msg.pose.pose.orientation.x = sample.qx
    msg.pose.pose.orientation.y = sample.qy
    msg.pose.pose.orientation.z = sample.qz
    msg.pose.pose.orientation.w = sample.qw
    # Full positive diagonal: RTAB-Map inverts this matrix in localization mode
    # and aborts on a singular (zero yaw variance) covariance.
    msg.pose.covariance = diagonal_covariance(sample)
    return msg


def _tf_from_sample(sample: OdomSample) -> TransformStamped:
    t = TransformStamped()
    t.header.stamp.sec = sample.stamp_sec
    t.header.stamp.nanosec = sample.stamp_nsec
    t.header.frame_id = "odom"
    t.child_frame_id = "base_footprint"
    t.transform.translation.x = sample.x
    t.transform.translation.y = sample.y
    t.transform.translation.z = 0.0
    t.transform.rotation.x = sample.qx
    t.transform.rotation.y = sample.qy
    t.transform.rotation.z = sample.qz
    t.transform.rotation.w = sample.qw
    return t


def _identity_map_odom(stamp) -> TransformStamped:
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = "map"
    t.child_frame_id = "odom"
    t.transform.rotation.w = 1.0
    return t


class HoldOdomTf(Node):
    def __init__(self) -> None:
        super().__init__("vision_hold_odom_tf")
        self.last_valid: OdomSample | None = None
        self.last_valid_time = 0.0
        self.wheel_origin: OdomSample | None = None
        self.last_wheel_time = 0.0
        self.last_wheel_stamp: tuple[int, int] | None = None
        self.seed_map_tf = True
        self.stitch = OdomStitch()
        self.last_tf = _tf_from_sample(
            OdomSample(0, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1e-5)
        )
        vo_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        odom_pub_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(Odometry, "/odom_vo", self._on_odom, vo_qos)
        self.create_subscription(Odometry, "/wheel/odom", self._on_wheel, vo_qos)
        wheel_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(Odometry, "/wheel/odom", self._on_wheel, wheel_qos)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, MAP_QOS)
        self.odom_pub = self.create_publisher(Odometry, "/odom", odom_pub_qos)
        self.tf_pub = self.create_publisher(TFMessage, "/tf", 50)
        self.create_timer(0.05, self._tick)

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _wheel_fresh(self) -> bool:
        return wheel_source_is_fresh(self._now(), self.last_wheel_time)

    def _on_map(self, _msg: OccupancyGrid) -> None:
        self.seed_map_tf = False

    def _bundle(self, stamp) -> list[TransformStamped]:
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.last_tf.header.frame_id
        t.child_frame_id = self.last_tf.child_frame_id
        t.transform = self.last_tf.transform
        out = [t]
        if self.seed_map_tf:
            out.append(_identity_map_odom(stamp))
        return out

    def _broadcast_tf(self, stamp) -> None:
        self.tf_pub.publish(TFMessage(transforms=self._bundle(stamp)))

    def _publish_sample(self, sample: OdomSample, template: Odometry | None = None) -> None:
        odom = _msg_from_sample(sample, template)
        self.odom_pub.publish(odom)
        t = _tf_from_sample(sample)
        self.last_tf = t
        self.tf_pub.publish(TFMessage(transforms=self._bundle(t.header.stamp)))
        self._broadcast_tf(self.get_clock().now().to_msg())

    def _on_wheel(self, msg: Odometry) -> None:
        stamp = (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))
        if stamp == self.last_wheel_stamp:
            return
        self.last_wheel_stamp = stamp
        incoming = _sample_from_msg(msg)
        self.wheel_origin, published = wheel_relative_sample(self.wheel_origin, incoming)
        self.last_valid = published
        self.last_valid_time = self._now()
        self.last_wheel_time = self.last_valid_time
        self._publish_sample(published, msg)

    def _on_odom(self, msg: Odometry) -> None:
        if self._wheel_fresh():
            return
        incoming = _sample_from_msg(msg)
        if odom_sample_is_valid(incoming):
            published = stitch_odom_sample(self.stitch, incoming)
            self.last_valid = published
            self.last_valid_time = self._now()
            self._publish_sample(published, msg)
            return
        gated = gated_odom_sample(self.last_valid, incoming)
        template = msg
        if self.last_valid is not None:
            template = _msg_from_sample(self.last_valid, msg)
        self._publish_sample(gated, template)

    def _tick(self) -> None:
        self._broadcast_tf(self.get_clock().now().to_msg())


def main() -> None:
    rclpy.init()
    node = HoldOdomTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
