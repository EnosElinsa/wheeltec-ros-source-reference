#!/usr/bin/env python3
"""Publish emitter-off infra1 paired with the nearest emitter-on depth."""
from __future__ import annotations

from collections import deque
from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from wheeltec_vision_nav.emitter_pair_logic import (
    FrameRef,
    find_pair,
    nearest_emitter,
    output_stamp_is_new,
    parse_emitter_mode,
)

try:
    from realsense2_camera_msgs.msg import Metadata
except ImportError:  # pragma: no cover - optional on the robot overlay
    Metadata = None


def _stamp_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class EmitterPair(Node):
    def __init__(self) -> None:
        super().__init__("vision_emitter_pair")
        self.max_interval = float(self.declare_parameter("max_interval", 0.12).value)
        self.min_interval = float(self.declare_parameter("min_interval", 0.02).value)
        buffer_size = int(self.declare_parameter("buffer_size", 12).value)
        infra_topic = str(self.declare_parameter("infra_topic", "/camera/infra1/image_rect_raw").value)
        infra_info_topic = str(
            self.declare_parameter("infra_info_topic", "/camera/infra1/camera_info").value
        )
        depth_topic = str(self.declare_parameter("depth_topic", "/camera/depth/image_rect_raw").value)
        depth_info_topic = str(
            self.declare_parameter("depth_info_topic", "/camera/depth/camera_info").value
        )
        infra_meta_topic = str(self.declare_parameter("infra_meta_topic", "/camera/infra1/metadata").value)
        depth_meta_topic = str(self.declare_parameter("depth_meta_topic", "/camera/depth/metadata").value)
        ir_out = str(
            self.declare_parameter("infra_out_topic", "/vision/paired/infra1/image_rect_raw").value
        )
        ir_info_out = str(
            self.declare_parameter("infra_info_out_topic", "/vision/paired/infra1/camera_info").value
        )
        depth_out = str(
            self.declare_parameter("depth_out_topic", "/vision/paired/depth/image_rect_raw").value
        )
        depth_info_out = str(
            self.declare_parameter("depth_info_out_topic", "/vision/paired/depth/camera_info").value
        )
        self._ir: deque[FrameRef] = deque(maxlen=buffer_size)
        self._depth: deque[FrameRef] = deque(maxlen=buffer_size)
        self._ir_meta: deque[tuple[float, int]] = deque(maxlen=buffer_size * 2)
        self._depth_meta: deque[tuple[float, int]] = deque(maxlen=buffer_size * 2)
        self._ir_info: CameraInfo | None = None
        self._depth_info: CameraInfo | None = None
        self._last_output_stamp: float | None = None
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(Image, infra_topic, self._on_ir, qos)
        self.create_subscription(CameraInfo, infra_info_topic, self._on_ir_info, qos)
        self.create_subscription(Image, depth_topic, self._on_depth, qos)
        self.create_subscription(CameraInfo, depth_info_topic, self._on_depth_info, qos)
        if Metadata is not None:
            self.create_subscription(Metadata, infra_meta_topic, self._on_ir_metadata, qos)
            self.create_subscription(Metadata, depth_meta_topic, self._on_depth_metadata, qos)
        self.ir_pub = self.create_publisher(Image, ir_out, qos)
        self.ir_info_pub = self.create_publisher(CameraInfo, ir_info_out, qos)
        self.depth_pub = self.create_publisher(Image, depth_out, qos)
        self.depth_info_pub = self.create_publisher(CameraInfo, depth_info_out, qos)

    def _on_ir_info(self, msg: CameraInfo) -> None:
        self._ir_info = msg

    def _on_depth_info(self, msg: CameraInfo) -> None:
        self._depth_info = msg

    def _on_ir_metadata(self, msg) -> None:
        mode = parse_emitter_mode(getattr(msg, "json_data", None))
        if mode is None:
            return
        self._ir_meta.append((_stamp_sec(msg.header.stamp), mode))

    def _on_depth_metadata(self, msg) -> None:
        mode = parse_emitter_mode(getattr(msg, "json_data", None))
        if mode is None:
            return
        self._depth_meta.append((_stamp_sec(msg.header.stamp), mode))

    def _on_ir(self, msg: Image) -> None:
        stamp = _stamp_sec(msg.header.stamp)
        emitter = nearest_emitter(self._ir_meta, stamp)
        self._ir.append(FrameRef(stamp=stamp, emitter=emitter, payload=msg, info=self._ir_info))
        self._publish_pair()

    def _on_depth(self, msg: Image) -> None:
        stamp = _stamp_sec(msg.header.stamp)
        emitter = nearest_emitter(self._depth_meta, stamp)
        self._depth.append(FrameRef(stamp=stamp, emitter=emitter, payload=msg, info=self._depth_info))
        self._publish_pair()

    def _publish_pair(self) -> None:
        pair = find_pair(
            self._ir,
            self._depth,
            max_interval=self.max_interval,
            min_interval=self.min_interval,
        )
        if pair is None:
            return
        ir, depth = pair
        ir_img = ir.payload
        depth_img = depth.payload
        if ir_img is None or depth_img is None:
            return
        if not output_stamp_is_new(self._last_output_stamp, ir.stamp):
            return
        self._last_output_stamp = ir.stamp
        depth_out = Image()
        depth_out.header = deepcopy(ir_img.header)
        depth_out.header.frame_id = depth_img.header.frame_id
        depth_out.height = depth_img.height
        depth_out.width = depth_img.width
        depth_out.encoding = depth_img.encoding
        depth_out.is_bigendian = depth_img.is_bigendian
        depth_out.step = depth_img.step
        depth_out.data = depth_img.data
        self.ir_pub.publish(ir_img)
        self.depth_pub.publish(depth_out)
        if ir.info is not None:
            info = deepcopy(ir.info)
            info.header.stamp = ir_img.header.stamp
            self.ir_info_pub.publish(info)
        if depth.info is not None:
            info = deepcopy(depth.info)
            info.header.stamp = ir_img.header.stamp
            self.depth_info_pub.publish(info)


def main() -> None:
    rclpy.init()
    node = EmitterPair()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
