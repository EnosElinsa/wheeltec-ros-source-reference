#!/usr/bin/env python3
"""Capture topic samples, PNG previews, and a YAML summary while mapping or navigating."""
from __future__ import annotations

import json
import math
import os
from collections import deque
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from wheeltec_vision_nav.capture_runtime_logic import (
    build_summary,
    encode_depth16_preview_png,
    encode_gray8_png,
    encode_occupancy_png,
    image_sample,
    stamp_to_sec,
    tf_parents,
    write_yaml,
)

try:
    from rtabmap_msgs.msg import OdomInfo, RGBDImage
except ImportError:  # pragma: no cover - overlay on the robot
    OdomInfo = None
    RGBDImage = None

try:
    from realsense2_camera_msgs.msg import Metadata
except ImportError:  # pragma: no cover - overlay on the robot
    Metadata = None


BEST_EFFORT = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
)
MAP_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)
TF_STATIC_QOS = QoSProfile(
    depth=100,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
)

PARAM_QUERIES = (
    ("vision_rgbd_odometry", ("guess_frame_id", "publish_tf", "imu_topic", "frame_id")),
    ("vision_rtabmap", ("database_path", "Mem/IncrementalMemory")),
    ("controller_server", ("FollowPath.vx_min",)),
    ("local_costmap/local_costmap", ("plugins", "static_layer.map_topic")),
    ("bt_navigator", ("default_nav_to_pose_bt_xml",)),
)


class CaptureRuntime(Node):
    def __init__(self) -> None:
        super().__init__("vision_capture_runtime")
        output_dir = str(self.declare_parameter("output_dir", str(Path.home() / "vision_captures")).value)
        self.max_images = int(self.declare_parameter("max_images", 8).value)
        self.image_period = float(self.declare_parameter("image_period", 5.0).value)
        stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
        self.session = Path(output_dir) / f"{stamp}-{os.getpid()}"
        (self.session / "images").mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f"capturing to {self.session}")

        self._samples: dict = {}
        self._params: dict = {}
        self._tf_pairs: list[tuple[str, str]] = []
        self._emitter_keys: list[str] = []
        self._images: list[str] = []
        self._saved: dict[str, int] = {}
        self._last_image_save: dict[str, float] = {}
        self._hz: dict[str, deque[float]] = {}
        self._param_clients = {}

        self.create_subscription(Image, "/camera/infra1/image_rect_raw", self._on_infra, BEST_EFFORT)
        self.create_subscription(Image, "/camera/depth/image_rect_raw", self._on_depth, BEST_EFFORT)
        self.create_subscription(Image, "/vision/paired/infra1/image_rect_raw", self._on_paired_infra, BEST_EFFORT)
        self.create_subscription(Image, "/vision/paired/depth/image_rect_raw", self._on_paired_depth, BEST_EFFORT)
        self.create_subscription(CameraInfo, "/camera/infra1/camera_info", self._on_camera_info, BEST_EFFORT)
        self.create_subscription(Odometry, "/odom", self._on_odom, BEST_EFFORT)
        self.create_subscription(Odometry, "/wheel/odom", self._on_wheel_odom, BEST_EFFORT)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, MAP_QOS)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(Twist, "/cmd_vel_safe", self._on_cmd_vel_safe, 10)
        self.create_subscription(String, "/vision/vo_status", self._on_vo_status, 10)
        self.create_subscription(TFMessage, "/tf", self._on_tf, BEST_EFFORT)
        self.create_subscription(TFMessage, "/tf_static", self._on_tf, TF_STATIC_QOS)
        if RGBDImage is not None:
            self.create_subscription(RGBDImage, "/rtabmap/rgbd_image", self._on_rgbd, BEST_EFFORT)
        if OdomInfo is not None:
            self.create_subscription(OdomInfo, "/odom_info_lite", self._on_odom_info, BEST_EFFORT)
        if Metadata is not None:
            self.create_subscription(Metadata, "/camera/infra1/metadata", self._on_metadata, BEST_EFFORT)
            self.create_subscription(Metadata, "/camera/depth/metadata", self._on_metadata, BEST_EFFORT)

        for node_name, _names in PARAM_QUERIES:
            self._param_clients[node_name] = self.create_client(GetParameters, f"/{node_name}/get_parameters")

        self.create_timer(2.0, self._tick)
        self._write_summary()

    def destroy_node(self) -> None:
        self._write_summary()
        super().destroy_node()

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _mark(self, topic: str) -> None:
        bucket = self._hz.setdefault(topic, deque(maxlen=80))
        bucket.append(self._now())

    def _hz_map(self) -> dict[str, float]:
        now = self._now()
        out = {}
        for topic, stamps in self._hz.items():
            recent = [t for t in stamps if now - t <= 2.0]
            if len(recent) >= 2:
                out[topic] = (len(recent) - 1) / max(recent[-1] - recent[0], 1e-6)
            else:
                out[topic] = 0.0
        return out

    def _image_fields(self, msg: Image) -> dict:
        return image_sample(
            msg.encoding,
            msg.width,
            msg.height,
            msg.header.frame_id,
            msg.header.stamp.sec,
            msg.header.stamp.nanosec,
            msg.step,
            len(msg.data),
        )

    def _maybe_save_image(self, key: str, msg: Image) -> None:
        count = self._saved.get(key, 0)
        if count >= self.max_images:
            return
        now = self._now()
        last = self._last_image_save.get(key)
        if last is not None and now - last < self.image_period:
            return
        try:
            payload = bytes(msg.data)
            if msg.encoding in ("mono8", "8UC1"):
                png = encode_gray8_png(msg.width, msg.height, payload[: msg.width * msg.height])
            elif msg.encoding in ("16UC1", "mono16"):
                png = encode_depth16_preview_png(
                    msg.width,
                    msg.height,
                    payload,
                    little_endian=not bool(msg.is_bigendian),
                )
            else:
                return
        except ValueError as exc:
            self.get_logger().warn(f"skip {key} png: {exc}")
            return
        name = f"{key}_{count:03d}.png"
        (self.session / "images" / name).write_bytes(png)
        self._images.append(name)
        self._saved[key] = count + 1
        self._last_image_save[key] = now

    def _on_infra(self, msg: Image) -> None:
        self._mark("/camera/infra1/image_rect_raw")
        self._samples["infra"] = self._image_fields(msg)
        self._maybe_save_image("infra1", msg)

    def _on_depth(self, msg: Image) -> None:
        self._mark("/camera/depth/image_rect_raw")
        self._samples["depth"] = self._image_fields(msg)
        self._maybe_save_image("depth", msg)

    def _on_paired_infra(self, msg: Image) -> None:
        self._mark("/vision/paired/infra1/image_rect_raw")
        self._samples["paired_infra"] = self._image_fields(msg)
        self._maybe_save_image("paired_infra1", msg)

    def _on_paired_depth(self, msg: Image) -> None:
        self._mark("/vision/paired/depth/image_rect_raw")
        self._samples["paired_depth"] = self._image_fields(msg)
        self._maybe_save_image("paired_depth", msg)

    def _on_rgbd(self, msg) -> None:
        self._mark("/rtabmap/rgbd_image")
        self._samples["rgbd"] = {
            "rgb": self._image_fields(msg.rgb),
            "depth": self._image_fields(msg.depth),
            "header_frame_id": msg.header.frame_id,
            "stamp_sec": stamp_to_sec(msg.header.stamp.sec, msg.header.stamp.nanosec),
        }
        if msg.rgb.width and msg.rgb.height:
            self._maybe_save_image("rgbd_rgb", msg.rgb)
        if msg.depth.width and msg.depth.height:
            self._maybe_save_image("rgbd_depth", msg.depth)

    def _on_camera_info(self, msg: CameraInfo) -> None:
        self._mark("/camera/infra1/camera_info")
        self._samples["camera_info_k"] = list(msg.k)
        self._samples["camera_info"] = {
            "frame_id": msg.header.frame_id,
            "width": msg.width,
            "height": msg.height,
            "distortion_model": msg.distortion_model,
        }

    def _on_odom(self, msg: Odometry) -> None:
        self._mark("/odom")
        q = msg.pose.pose.orientation
        self._samples["odom"] = {
            "frame_id": msg.header.frame_id,
            "child_frame_id": msg.child_frame_id,
            "quat_norm": math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w),
            "covariance0": msg.pose.covariance[0],
        }

    def _on_wheel_odom(self, msg: Odometry) -> None:
        self._mark("/wheel/odom")
        self._samples["wheel_odom"] = {
            "frame_id": msg.header.frame_id,
            "child_frame_id": msg.child_frame_id,
        }

    def _on_odom_info(self, msg) -> None:
        self._mark("/odom_info_lite")
        self._samples["odom_info"] = {
            "lost": bool(msg.lost),
            "inliers": int(msg.inliers),
            "features": int(msg.features),
        }

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._mark("/map")
        self._samples["map"] = {
            "frame_id": msg.header.frame_id,
            "resolution": msg.info.resolution,
            "width": msg.info.width,
            "height": msg.info.height,
            "origin": {
                "x": msg.info.origin.position.x,
                "y": msg.info.origin.position.y,
            },
        }
        key = "map"
        count = self._saved.get(key, 0)
        if count >= self.max_images:
            return
        now = self._now()
        last = self._last_image_save.get(key)
        if last is not None and now - last < self.image_period:
            return
        try:
            png = encode_occupancy_png(msg.info.width, msg.info.height, list(msg.data))
        except ValueError as exc:
            self.get_logger().warn(f"skip map png: {exc}")
            return
        name = f"map_{count:03d}.png"
        (self.session / "images" / name).write_bytes(png)
        self._images.append(name)
        self._saved[key] = count + 1
        self._last_image_save[key] = now

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._mark("/cmd_vel")
        self._samples["cmd_vel"] = {"linear_x": msg.linear.x, "angular_z": msg.angular.z}

    def _on_cmd_vel_safe(self, msg: Twist) -> None:
        self._mark("/cmd_vel_safe")
        self._samples["cmd_vel_safe"] = {"linear_x": msg.linear.x, "angular_z": msg.angular.z}

    def _on_vo_status(self, msg: String) -> None:
        self._mark("/vision/vo_status")
        self._samples["vo_status"] = msg.data

    def _on_tf(self, msg: TFMessage) -> None:
        self._mark("/tf")
        for item in msg.transforms:
            self._tf_pairs.append((item.header.frame_id, item.child_frame_id))
        if len(self._tf_pairs) > 400:
            self._tf_pairs = self._tf_pairs[-400:]

    def _on_metadata(self, msg) -> None:
        self._mark("/camera/metadata")
        text = getattr(msg, "json_data", "") or getattr(msg, "data", "")
        if not isinstance(text, str) or not text:
            return
        if "metadata_json_preview" not in self._samples:
            self._samples["metadata_json_preview"] = text[:800]
        try:
            payload = json.loads(text)
        except json.JSONDecodeError:
            return
        if isinstance(payload, dict):
            for key in payload:
                if key not in self._emitter_keys:
                    self._emitter_keys.append(str(key))
            emit = {key: payload[key] for key in payload if "emit" in str(key).lower()}
            if emit:
                self._samples["emitter_sample"] = emit

    def _tick(self) -> None:
        self._poll_params()
        self._write_summary()

    def _poll_params(self) -> None:
        for node_name, names in PARAM_QUERIES:
            client = self._param_clients[node_name]
            if not client.service_is_ready():
                continue
            request = GetParameters.Request()
            request.names = list(names)
            future = client.call_async(request)
            future.add_done_callback(lambda fut, prefix=node_name, keys=names: self._on_params(fut, prefix, keys))

    def _on_params(self, future, prefix: str, keys: tuple[str, ...]) -> None:
        try:
            result = future.result()
        except Exception:
            return
        if result is None:
            return
        for name, value in zip(keys, result.values):
            self._params[f"{prefix}.{name}"] = _param_value(value)

    def _write_summary(self) -> None:
        parts = dict(self._samples)
        parts["params"] = dict(self._params)
        parts["tf_parents"] = tf_parents(self._tf_pairs)
        parts["hz"] = self._hz_map()
        parts["emitter_keys"] = list(self._emitter_keys)
        parts["images"] = list(self._images)
        summary = build_summary(parts)
        summary["session"] = str(self.session)
        summary["samples"] = parts
        write_yaml(self.session / "summary.yaml", summary)


def _param_value(value):
    kind = value.type
    if kind == 1:
        return value.bool_value
    if kind == 2:
        return value.integer_value
    if kind == 3:
        return value.double_value
    if kind == 4:
        return value.string_value
    if kind == 5:
        return list(value.byte_array_value)
    if kind == 6:
        return list(value.bool_array_value)
    if kind == 7:
        return list(value.integer_array_value)
    if kind == 8:
        return list(value.double_array_value)
    if kind == 9:
        return list(value.string_array_value)
    return None


def main() -> None:
    rclpy.init()
    node = CaptureRuntime()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
