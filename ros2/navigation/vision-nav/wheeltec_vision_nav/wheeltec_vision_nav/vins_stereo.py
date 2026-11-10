"""ROS adapter for exact, emitter-off stereo pairs; original messages are forwarded."""
import json
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String

from wheeltec_vision_nav.emitter_pair_logic import (
    EmitterClassifier, ir_dot_density, depth_fill_ratio, parse_emitter_mode,
)
from wheeltec_vision_nav.vins_stereo_logic import StereoBuffer


def ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class VinsStereo(Node):
    def __init__(self):
        super().__init__("vins_stereo")
        self.buffer = StereoBuffer()
        self.classifier = EmitterClassifier(keep=120)
        self.policy = self.declare_parameter("emitter_policy", "interleaved").value
        if self.policy not in {"interleaved", "off"}:
            raise ValueError("emitter_policy must be interleaved or off")
        self.pending_stamps = set()
        self.paired = 0
        self.last_source = "unknown"
        self.last_stamp = None
        qos = qos_profile_sensor_data
        self.pubs = {}
        for side, infra in (("left", "infra1"), ("right", "infra2")):
            self.pubs[side] = self.create_publisher(Image, f"/vins/stereo/{side}/image_rect", qos)
            self.pubs[side + "_info"] = self.create_publisher(CameraInfo, f"/vins/stereo/{side}/camera_info", qos)
            self.create_subscription(Image, f"/camera/{infra}/image_rect_raw",
                                     lambda msg, s=side: self._image(s, msg), qos)
            self.create_subscription(CameraInfo, f"/camera/{infra}/camera_info",
                                     lambda msg, s=side: self._info(s, msg), qos)
        self.create_subscription(Image, "/camera/depth/image_rect_raw", self._depth, qos)
        from realsense2_camera_msgs.msg import Metadata
        for infra in ("infra1", "infra2", "depth"):
            self.create_subscription(Metadata, f"/camera/{infra}/metadata", self._metadata, qos)
        self.status = self.create_publisher(String, "/vins/stereo/status", 10)
        self.create_timer(1., self._status)

    def _image(self, side, msg):
        stamp = ns(msg.header.stamp)
        if msg.encoding not in {"mono8", "8UC1"} or msg.step < msg.width or len(msg.data) != msg.step * msg.height:
            return
        self.buffer.add(side, stamp, msg)
        self.pending_stamps.add(stamp)
        if self.policy == "off":
            self.buffer.label(stamp, 0, "configured_off")
        elif side == "left":
            array = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width]
            self.classifier.update(stamp / 1e9, dots=ir_dot_density(array))
        self._flush()

    def _info(self, side, msg):
        self.buffer.add(side + "_info", ns(msg.header.stamp),
                        {"width": msg.width, "height": msg.height, "p": list(msg.p), "message": msg})
        self._flush()

    def _depth(self, msg):
        if (self.policy == "interleaved" and msg.encoding in {"16UC1", "mono16"}
                and msg.step >= 2 * msg.width and msg.step % 2 == 0 and len(msg.data) == msg.height * msg.step):
            dtype = ">u2" if msg.is_bigendian else "<u2"
            array = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.step // 2)[:, :msg.width]
            self.classifier.update(ns(msg.header.stamp) / 1e9, fill=depth_fill_ratio(array))
            self._flush()

    def _metadata(self, msg):
        mode = parse_emitter_mode(msg.json_data)
        if mode in (0, 1):
            self.buffer.label(ns(msg.header.stamp), mode, "metadata")
            self._flush()

    def _flush(self):
        for stamp in self.pending_stamps:
            mode = self.classifier.label(stamp / 1e9)
            if mode is not None:
                self.buffer.label(stamp, mode, "image")
        for pair in self.buffer.ready():
            for side in ("left", "right", "left_info", "right_info"):
                self.pubs[side].publish(getattr(pair, side))
            self.paired += 1
            self.last_source, self.last_stamp = pair.source, pair.stamp_ns
        # Bounds memory even when inputs are incomplete or timestamps jump.
        self.pending_stamps = set(sorted(s for s in self.pending_stamps if s > self.buffer.last_output)[-60:])

    def _status(self):
        self.status.publish(String(data=json.dumps({"schema_version": 1, "paired_total": self.paired,
            "last_pair_stamp_ns": self.last_stamp, "emitter_source": self.last_source,
            "timestamp_policy": "exact_original", "emitter_policy": self.policy})))


def main():
    rclpy.init()
    node = VinsStereo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
