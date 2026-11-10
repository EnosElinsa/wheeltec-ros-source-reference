"""Keep unusable interleaved raw depth out of cloud generation and clearing."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from wheeltec_vision_nav.depth_validity import ExactDepthGate


class DepthValidityGate(Node):
    def __init__(self):
        super().__init__("vision_depth_validity")
        self.gate = ExactDepthGate(
            timeout_s=self.declare_parameter("sensor_timeout_s", .3).value,
            future_tolerance_s=self.declare_parameter("future_tolerance_s", .1).value)
        qos = QoSProfile(depth=8, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.image_pub = self.create_publisher(Image, "depth_valid/image", qos)
        self.info_pub = self.create_publisher(CameraInfo, "depth_valid/camera_info", qos)
        self.create_subscription(Image, "depth/image", self._on_image, qos)
        self.create_subscription(CameraInfo, "depth/camera_info", self._on_info, qos)

    def _publish(self, pair):
        if pair is not None:
            image, info = pair
            self.info_pub.publish(info)
            self.image_pub.publish(image)

    def _on_image(self, msg):
        self._publish(self.gate.image(msg, self.get_clock().now().nanoseconds))

    def _on_info(self, msg):
        self._publish(self.gate.camera_info(msg, self.get_clock().now().nanoseconds))


def main():
    rclpy.init()
    node = DepthValidityGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
