"""Exit successfully after watchdog readiness, or fail after a steady deadline."""
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_msgs.msg import Bool

from wheeltec_vision_nav.startup import ReadinessDeadline


class WaitReady(Node):
    def __init__(self):
        super().__init__("vision_wait_ready")
        timeout = float(self.declare_parameter("timeout_s", 30.0).value)
        self.deadline = ReadinessDeadline(timeout)
        self.result_code = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(Bool, "/vision/ready", self._ready, qos)
        self.create_timer(0.05, self._check, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def _ready(self, message):
        self.deadline.observe(message.data)

    def _check(self):
        self.result_code = self.deadline.result()
        if self.result_code == 2:
            self.get_logger().error("Readiness timed out waiting for /vision/ready; inspect /vision/health/status")


def main(args=None):
    rclpy.init(args=args)
    node = WaitReady()
    try:
        while rclpy.ok() and node.result_code is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        code = node.result_code if node.result_code is not None else 2
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return code
