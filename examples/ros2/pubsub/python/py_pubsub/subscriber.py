# Copyright (c) 2026 WHEELTEC ROS Textbook Contributors
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Receive textbook messages from a fixed ROS 2 topic."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CourseSubscriber(Node):
    """Log each message received from the course topic."""

    def __init__(self) -> None:
        super().__init__("course_py_listener")
        self.subscription = self.create_subscription(
            String,
            "/course/chatter",
            self.receive_message,
            10,
        )

    def receive_message(self, message: String) -> None:
        """Log a received string message."""
        self.get_logger().info(f"Received: '{message.data}'")


def main(args=None) -> None:
    """Run the subscriber until interrupted."""
    rclpy.init(args=args)
    node = CourseSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
