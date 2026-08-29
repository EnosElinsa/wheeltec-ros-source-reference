# Copyright (c) 2026 WHEELTEC ROS Textbook Contributors
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

"""Publish numbered textbook messages on a fixed ROS 2 topic."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CoursePublisher(Node):
    """Publish one message every half-second."""

    def __init__(self) -> None:
        super().__init__("course_py_talker")
        self.publisher = self.create_publisher(String, "/course/chatter", 10)
        self.sequence = 0
        self.timer = self.create_timer(0.5, self.publish_message)

    def publish_message(self) -> None:
        """Build and publish the next numbered message."""
        message = String()
        message.data = f"Python says hello #{self.sequence}"
        self.publisher.publish(message)
        self.get_logger().info(f"Publishing: '{message.data}'")
        self.sequence += 1


def main(args=None) -> None:
    """Run the publisher until interrupted."""
    rclpy.init(args=args)
    node = CoursePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
