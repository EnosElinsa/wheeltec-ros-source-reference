#!/usr/bin/env python3
"""Publish wheel odometry as TF without stealing odom→base_footprint.

The chassis topic is the pose of base_footprint in wheel_odom. This node
broadcasts the inverse: base_footprint → wheel_odom. Visual odometry keeps
odom → base_footprint as the only parent of the robot frame. RTAB-Map's
guess_frame_id=wheel_odom still looks up wheel_odom → base_footprint.
"""
from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_msgs.msg import TFMessage

from wheeltec_vision_nav.wheel_odom_tf_logic import invert_pose, planar_wheel_pose, pose_relative_to


class WheelOdomTf(Node):
    def __init__(self) -> None:
        super().__init__("vision_wheel_odom_tf")
        topic = str(self.declare_parameter("odom_topic", "/wheel/odom").value)
        self.parent_frame = str(self.declare_parameter("parent_frame", "base_footprint").value)
        self.child_frame = str(self.declare_parameter("child_frame", "wheel_odom").value)
        self._origin = None
        self.create_subscription(Odometry, topic, self._on_odom, 20)
        self.pub = self.create_publisher(TFMessage, "/tf", 50)

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        planar = planar_wheel_pose(p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if self._origin is None:
            self._origin = planar
        rel = pose_relative_to(self._origin, planar)
        ix, iy, iz, iqx, iqy, iqz, iqw = invert_pose(*rel)
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame
        t.transform.translation.x = ix
        t.transform.translation.y = iy
        t.transform.translation.z = iz
        t.transform.rotation.x = iqx
        t.transform.rotation.y = iqy
        t.transform.rotation.z = iqz
        t.transform.rotation.w = iqw
        self.pub.publish(TFMessage(transforms=[t]))


def main() -> None:
    rclpy.init()
    node = WheelOdomTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
