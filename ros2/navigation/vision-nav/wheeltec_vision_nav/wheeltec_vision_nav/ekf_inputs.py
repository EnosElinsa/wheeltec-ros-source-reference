#!/usr/bin/env python3
"""Republish the chassis gyro without its bias and VO velocity with realistic variance."""
from __future__ import annotations

from copy import deepcopy

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

from wheeltec_vision_nav.ekf_inputs_logic import VX, VY, VYAW, GyroBias, floor_twist_covariance


def _sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class EkfInputs(Node):
    def __init__(self) -> None:
        super().__init__("vision_ekf_inputs")
        self.gyro_variance = float(self.declare_parameter("gyro_yaw_rate_variance", 1e-5).value)
        vo_vx = float(self.declare_parameter("vo_vx_variance_min", 2.5e-3).value)
        vo_vyaw = float(self.declare_parameter("vo_vyaw_variance_min", 1e-3).value)
        self.vo_floors = {VX: vo_vx, VY: vo_vx, VYAW: vo_vyaw}
        self.bias = GyroBias()
        sensor_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT,
                                history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Odometry, "/wheel/odom", self._on_wheel, sensor_qos)
        self.create_subscription(Imu, "/imu/data_raw", self._on_imu, sensor_qos)
        self.create_subscription(Odometry, "/odom_vo", self._on_vo, sensor_qos)
        self.imu_pub = self.create_publisher(Imu, "/vision/imu", 20)
        self.vo_pub = self.create_publisher(Odometry, "/vision/odom_vo", 20)
        self.create_timer(10.0, self._log_bias)

    def _on_wheel(self, msg: Odometry) -> None:
        self.bias.note_wheel(_sec(msg.header.stamp), msg.twist.twist.linear.x, msg.twist.twist.angular.z)

    def _on_imu(self, msg: Imu) -> None:
        out = deepcopy(msg)
        out.angular_velocity.z = self.bias.correct(_sec(msg.header.stamp), msg.angular_velocity.z)
        covariance = list(out.angular_velocity_covariance)
        covariance[8] = self.gyro_variance
        out.angular_velocity_covariance = covariance
        self.imu_pub.publish(out)

    def _on_vo(self, msg: Odometry) -> None:
        out = deepcopy(msg)
        out.twist.covariance = floor_twist_covariance(msg.twist.covariance, self.vo_floors)
        self.vo_pub.publish(out)

    def _log_bias(self) -> None:
        self.get_logger().info(
            f"gyro yaw-rate bias {self.bias.bias:.6f} rad/s from {self.bias.samples} standstill samples"
        )


def main() -> None:
    rclpy.init()
    node = EkfInputs()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
