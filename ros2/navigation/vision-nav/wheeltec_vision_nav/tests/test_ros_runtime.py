"""Real ROS middleware tests with synthetic sensors and no chassis process.

Opt in with WHEELTEC_RUN_ROS_TESTS=1 and an isolated nonzero ROS_DOMAIN_ID.
"""
import json
import os
import time

import pytest

pytestmark = pytest.mark.ros
if os.environ.get("WHEELTEC_RUN_ROS_TESTS") != "1" or os.environ.get("ROS_DOMAIN_ID", "0") == "0":
    pytest.skip("ROS integration requires explicit opt-in and an isolated nonzero domain", allow_module_level=True)

rclpy = pytest.importorskip("rclpy")
from action_msgs.srv import CancelGoal
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rtabmap_msgs.msg import OdomInfo
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Bool, Header, String
from tf2_ros import TransformBroadcaster

from wheeltec_vision_nav.vo_watchdog import VoWatchdog
from wheeltec_vision_nav.depth_validity_gate import DepthValidityGate


def spin_until(executor, condition, timeout=7):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        executor.spin_once(timeout_sec=.01)
        if condition():
            return
    raise AssertionError("ROS condition timed out")


def test_real_qos_tf_motion_gate_sensor_silence_cancellation_and_shutdown():
    rclpy.init(args=["--ros-args", "-p", "mode:=navigation", "-p", "wheel_backup:=false",
                    "-p", "map_stale_s:=0.4"])
    watchdog = VoWatchdog()
    sensors = Node("vision_test_sensors")
    executor = SingleThreadedExecutor()
    executor.add_node(watchdog)
    executor.add_node(sensors)
    qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
    pub = {name: sensors.create_publisher(kind, topic, qos) for name, kind, topic in (
        ("vo", Odometry, "/odom_vo"), ("odom", Odometry, "/odom"),
        ("info", OdomInfo, "/odom_info_lite"), ("depth", Image, "/camera/depth/image_rect_raw"),
        ("cloud", PointCloud2, "/vision/depth_cloud"), ("obstacles", PointCloud2, "/vision/obstacles"),
        ("localization", PoseWithCovarianceStamped, "/localization_pose"))}
    command_pub = sensors.create_publisher(Twist, "/cmd_vel", 1)
    goal_pub = sensors.create_publisher(GoalStatusArray, "/navigate_to_pose/_action/status",
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    broadcaster = TransformBroadcaster(sensors)
    outputs, health, ready, cancel_calls = [], [], [], []
    sensors.create_subscription(Twist, "/cmd_vel_safe", lambda m: outputs.append((time.monotonic(), m.linear.x)), 5)
    sensors.create_subscription(String, "/vision/health/status", lambda m: health.append(json.loads(m.data)), 5)
    sensors.create_subscription(Bool, "/vision/ready", lambda m: ready.append(m.data),
                                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    def cancel(request, response):
        cancel_calls.append(bytes(request.goal_info.goal_id.uuid))
        response.return_code = CancelGoal.Response.ERROR_NONE
        return response

    for action in ("navigate_to_pose", "navigate_through_poses", "follow_waypoints"):
        sensors.create_service(CancelGoal, f"/{action}/_action/cancel_goal", cancel)
    send_cloud = [True]
    send_map_tf = [True]

    def publish():
        now = sensors.get_clock().now()
        stamp = now.to_msg()
        transform = TransformStamped()
        transform.header = Header(stamp=stamp, frame_id="odom")
        transform.child_frame_id = "base_footprint"
        transform.transform.rotation.w = 1.
        broadcaster.sendTransform(transform)
        map_transform = TransformStamped()
        map_transform.header = Header(stamp=stamp, frame_id="map")
        map_transform.child_frame_id = "odom"
        map_transform.transform.rotation.w = 1.
        if send_map_tf[0]:
            broadcaster.sendTransform(map_transform)
        for name in ("odom", "vo"):
            msg = Odometry()
            msg.header = Header(stamp=stamp, frame_id="odom")
            msg.child_frame_id = "base_footprint"
            msg.pose.pose.orientation.w = 1.
            for index in (0, 7, 14, 21, 28, 35):
                msg.pose.covariance[index] = msg.twist.covariance[index] = .01
            pub[name].publish(msg)
        info = OdomInfo()
        info.header = Header(stamp=stamp, frame_id="base_footprint")
        info.inliers, info.lost = 40, False
        pub["info"].publish(info)
        depth = Image()
        depth.header = Header(stamp=stamp, frame_id="camera_depth_optical_frame")
        depth.width, depth.height, depth.step, depth.encoding = 4, 4, 8, "16UC1"
        depth.data = bytes([232, 3] * 16)
        pub["depth"].publish(depth)
        header = Header(stamp=(now - Duration(seconds=.06)).to_msg(), frame_id="base_footprint")
        if send_cloud[0]:
            points = [(x, y, 0.) for x in (1., 1.5, 2.) for y in (-.2, 0., .2)]
            pub["cloud"].publish(create_cloud_xyz32(header, points))
        pub["obstacles"].publish(create_cloud_xyz32(header, []))
        localization = PoseWithCovarianceStamped()
        localization.header = Header(stamp=stamp, frame_id="map")
        localization.pose.pose.orientation.w = 1.
        for index in (0, 7, 35):
            localization.pose.covariance[index] = .01
        pub["localization"].publish(localization)
        command = Twist()
        command.linear.x = .2
        command_pub.publish(command)

    sensors.create_timer(.05, publish)
    try:
        spin_until(executor, lambda: any(ready) and any(v > 0 for _, v in outputs))
        assert len(sensors.get_publishers_info_by_topic("/odom")) == 1
        assert len(sensors.get_publishers_info_by_topic("/cmd_vel_safe")) == 1
        failed_at = time.monotonic()
        old_acceptance = sensors.get_clock().now().to_msg()
        send_cloud[0] = False
        spin_until(executor, lambda: health and health[-1]["state"] == "STOPPED")
        spin_until(executor, lambda: len(cancel_calls) >= 3)
        end = time.monotonic() + .4
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.01)
        late = [v for t, v in outputs if t > failed_at + .4]
        assert late and all(v == 0 for v in late)
        assert health[-1]["new_goal_required"]
        send_cloud[0] = True
        spin_until(executor, lambda: health[-1]["state"] == "READY" and health[-1]["new_goal_required"])
        old_goal = GoalStatus()
        old_goal.goal_info.goal_id.uuid = [1] * 16
        old_goal.goal_info.stamp = old_acceptance
        old_goal.status = GoalStatus.STATUS_EXECUTING
        goal_pub.publish(GoalStatusArray(status_list=[old_goal]))
        retained_at = time.monotonic()
        spin_until(executor, lambda: time.monotonic() - retained_at > .3)
        assert health[-1]["new_goal_required"]
        assert all(v == 0 for t, v in outputs if t > retained_at)
        new_goal = GoalStatus()
        new_goal.goal_info.goal_id.uuid = [2] * 16
        new_goal.goal_info.stamp = sensors.get_clock().now().to_msg()
        new_goal.status = GoalStatus.STATUS_ACCEPTED
        accepted_at = time.monotonic()
        goal_pub.publish(GoalStatusArray(status_list=[new_goal]))
        spin_until(executor, lambda: any(t > accepted_at and v > 0 for t, v in outputs))
        # Freeze only map->odom; localization, odometry, VO and geometry advance.
        send_map_tf[0] = False
        spin_until(executor, lambda: health[-1]["state"] == "STOPPED"
                   and health[-1]["reason"] == "map_tf_invalid_or_stale")
        assert outputs[-1][1] == 0
    finally:
        executor.remove_node(watchdog)
        watchdog.destroy_node()
        executor.remove_node(sensors)
        sensors.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def test_real_depth_gate_preserves_exact_pairs_and_suppresses_emitter_off_frames():
    rclpy.init()
    gate = DepthValidityGate()
    sensors = Node("vision_test_depth_gate")
    executor = SingleThreadedExecutor()
    executor.add_node(gate)
    executor.add_node(sensors)
    qos = QoSProfile(depth=8, reliability=ReliabilityPolicy.BEST_EFFORT)
    depth_pub = sensors.create_publisher(Image, "/depth/image", qos)
    info_pub = sensors.create_publisher(CameraInfo, "/depth/camera_info", qos)
    images, infos, originals = {}, {}, {}
    key = lambda msg: msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
    sensors.create_subscription(Image, "/depth_valid/image", lambda msg: images.update({key(msg): msg}), qos)
    sensors.create_subscription(CameraInfo, "/depth_valid/camera_info", lambda msg: infos.update({key(msg): msg}), qos)
    counter, lost = [0], [False]

    def publish():
        counter[0] += 1
        valid = not lost[0] and counter[0] % 2 == 0
        msg = Image()
        msg.header = Header(stamp=sensors.get_clock().now().to_msg(), frame_id="depth_optical")
        msg.width, msg.height, msg.step, msg.encoding = 4, 4, 8, "16UC1"
        msg.data = bytes([232, 3] * 16) if valid else bytes(32)
        calibration = CameraInfo()
        calibration.header, calibration.width, calibration.height = msg.header, 4, 4
        calibration.k = [400., 0., 2., 0., 400., 2., 0., 0., 1.]
        originals[key(msg)] = (msg, calibration, valid)
        # Exercise both DDS arrival orders, with exactly matching acquisition stamps.
        if counter[0] % 4 == 0:
            depth_pub.publish(msg)
            info_pub.publish(calibration)
        else:
            info_pub.publish(calibration)
            depth_pub.publish(msg)

    sensors.create_timer(.033, publish)
    try:
        spin_until(executor, lambda: len(set(images) & set(infos)) >= 8)
        lost[0] = True
        loss_stamp = sensors.get_clock().now().nanoseconds
        deadline = time.monotonic() + .4
        spin_until(executor, lambda: time.monotonic() >= deadline)
        assert images and all(acquisition < loss_stamp for acquisition in images)
        for acquisition in set(images) & set(infos):
            original, calibration, valid = originals[acquisition]
            assert valid
            assert images[acquisition] == original
            assert infos[acquisition] == calibration
    finally:
        executor.remove_node(gate)
        gate.destroy_node()
        executor.remove_node(sensors)
        sensors.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
