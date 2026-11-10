"""Write incremental evaluation CSVs and visual health; never publishes velocity or TF."""
import csv
import json
import math
import time
from pathlib import Path
import psutil
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rtabmap_msgs.msg import OdomInfo
from wheeltec_vision_nav.vins_metrics import VisualHealth
from wheeltec_vision_nav.vins_stereo import ns


class VinsMonitor(Node):
    def __init__(self):
        super().__init__("vins_monitor")
        output = Path(self.declare_parameter("output_dir", "").value)
        if not output.is_dir():
            raise ValueError("output_dir must be an existing experiment directory")
        self.files, self.writers = {}, {}
        columns = {"odometry": ["stamp_ns", "x", "y", "yaw", "latency_s"],
                   "visual": ["stamp_ns", "lost", "inliers", "processing_s"],
                   "frames": ["stamp_ns"], "resources": ["wall_s", "rss_mb", "cpu_percent"]}
        for key, fields in columns.items():
            f = (output / f"{key}.csv").open("x", encoding="utf-8", newline="")
            self.files[key] = f
            self.writers[key] = csv.writer(f)
            self.writers[key].writerow(fields)
        odom = self.declare_parameter("odom_topic", "/odom_vins").value
        info = self.declare_parameter("info_topic", "/vins/odom_info").value
        image = "/vision/paired/infra1/image_rect_raw" if odom == "/odom" else "/vins/stereo/left/image_rect"
        self.health = VisualHealth(min_inliers=int(self.declare_parameter("min_inliers", 10).value))
        self.processes = {}
        # The launch passes its own PID. Scanning every host process would include
        # the real robot or other domains in the A/B/C resource comparison.
        self.resource_root = psutil.Process(int(self.declare_parameter("resource_root_pid", 0).value))
        if self.resource_root.pid not in {p.pid for p in psutil.Process().parents()}:
            raise ValueError("resource_root_pid must be an ancestor of this monitor")
        self.create_subscription(Odometry, odom, self._odom, qos_profile_sensor_data)
        self.create_subscription(OdomInfo, info, self._info, qos_profile_sensor_data)
        self.create_subscription(Image, image, lambda m: self._write("frames", [ns(m.header.stamp)]), qos_profile_sensor_data)
        self.pub = self.create_publisher(String, "/vins/health", 10)
        self.create_timer(1., self._tick)

    def _write(self, key, row):
        self.writers[key].writerow(row)
        self.files[key].flush()

    def _odom(self, msg):
        q, p = msg.pose.pose.orientation, msg.pose.pose.position
        qnorm = math.hypot(q.x, q.y, q.z, q.w)
        yaw = math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z)) if abs(qnorm-1.) < 1e-3 else float("nan")
        self._write("odometry", [ns(msg.header.stamp), p.x, p.y, yaw,
                                  (self.get_clock().now().nanoseconds - ns(msg.header.stamp)) / 1e9])

    def _info(self, msg):
        stamp = ns(msg.header.stamp)
        self._write("visual", [stamp, bool(msg.lost), msg.inliers, msg.time_estimation])
        self.health.note_visual(stamp, msg.lost, msg.inliers, time.monotonic())

    def _tick(self):
        wanted = {"stereo_odometry", "rgbd_odometry", "ekf_node", "vins_stereo", "emitter_pair", "rgbd_sync"}
        rss, cpu = 0., 0.
        try:
            children = self.resource_root.children(recursive=True)
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            children = []
        for item in children:
            try:
                if not any(Path(part).name in wanted for part in item.cmdline()):
                    continue
                proc = self.processes.setdefault(item.pid, psutil.Process(item.pid))
                rss += proc.memory_info().rss / 1024**2
                cpu += proc.cpu_percent()
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                self.processes.pop(item.pid, None)
        self._write("resources", [time.time(), rss, cpu])
        self.pub.publish(String(data=json.dumps({"schema_version": 1,
            "visual_state": self.health.status(time.monotonic()), "drives_robot": False})))

    def destroy_node(self):
        for f in self.files.values():
            f.close()
        return super().destroy_node()


def main():
    rclpy.init()
    node = VinsMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
