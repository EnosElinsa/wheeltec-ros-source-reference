"""Bounded live-run CSV evidence and configuration/software manifests."""
from __future__ import annotations

import csv
import json
import math
import os
import platform
import time
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from pathlib import Path
from xml.etree import ElementTree

import psutil
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import GetParameters, ListParameters
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rtabmap_msgs.msg import OdomInfo
from sensor_msgs.msg import Image, Imu, PointCloud2
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage

from wheeltec_vision_nav.runtime_report import build_report, digest_file


def ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class RuntimeMonitor(Node):
    def __init__(self):
        super().__init__("vision_runtime_monitor")
        root = Path(self.declare_parameter("output_dir", "~/vision_runs").value).expanduser()
        self.directory = root / (datetime.now().strftime("%Y%m%d-%H%M%S-") + str(os.getpid()))
        self.directory.mkdir(parents=True, exist_ok=False)
        self._files, self._writers = {}, {}
        columns = {"odometry": ["stamp_ns", "x", "y", "yaw", "latency_s"],
                   "visual": ["stamp_ns", "lost", "inliers", "processing_s"],
                   "frames": ["stamp_ns"],
                   "resources": ["received_s", "rss_mb", "cpu_percent", "temperature_c", "gpu_load_percent"],
                   "processes": ["received_s", "pid", "name", "rss_mb", "cpu_percent"],
                   "map_corrections": ["stamp_ns", "x", "y", "yaw"],
                   "topics": ["topic", "stamp_ns", "received_s", "age_s"],
                   "health": ["received_s", "state", "reason", "output_vx", "output_wz"],
                   "commands": ["received_s", "topic", "vx", "wz"]}
        for name, header in columns.items():
            stream = (self.directory / f"{name}.csv").open("x", encoding="utf-8", newline="")
            self._files[name] = stream
            self._writers[name] = csv.writer(stream)
            self._writers[name].writerow(header)
        self._provenance = (self.directory / "provenance.jsonl").open("x", encoding="utf-8")
        self._details = (self.directory / "health.jsonl").open("x", encoding="utf-8")
        root_pid = int(self.declare_parameter("resource_root_pid", os.getppid()).value)
        ancestors = {p.pid for p in psutil.Process().parents()}
        if root_pid not in ancestors:
            raise ValueError("resource_root_pid must be an ancestor launch process")
        self._root = psutil.Process(root_pid)
        self._processes = {}
        self._param_clients, self._requested = {}, set()
        self._param_snapshot = {}
        self._artifact_args = {name: self.declare_parameter(name, "").value
                               for name in ("runtime_config", "nav_params_file", "database_path")}
        self._worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="vision-manifest")
        self._manifest = self._worker.submit(self._write_manifest)
        qos = qos_profile_sensor_data
        self.create_subscription(Odometry, "/odom", self._odom, qos)
        self.create_subscription(OdomInfo, "/odom_info_lite", self._visual, qos)
        self.create_subscription(Image, "/vision/paired/infra1/image_rect_raw", self._frame, qos)
        for topic, message in (("/odom_vo", Odometry), ("/wheel/odom", Odometry),
                               ("/vision/imu", Imu), ("/camera/depth/image_rect_raw", Image),
                               ("/camera/infra1/image_rect_raw", Image),
                               ("/vision/depth_cloud", PointCloud2), ("/vision/obstacles", PointCloud2)):
            self.create_subscription(message, topic, lambda msg, t=topic: self._topic(t, msg), qos)
        self.create_subscription(String, "/vision/health/status", self._health, 5)
        self.create_subscription(String, "/vision/emitter_pair/provenance", self._pair, 20)
        self.create_subscription(TFMessage, "/tf", self._map_tf, qos)
        for topic in ("/cmd_vel", "/cmd_vel_nav", "/cmd_vel_smoothed", "/cmd_vel_safe"):
            self.create_subscription(Twist, topic, lambda msg, t=topic: self._command(t, msg), 5)
        self._steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(1.0, self._tick, clock=self._steady_clock)
        self.get_logger().info(f"runtime evidence: {self.directory}")

    def _write_manifest(self):
        config = self._artifact_args["runtime_config"]
        nav = self._artifact_args["nav_params_file"]
        database = self._artifact_args["database_path"]
        artifact_paths = [Path(p).expanduser().resolve() for p in (config, nav) if p]
        artifacts = {}
        for index, path in enumerate(artifact_paths):
            data = path.read_bytes()
            saved = self.directory / f"config-{index}-{path.name}"
            saved.write_bytes(data)
            artifacts[str(path)] = {"sha256": digest_file(saved), "copy": saved.name}
        versions = {}
        for name in ("wheeltec_vision_nav", "realsense2_camera", "rtabmap_odom", "rtabmap_slam",
                     "robot_localization", "nav2_bringup", "nav2_mppi_controller", "turn_on_wheeltec_robot"):
            try:
                share = Path(get_package_share_directory(name))
                versions[name] = {"version": ElementTree.parse(share / "package.xml").findtext("version"),
                                  "share": str(share)}
            except (OSError, LookupError):
                versions[name] = {"version": "unavailable"}
        map_info = {"path": str(Path(database).expanduser().resolve())} if database else {}
        if database and Path(database).expanduser().is_file():
            path = Path(database).expanduser()
            before = path.stat()
            map_info.update(sha256=digest_file(path), size=before.st_size,
                            mtime_ns=before.st_mtime_ns,
                            changing=before.st_mtime_ns != path.stat().st_mtime_ns,
                            wal_present=Path(str(path) + "-wal").exists(),
                            note="file identity only; live SQLite/WAL is not a frozen map snapshot")
        package = Path(__file__).parent
        result = {"schema_version": 1, "created_at": datetime.now().astimezone().isoformat(),
                  "platform": platform.platform(), "python": platform.python_version(),
                  "ros_distro": os.environ.get("ROS_DISTRO"), "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
                  "software": versions, "config_artifacts": artifacts, "map": map_info,
                  "source_sha256": {p.name: digest_file(p) for p in sorted(package.glob("*.py"))},
                  "calibration_and_braking": "require_target_validation",
                  "realsense_firmware": "record_from_target_device_acceptance"}
        result["metric_min_inliers"] = (yaml.safe_load(Path(config).expanduser().read_text(encoding="utf-8"))
                                         ["safety"]["min_inliers"] if config else 10)
        (self.directory / "manifest.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")

    def _write(self, name, values):
        self._writers[name].writerow(values)

    def _topic(self, topic, msg):
        stamp = ns(msg.header.stamp)
        self._write("topics", [topic, stamp, time.monotonic(),
                               (self.get_clock().now().nanoseconds - stamp) / 1e9])

    def _odom(self, msg):
        self._topic("/odom", msg)
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
        self._write("odometry", [ns(msg.header.stamp), p.x, p.y, yaw,
                                 (self.get_clock().now().nanoseconds - ns(msg.header.stamp)) / 1e9])

    def _visual(self, msg):
        self._write("visual", [ns(msg.header.stamp), int(msg.lost), int(msg.inliers),
                               float(getattr(msg, "time_estimation", 0.0))])

    def _map_tf(self, msg):
        for transform in msg.transforms:
            if transform.header.frame_id == "map" and transform.child_frame_id == "odom":
                p, q = transform.transform.translation, transform.transform.rotation
                yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1-2*(q.y*q.y + q.z*q.z))
                self._write("map_corrections", [ns(transform.header.stamp), p.x, p.y, yaw])

    def _frame(self, msg):
        self._write("frames", [ns(msg.header.stamp)])

    def _health(self, msg):
        try:
            row = json.loads(msg.data)
            self._write("health", [time.monotonic(), row["state"], row["reason"],
                                   row.get("output_vx", 0), row.get("output_wz", 0)])
            self._details.write(json.dumps({"received_s": time.monotonic(), **row}, allow_nan=False) + "\n")
        except (ValueError, KeyError):
            self.get_logger().warning("invalid health evidence rejected")

    def _pair(self, msg):
        try:
            row = json.loads(msg.data)
            self._provenance.write(json.dumps(row, allow_nan=False) + "\n")
        except ValueError:
            self.get_logger().warning("invalid pairing evidence rejected")

    def _command(self, topic, msg):
        now = time.monotonic()
        self._write("commands", [now, topic, msg.linear.x, msg.angular.z])
        if topic in ("/cmd_vel", "/cmd_vel_nav"):
            self._write("topics", [topic, 0, now, 0])

    def _poll_parameters(self):
        names = ("vision_vo_watchdog", "vision_rgbd_odometry", "vision_rtabmap", "vision_ekf",
                 "vision_ekf_inputs", "vision_emitter_pair", "controller_server", "planner_server",
                 "local_costmap/local_costmap", "global_costmap/global_costmap", "velocity_smoother")
        for name in names:
            if name in self._requested:
                continue
            clients = self._param_clients.get(name)
            if clients is None:
                clients = (self.create_client(ListParameters, f"/{name}/list_parameters"),
                           self.create_client(GetParameters, f"/{name}/get_parameters"))
                self._param_clients[name] = clients
            listing, getter = clients
            if listing.service_is_ready() and getter.service_is_ready():
                self._requested.add(name)
                future = listing.call_async(ListParameters.Request(depth=0))
                future.add_done_callback(lambda f, n=name, g=getter: self._parameter_names(f, n, g))

    def _parameter_names(self, future, name, getter):
        try:
            keys = future.result().result.names
            result = getter.call_async(GetParameters.Request(names=keys))
            result.add_done_callback(lambda f: self._on_parameter_values(f, name, keys))
        except Exception:
            self._requested.discard(name)

    def _on_parameter_values(self, future, name, keys):
        try:
            from rosidl_runtime_py.convert import message_to_ordereddict
            self._param_snapshot[name] = {k: dict(message_to_ordereddict(v))
                                         for k, v in zip(keys, future.result().values)}
            (self.directory / "effective_parameters.json").write_text(
                json.dumps(self._param_snapshot, indent=2) + "\n", encoding="utf-8")
        except Exception:
            self._requested.discard(name)

    def _tick(self):
        rss, cpu = 0.0, 0.0
        try:
            current = {p.pid: p for p in [self._root, *self._root.children(recursive=True)]}
        except psutil.Error:
            current = {}
        self._processes = {pid: self._processes.get(pid, p) for pid, p in current.items()}
        for process in self._processes.values():
            try:
                process_rss = process.memory_info().rss / (1024*1024)
                process_cpu = process.cpu_percent()
                rss += process_rss
                cpu += process_cpu
                self._write("processes", [time.monotonic(), process.pid, process.name(), process_rss, process_cpu])
            except psutil.Error:
                pass
        temperatures = []
        for path in Path("/sys/class/thermal").glob("thermal_zone*/temp"):
            try:
                temperatures.append(float(path.read_text()) / 1000)
            except (OSError, ValueError):
                pass
        gpu = None
        for path in (Path("/sys/devices/gpu.0/load"), Path("/sys/devices/platform/17000000.gpu/load")):
            try:
                gpu = float(path.read_text()) / 10
                break
            except (OSError, ValueError):
                pass
        self._write("resources", [time.monotonic(), rss, cpu, max(temperatures) if temperatures else None, gpu])
        self._poll_parameters()
        for stream in [*self._files.values(), self._provenance, self._details]:
            stream.flush()
        if self._manifest.done() and self._manifest.exception() is not None:
            self.get_logger().error(f"manifest collection failed: {self._manifest.exception()}")

    def destroy_node(self):
        self._worker.shutdown(wait=True)
        for stream in [*self._files.values(), self._provenance, self._details]:
            stream.close()
        try:
            report = build_report(self.directory)
            (self.directory / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n",
                                                        encoding="utf-8")
        except (ValueError, OSError, KeyError) as error:
            self.get_logger().error(f"runtime report failed; raw evidence retained: {error}")
        return super().destroy_node()


def main():
    rclpy.init()
    node = RuntimeMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
