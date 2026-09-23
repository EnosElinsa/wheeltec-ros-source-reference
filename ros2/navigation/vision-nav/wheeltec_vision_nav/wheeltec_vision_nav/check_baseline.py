#!/usr/bin/env python3
"""Collect or score an R680 baseline bag without changing robot config."""

from __future__ import annotations

import argparse
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path

import yaml

from wheeltec_vision_nav.baseline_criteria import (
    BaselineStats,
    Check,
    TopicStats,
    evaluate,
    passed,
    record_topics,
)


def _year_from_stamp(sec: int) -> int:
    return 1970 + max(sec, 0) // 31536000


def _orientation_invalid(x: float, y: float, z: float, w: float) -> bool:
    return math.sqrt(x * x + y * y + z * z + w * w) < 0.5


def stats_from_metadata(bag_dir: Path, session: str) -> BaselineStats:
    meta_path = bag_dir / "metadata.yaml"
    payload = yaml.safe_load(meta_path.read_text(encoding="utf-8"))
    info = payload.get("rosbag2_bagfile_information") or payload
    duration = info.get("duration")
    if isinstance(duration, dict):
        duration_sec = int(duration.get("nanoseconds") or 0) / 1e9
    else:
        duration_sec = float(duration or 0)
    topics: dict[str, TopicStats] = {}
    for item in info.get("topics_with_message_count") or []:
        name = item["topic_metadata"]["name"]
        count = int(item.get("message_count") or 0)
        hz = (count / duration_sec) if duration_sec > 0 else None
        topics[name] = TopicStats(name=name, count=count, hz=hz)
    start_ns = int((info.get("starting_time") or {}).get("nanoseconds_since_epoch") or 0)
    clock_ok = True
    if start_ns:
        clock_ok = _year_from_stamp(start_ns // 1_000_000_000) >= 2024
    return BaselineStats(
        session=session,
        duration_sec=duration_sec,
        topics=topics,
        clock_ok=clock_ok,
        metadata_only=True,
        notes=["metadata-only: TF pairs, VO inliers, and quaternions need --live or rosbag2 on the robot"],
    )


def _try_rosbag(bag_dir: Path, session: str) -> BaselineStats | None:
    try:
        from rclpy.serialization import deserialize_message
        from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
        from rosidl_runtime_py.utilities import get_message
    except ImportError:
        return None

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_dir), storage_id="sqlite3"),
        ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    counts: dict[str, int] = defaultdict(int)
    frames: dict[str, set[str]] = defaultdict(set)
    invalid_orient: dict[str, int] = defaultdict(int)
    lost_true = 0
    inliers: list[int] = []
    tf_pairs: set[tuple[str, str]] = set()
    min_year = 9999
    t0 = None
    t1 = None
    type_map = {item.name: item.type for item in reader.get_all_topics_and_types()}
    while reader.has_next():
        name, data, timestamp = reader.read_next()
        counts[name] += 1
        t1 = timestamp
        if t0 is None:
            t0 = timestamp
        min_year = min(min_year, _year_from_stamp(timestamp // 1_000_000_000))
        msg_type = type_map.get(name)
        if not msg_type:
            continue
        try:
            msg = deserialize_message(data, get_message(msg_type))
        except Exception:
            continue
        header = getattr(msg, "header", None)
        if header is not None and getattr(header, "frame_id", ""):
            frames[name].add(header.frame_id)
        if name in {"/tf", "/tf_static"}:
            for item in getattr(msg, "transforms", []):
                tf_pairs.add((item.header.frame_id, item.child_frame_id))
        if name in {"/odom", "/wheel/odom"}:
            q = msg.pose.pose.orientation
            if _orientation_invalid(q.x, q.y, q.z, q.w):
                invalid_orient[name] += 1
        if name == "/odom_info_lite":
            if bool(getattr(msg, "lost", False)):
                lost_true += 1
            inliers.append(int(getattr(msg, "inliers", 0)))
    duration = 0.0 if t0 is None or t1 is None else max((t1 - t0) / 1e9, 0.0)
    topics = {}
    for name, count in counts.items():
        topics[name] = TopicStats(
            name=name,
            count=count,
            hz=(count / duration) if duration > 0 else None,
            frame_ids=frames.get(name, set()),
            invalid_orientation=invalid_orient.get(name, 0),
            lost_true=lost_true if name == "/odom_info_lite" else 0,
            inliers_min=(float(min(inliers)) if name == "/odom_info_lite" and inliers else None),
            inliers_median=(float(statistics.median(inliers)) if name == "/odom_info_lite" and inliers else None),
        )
    return BaselineStats(
        session=session,
        duration_sec=duration,
        topics=topics,
        tf_pairs=tf_pairs,
        clock_ok=min_year >= 2024 if min_year != 9999 else True,
    )


def collect_live(session: str, seconds: float) -> BaselineStats:
    import time

    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan
    from tf2_msgs.msg import TFMessage

    try:
        from rtabmap_msgs.msg import OdomInfo
    except ImportError:
        OdomInfo = None

    sensor_qos = QoSProfile(
        depth=20,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )
    static_qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )

    class Probe(Node):
        def __init__(self) -> None:
            super().__init__("r680_baseline_probe")
            self.counts: dict[str, int] = defaultdict(int)
            self.frames: dict[str, set[str]] = defaultdict(set)
            self.invalid_orient: dict[str, int] = defaultdict(int)
            self.tf_pairs: set[tuple[str, str]] = set()
            self.lost_true = 0
            self.inliers: list[int] = []
            self.min_year = 9999
            wanted = set(record_topics(session))
            for topic in wanted:
                if topic in {"/camera/infra1/image_rect_raw", "/camera/depth/image_rect_raw"}:
                    self.create_subscription(Image, topic, self._make_header_cb(topic), sensor_qos)
                elif topic.endswith("camera_info"):
                    self.create_subscription(CameraInfo, topic, self._make_header_cb(topic), sensor_qos)
                elif topic in {"/odom", "/wheel/odom"}:
                    self.create_subscription(Odometry, topic, self._make_odom_cb(topic), sensor_qos)
                elif topic == "/imu/data_raw":
                    self.create_subscription(Imu, topic, self._make_header_cb(topic), sensor_qos)
                elif topic == "/scan":
                    scan_qos = QoSProfile(
                        depth=10,
                        reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.VOLATILE,
                        history=HistoryPolicy.KEEP_LAST,
                    )
                    self.create_subscription(LaserScan, topic, self._make_header_cb(topic), scan_qos)
                elif topic == "/tf":
                    self.create_subscription(TFMessage, topic, self._on_tf, 50)
                elif topic == "/tf_static":
                    self.create_subscription(TFMessage, topic, self._on_tf_static, static_qos)
                elif topic == "/odom_info_lite" and OdomInfo is not None:
                    self.create_subscription(OdomInfo, topic, self._on_info, sensor_qos)
                elif topic == "/map":
                    from nav_msgs.msg import OccupancyGrid

                    map_qos = QoSProfile(
                        depth=1,
                        reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL,
                        history=HistoryPolicy.KEEP_LAST,
                    )
                    self.create_subscription(OccupancyGrid, topic, self._make_header_cb(topic), map_qos)
                elif topic == "/rtabmap/rgbd_image":
                    continue

        def _stamp_year(self, stamp) -> None:
            self.min_year = min(self.min_year, _year_from_stamp(int(stamp.sec)))

        def _make_header_cb(self, topic: str):
            def _cb(msg) -> None:
                self.counts[topic] += 1
                if getattr(msg, "header", None) is not None:
                    if msg.header.frame_id:
                        self.frames[topic].add(msg.header.frame_id)
                    self._stamp_year(msg.header.stamp)

            return _cb

        def _make_odom_cb(self, topic: str):
            def _cb(msg: Odometry) -> None:
                self.counts[topic] += 1
                self._stamp_year(msg.header.stamp)
                if msg.header.frame_id:
                    self.frames[topic].add(msg.header.frame_id)
                q = msg.pose.pose.orientation
                if _orientation_invalid(q.x, q.y, q.z, q.w):
                    self.invalid_orient[topic] += 1

            return _cb

        def _on_tf(self, msg: TFMessage) -> None:
            self.counts["/tf"] += 1
            self._ingest_tf(msg)

        def _on_tf_static(self, msg: TFMessage) -> None:
            self.counts["/tf_static"] += 1
            self._ingest_tf(msg)

        def _ingest_tf(self, msg: TFMessage) -> None:
            for item in msg.transforms:
                self.tf_pairs.add((item.header.frame_id, item.child_frame_id))
                self._stamp_year(item.header.stamp)

        def _on_info(self, msg) -> None:
            self.counts["/odom_info_lite"] += 1
            if bool(getattr(msg, "lost", False)):
                self.lost_true += 1
            self.inliers.append(int(getattr(msg, "inliers", 0)))

    rclpy.init()
    node = Probe()
    end = time.time() + seconds
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    duration = seconds
    topics = {}
    for name, count in node.counts.items():
        topics[name] = TopicStats(
            name=name,
            count=count,
            hz=(count / duration) if duration > 0 else None,
            frame_ids=node.frames.get(name, set()),
            invalid_orientation=node.invalid_orient.get(name, 0),
            lost_true=node.lost_true if name == "/odom_info_lite" else 0,
            inliers_min=(float(min(node.inliers)) if name == "/odom_info_lite" and node.inliers else None),
            inliers_median=(
                float(statistics.median(node.inliers)) if name == "/odom_info_lite" and node.inliers else None
            ),
        )
    stats = BaselineStats(
        session=session,
        duration_sec=duration,
        topics=topics,
        tf_pairs=node.tf_pairs,
        clock_ok=node.min_year >= 2024 if node.min_year != 9999 else True,
    )
    node.destroy_node()
    rclpy.shutdown()
    return stats


def checks_to_dict(checks: list[Check]) -> list[dict]:
    return [
        {"name": item.name, "ok": item.ok, "required": item.required, "detail": item.detail}
        for item in checks
    ]


def stats_to_dict(stats: BaselineStats) -> dict:
    return {
        "session": stats.session,
        "duration_sec": stats.duration_sec,
        "clock_ok": stats.clock_ok,
        "tf_pairs": sorted(f"{a}→{b}" for a, b in stats.tf_pairs),
        "metadata_only": stats.metadata_only,
        "notes": stats.notes,
        "topics": {
            name: {
                "count": topic.count,
                "hz": topic.hz,
                "frame_ids": sorted(topic.frame_ids),
                "invalid_orientation": topic.invalid_orientation,
                "lost_true": topic.lost_true,
                "inliers_min": topic.inliers_min,
                "inliers_median": topic.inliers_median,
            }
            for name, topic in stats.topics.items()
        },
    }


def print_report(checks: list[Check], ok: bool) -> None:
    print("PASS" if ok else "FAIL")
    for item in checks:
        flag = "ok" if item.ok else ("FAIL" if item.required else "warn")
        print(f"  [{flag}] {item.name}: {item.detail}")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Check an R680 baseline recording")
    parser.add_argument("--session", choices=["vision", "vision_base", "lidar"], required=True)
    parser.add_argument("--bag", type=Path, help="rosbag2 directory with metadata.yaml")
    parser.add_argument("--live", action="store_true", help="sample live topics on the robot")
    parser.add_argument("--seconds", type=float, default=15.0)
    parser.add_argument("--print-topics", action="store_true")
    parser.add_argument("--json-out", type=Path)
    args = parser.parse_args(argv)
    if args.print_topics:
        print(" ".join(record_topics(args.session)))
        return 0
    if args.live:
        stats = collect_live(args.session, args.seconds)
    elif args.bag:
        stats = _try_rosbag(args.bag, args.session) or stats_from_metadata(args.bag, args.session)
    else:
        parser.error("need --bag, --live, or --print-topics")
        return 2
    checks = evaluate(stats)
    ok = passed(checks)
    print_report(checks, ok)
    if stats.notes:
        for note in stats.notes:
            print(f"  note: {note}")
    report = {"passed": ok, "stats": stats_to_dict(stats), "checks": checks_to_dict(checks)}
    if args.json_out:
        args.json_out.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
