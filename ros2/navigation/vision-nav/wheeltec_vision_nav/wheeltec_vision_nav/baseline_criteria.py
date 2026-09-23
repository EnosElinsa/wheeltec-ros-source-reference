"""Pass/fail rules for R680 baseline bags. Pure Python, no ROS import."""

from __future__ import annotations

from dataclasses import dataclass, field

SESSIONS = ("vision", "vision_base", "lidar")

VISION_TOPICS = (
    "/camera/infra1/image_rect_raw",
    "/camera/infra1/camera_info",
    "/camera/depth/image_rect_raw",
    "/camera/depth/camera_info",
    "/odom",
    "/tf",
    "/tf_static",
)

VISION_BASE_EXTRA = ("/wheel/odom", "/imu/data_raw")
LIDAR_TOPICS = ("/scan",)
OPTIONAL_VISION = ("/rtabmap/rgbd_image", "/odom_info_lite", "/map")
LIDAR_RECORD = ("/scan", "/tf", "/tf_static")
RECORD_TOPICS = {
    "vision": VISION_TOPICS + OPTIONAL_VISION,
    "vision_base": VISION_TOPICS + OPTIONAL_VISION + VISION_BASE_EXTRA,
    "lidar": LIDAR_RECORD,
}

REQUIRED_TF = {
    "vision": (("odom", "base_footprint"), ("base_link", "camera_link")),
    "vision_base": (("odom", "base_footprint"), ("base_link", "camera_link"), ("base_footprint", "gyro_link")),
    "lidar": (("base_footprint", "laser"),),
}


@dataclass
class TopicStats:
    name: str
    count: int = 0
    hz: float | None = None
    frame_ids: set[str] = field(default_factory=set)
    min_stamp_year: int | None = None
    invalid_orientation: int = 0
    lost_true: int = 0
    inliers_min: float | None = None
    inliers_median: float | None = None


@dataclass
class BaselineStats:
    session: str
    duration_sec: float
    topics: dict[str, TopicStats]
    tf_pairs: set[tuple[str, str]] = field(default_factory=set)
    clock_ok: bool = True
    metadata_only: bool = False
    notes: list[str] = field(default_factory=list)


def record_topics(session: str) -> tuple[str, ...]:
    try:
        return RECORD_TOPICS[session]
    except KeyError as exc:
        raise ValueError(f"session must be one of {SESSIONS}") from exc


@dataclass
class Check:
    name: str
    ok: bool
    detail: str
    required: bool = True


def _topic(stats: BaselineStats, name: str) -> TopicStats | None:
    return stats.topics.get(name)


def _hz_in(topic: TopicStats | None, lo: float, hi: float) -> tuple[bool, str]:
    if topic is None or topic.count == 0:
        return False, "missing"
    if topic.hz is None:
        return False, f"count={topic.count}, hz unknown"
    ok = lo <= topic.hz <= hi
    return ok, f"{topic.hz:.2f} Hz (want {lo:g}–{hi:g})"


def _hz_min(topic: TopicStats | None, lo: float) -> tuple[bool, str]:
    if topic is None or topic.count == 0:
        return False, "missing"
    if topic.hz is None:
        return False, f"count={topic.count}, hz unknown"
    ok = topic.hz >= lo
    return ok, f"{topic.hz:.2f} Hz (want ≥ {lo:g})"


def evaluate(stats: BaselineStats) -> list[Check]:
    session = stats.session
    if session not in SESSIONS:
        return [Check("session", False, f"unknown session {session}")]

    checks: list[Check] = []
    min_duration = 30.0 if session == "lidar" else 60.0
    checks.append(
        Check(
            "duration",
            stats.duration_sec >= min_duration,
            f"{stats.duration_sec:.1f} s (want ≥ {min_duration:g})",
        )
    )
    checks.append(Check("clock", stats.clock_ok, "header stamps are not epoch/1970"))

    required = list(VISION_TOPICS if session.startswith("vision") else LIDAR_TOPICS)
    if session == "vision_base":
        required.extend(VISION_BASE_EXTRA)
    for name in required:
        topic = _topic(stats, name)
        present = topic is not None and topic.count > 0
        checks.append(Check(f"topic {name}", present, "present" if present else "missing"))

    if session.startswith("vision"):
        ok, detail = _hz_in(_topic(stats, "/camera/infra1/image_rect_raw"), 10.0, 20.0)
        checks.append(Check("infra rate", ok, detail))
        ok, detail = _hz_in(_topic(stats, "/camera/depth/image_rect_raw"), 10.0, 20.0)
        checks.append(Check("depth rate", ok, detail))
        ok, detail = _hz_min(_topic(stats, "/odom"), 5.0)
        checks.append(Check("odom rate", ok, detail))
        odom = _topic(stats, "/odom")
        if odom and odom.count:
            if stats.metadata_only:
                checks.append(
                    Check("odom orientation", True, "not in metadata.yaml", required=False)
                )
            else:
                bad = odom.invalid_orientation
                checks.append(
                    Check(
                        "odom orientation",
                        bad == 0,
                        f"{bad} invalid quaternion(s) in {odom.count} messages",
                    )
                )
        info = _topic(stats, "/odom_info_lite")
        if info and info.count:
            lost_ok = info.lost_true == 0
            checks.append(
                Check(
                    "vo lost flag",
                    lost_ok,
                    f"lost=true on {info.lost_true}/{info.count} messages",
                    required=False,
                )
            )
            lost_ratio = info.lost_true / info.count
            checks.append(
                Check(
                    "vo lost ratio",
                    lost_ratio <= 0.10,
                    f"{lost_ratio:.1%} lost (warn if > 10% on textured walls)",
                    required=False,
                )
            )
            inliers_ok = info.inliers_min is not None and info.inliers_min > 20
            checks.append(
                Check(
                    "vo inliers",
                    inliers_ok,
                    f"min inliers={info.inliers_min} (want > 20)",
                    required=False,
                )
            )
            median = info.inliers_median if info.inliers_median is not None else info.inliers_min
            checks.append(
                Check(
                    "vo inliers median",
                    median is not None and median > 20,
                    f"median inliers={median} (want > 20)",
                    required=False,
                )
            )
        elif session.startswith("vision"):
            checks.append(Check("vo diagnostics", False, "/odom_info_lite missing", required=False))

        infra = _topic(stats, "/camera/infra1/image_rect_raw")
        if infra and infra.hz is not None:
            checks.append(
                Check(
                    "848x480 rate",
                    infra.hz >= 10.0,
                    f"{infra.hz:.2f} Hz (warn if < 10; fall back to 640x480)",
                    required=False,
                )
            )

    if session == "vision_base":
        ok, detail = _hz_min(_topic(stats, "/imu/data_raw"), 10.0)
        checks.append(Check("imu rate", ok, detail))
        ok, detail = _hz_min(_topic(stats, "/wheel/odom"), 5.0)
        checks.append(Check("wheel odom rate", ok, detail))
        imu = _topic(stats, "/imu/data_raw")
        if imu and imu.frame_ids and "gyro_link" not in imu.frame_ids:
            checks.append(
                Check(
                    "imu frame",
                    False,
                    f"frame_id={sorted(imu.frame_ids)} (want gyro_link)",
                    required=False,
                )
            )
        wheel_child = ("base_footprint", "wheel_odom") in stats.tf_pairs
        wheel_as_parent = ("wheel_odom", "base_footprint") in stats.tf_pairs
        odom_tf = ("odom", "base_footprint") in stats.tf_pairs
        checks.append(
            Check(
                "tf base_footprint→wheel_odom",
                wheel_child,
                "present" if wheel_child else "missing (inverse wheel pose)",
                required=False,
            )
        )
        checks.append(
            Check(
                "tf odom and wheel_odom",
                odom_tf and wheel_child and not wheel_as_parent,
                "odom parents base_footprint; wheel_odom is a child"
                if odom_tf and wheel_child and not wheel_as_parent
                else f"odom_parent={odom_tf}, wheel_child={wheel_child}, wheel_as_parent={wheel_as_parent}",
                required=False,
            )
        )

    if session == "lidar":
        ok, detail = _hz_in(_topic(stats, "/scan"), 8.0, 16.0)
        checks.append(Check("scan rate", ok, detail))
        scan = _topic(stats, "/scan")
        if scan and scan.frame_ids:
            checks.append(
                Check("scan frame", "laser" in scan.frame_ids, f"frame_id={sorted(scan.frame_ids)}")
            )

    expected_tf = REQUIRED_TF[session]
    for parent, child in expected_tf:
        ok = (parent, child) in stats.tf_pairs
        required = session != "lidar" and not stats.metadata_only
        detail = "present" if ok else ("missing (metadata-only bag)" if stats.metadata_only else "missing")
        checks.append(Check(f"tf {parent}→{child}", ok, detail, required=required))

    return checks


def passed(checks: list[Check]) -> bool:
    return all(item.ok for item in checks if item.required)
