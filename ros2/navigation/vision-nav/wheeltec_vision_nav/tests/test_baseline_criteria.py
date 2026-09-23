from wheeltec_vision_nav.baseline_criteria import (
    BaselineStats,
    TopicStats,
    evaluate,
    passed,
    record_topics,
)


def _topic(name: str, hz: float, count: int = 100) -> TopicStats:
    return TopicStats(name=name, count=count, hz=hz)


def _vision_topics(**extra: TopicStats) -> dict[str, TopicStats]:
    topics = {
        "/camera/infra1/image_rect_raw": _topic("/camera/infra1/image_rect_raw", 15.0),
        "/camera/infra1/camera_info": _topic("/camera/infra1/camera_info", 15.0),
        "/camera/depth/image_rect_raw": _topic("/camera/depth/image_rect_raw", 15.0),
        "/camera/depth/camera_info": _topic("/camera/depth/camera_info", 15.0),
        "/odom": _topic("/odom", 15.0),
        "/tf": _topic("/tf", 50.0),
        "/tf_static": _topic("/tf_static", 1.0, count=2),
        "/odom_info_lite": TopicStats(
            name="/odom_info_lite", count=90, hz=15.0, lost_true=0, inliers_min=40.0
        ),
    }
    topics.update(extra)
    return topics


def test_vision_base_passes_with_expected_rates():
    stats = BaselineStats(
        session="vision_base",
        duration_sec=90.0,
        topics=_vision_topics(
            **{
                "/imu/data_raw": _topic("/imu/data_raw", 50.0, count=4500),
                "/wheel/odom": _topic("/wheel/odom", 20.0),
            }
        ),
        tf_pairs={
            ("odom", "base_footprint"),
            ("base_link", "camera_link"),
            ("base_footprint", "gyro_link"),
        },
    )
    checks = evaluate(stats)
    assert passed(checks), checks


def test_missing_imu_fails_vision_base():
    stats = BaselineStats(
        session="vision_base",
        duration_sec=90.0,
        topics=_vision_topics(),
        tf_pairs={
            ("odom", "base_footprint"),
            ("base_link", "camera_link"),
            ("base_footprint", "gyro_link"),
        },
    )
    checks = evaluate(stats)
    assert not passed(checks)
    assert any(item.name == "topic /imu/data_raw" and not item.ok for item in checks)


def test_epoch_clock_fails():
    stats = BaselineStats(
        session="lidar",
        duration_sec=40.0,
        topics={"/scan": TopicStats(name="/scan", count=480, hz=12.0, frame_ids={"laser"})},
        clock_ok=False,
    )
    checks = evaluate(stats)
    assert not passed(checks)
    assert any(item.name == "clock" and not item.ok for item in checks)


def test_lidar_rate_and_frame():
    stats = BaselineStats(
        session="lidar",
        duration_sec=40.0,
        topics={"/scan": TopicStats(name="/scan", count=480, hz=12.0, frame_ids={"laser"})},
        tf_pairs=set(),
    )
    checks = evaluate(stats)
    assert passed(checks)
    tf_check = next(item for item in checks if item.name.startswith("tf "))
    assert not tf_check.required


def test_metadata_only_does_not_require_tf():
    stats = BaselineStats(
        session="vision",
        duration_sec=70.0,
        topics=_vision_topics(),
        metadata_only=True,
    )
    checks = evaluate(stats)
    assert passed(checks)
    assert any(item.name.startswith("tf ") and not item.required for item in checks)


def test_record_topics_do_not_mix_scan_into_vision():
    vision = record_topics("vision_base")
    assert "/scan" not in vision
    assert "/imu/data_raw" in vision
    assert "/scan" in record_topics("lidar")


def test_vision_base_warns_without_wheel_odom_tf():
    stats = BaselineStats(
        session="vision_base",
        duration_sec=90.0,
        topics=_vision_topics(
            **{
                "/imu/data_raw": _topic("/imu/data_raw", 50.0, count=4500),
                "/wheel/odom": _topic("/wheel/odom", 20.0),
            }
        ),
        tf_pairs={
            ("odom", "base_footprint"),
            ("base_link", "camera_link"),
            ("base_footprint", "gyro_link"),
        },
    )
    checks = evaluate(stats)
    assert passed(checks), checks
    wheel = next(item for item in checks if item.name == "tf base_footprint→wheel_odom")
    both = next(item for item in checks if item.name == "tf odom and wheel_odom")
    assert not wheel.ok and not wheel.required
    assert not both.ok and not both.required


def test_vision_quality_warns_on_lost_ratio_and_inliers():
    stats = BaselineStats(
        session="vision",
        duration_sec=70.0,
        topics=_vision_topics(
            **{
                "/odom_info_lite": TopicStats(
                    name="/odom_info_lite",
                    count=100,
                    hz=15.0,
                    lost_true=20,
                    inliers_min=8.0,
                    inliers_median=12.0,
                )
            }
        ),
        tf_pairs={("odom", "base_footprint"), ("base_link", "camera_link")},
    )
    checks = evaluate(stats)
    assert passed(checks), checks
    ratio = next(item for item in checks if item.name == "vo lost ratio")
    median = next(item for item in checks if item.name == "vo inliers median")
    rate = next(item for item in checks if item.name == "848x480 rate")
    assert not ratio.ok and not ratio.required
    assert not median.ok and not median.required
    assert rate.ok and not rate.required


def test_vision_base_passes_wheel_odom_tf_warns_when_present():
    stats = BaselineStats(
        session="vision_base",
        duration_sec=90.0,
        topics=_vision_topics(
            **{
                "/imu/data_raw": _topic("/imu/data_raw", 50.0, count=4500),
                "/wheel/odom": _topic("/wheel/odom", 20.0),
            }
        ),
        tf_pairs={
            ("odom", "base_footprint"),
            ("base_footprint", "wheel_odom"),
            ("base_link", "camera_link"),
            ("base_footprint", "gyro_link"),
        },
    )
    checks = evaluate(stats)
    assert passed(checks), checks
    both = next(item for item in checks if item.name == "tf odom and wheel_odom")
    assert both.ok and not both.required


def test_vision_base_warns_if_wheel_odom_parents_base_footprint():
    stats = BaselineStats(
        session="vision_base",
        duration_sec=90.0,
        topics=_vision_topics(
            **{
                "/imu/data_raw": _topic("/imu/data_raw", 50.0, count=4500),
                "/wheel/odom": _topic("/wheel/odom", 20.0),
            }
        ),
        tf_pairs={
            ("odom", "base_footprint"),
            ("wheel_odom", "base_footprint"),
            ("base_link", "camera_link"),
            ("base_footprint", "gyro_link"),
        },
    )
    checks = evaluate(stats)
    assert passed(checks), checks
    both = next(item for item in checks if item.name == "tf odom and wheel_odom")
    assert not both.ok and not both.required
