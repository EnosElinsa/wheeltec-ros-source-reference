"""Exercise replay isolation and calibration gates without importing ROS launch."""
import json
import hashlib
from pathlib import Path

import pytest

from wheeltec_vision_nav.vins_experiment import experiment_nodes, playback_command, validate_run


def test_stereo_uses_exact_sync_and_cannot_publish_driving_tf():
    specs = experiment_nodes("stereo", "/share", "/config.yaml")
    odom = next(n for n in specs if n["executable"] == "stereo_odometry")
    assert odom["parameters"][-1]["publish_tf"] is False
    assert odom["parameters"][-1]["approx_sync"] is False
    assert odom["parameters"][-1]["Odom/Strategy"] == "9"
    assert ("odom", "/odom_vins") in odom["remappings"]
    assert ("imu", "/vins/unused_imu") in odom["remappings"]
    assert not any(n["package"] in {"nav2_bringup", "turn_on_wheeltec_robot"} for n in specs)


def test_bag_playback_excludes_original_tf_odom_commands_and_clock():
    cmd = playback_command("/bag path", rate=1.)
    topics = cmd[cmd.index("--topics") + 1:]
    assert "/tf_static" in topics and "/wheel/odom" in topics
    assert "/camera/infra2/image_rect_raw" in topics
    assert not {"/odom", "/tf", "/cmd_vel", "/clock"}.intersection(topics)
    assert cmd[cmd.index("--clock") + 1] == "100"


def test_vio_cannot_start_without_exact_bag_and_config_audit(tmp_path):
    bag = tmp_path / "bag"
    bag.mkdir()
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information: {}")
    cfg = tmp_path / "config.yaml"
    cfg.write_text("imu: 1\nnum_of_cam: 2\n")
    audit = tmp_path / "audit.json"
    audit.write_text(json.dumps({"schema_version": 1, "kind": "vins_sensor_audit", "vio_ready": True,
        "bag_uri": str(bag.resolve()), "bag_metadata_sha256": "wrong", "calibration_sha256": "wrong"}))
    with pytest.raises(ValueError, match="audit|digest|calibration"):
        validate_run("vio", bag, cfg, audit)


def test_baseline_needs_no_vins_config(tmp_path):
    bag = tmp_path / "bag"
    bag.mkdir()
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information: {}")
    validate_run("baseline", bag, None, None)


def test_invalid_replay_rate_is_rejected():
    with pytest.raises(ValueError):
        playback_command("/bag", rate=float("nan"))


@pytest.mark.parametrize("mode,subscribe", [("stereo", False), ("vio", True)])
def test_imu_subscription_enabled_only_for_vio(mode, subscribe):
    specs = experiment_nodes(mode, "/share", "/config.yaml")
    odom = next(n for n in specs if n["executable"] == "stereo_odometry")
    # OdometryROS creates the IMU subscription only when this flag is true.
    assert odom["parameters"][-1]["wait_imu_to_init"] is subscribe


def test_audited_bag_payload_and_generated_config_are_bound_to_replay(tmp_path):
    from wheeltec_vision_nav.vins_config import make_config
    from wheeltec_vision_nav.vins_experiment import bag_fingerprint, digest
    bag = tmp_path / "bag"
    bag.mkdir()
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information:\n  relative_file_paths: [trial_0.db3]\n")
    (bag / "trial_0.db3").write_bytes(b"original bag payload")
    p = [420., 0., 420., 0., 0., 420., 240., 0., 0., 0., 1., 0.]
    left = {"width": 848, "height": 480, "p": p, "frame_id": "left"}
    right = {**left, "p": [*p[:3], -21., *p[4:]], "frame_id": "right"}
    report = {"schema_version": 1, "kind": "vins_sensor_audit", "stereo_ready": True,
              "bag_uri": str(bag.resolve()), "bag_metadata_sha256": digest(bag / "metadata.yaml"),
              "bag_files": bag_fingerprint(bag), "camera_models": {"left": left, "right": right}}
    audit = tmp_path / "audit.json"
    audit.write_text(json.dumps(report))
    cfg = make_config(audit, tmp_path / "config", "stereo")
    validate_run("stereo", bag, cfg, audit)
    # Do not allow camera calibration to change after its image audit.
    camera = cfg.parent / "left.yaml"
    original_camera = camera.read_text()
    camera.write_text(original_camera.replace("fx: 420.0", "fx: 421.0"))
    with pytest.raises(ValueError, match="CameraInfo"):
        validate_run("stereo", bag, cfg, audit)
    camera.write_text(original_camera)
    # The pinned wrapper feeds IMU only through the image time; a future td
    # would return a lagged solver pose stamped as the current image.
    import yaml
    from wheeltec_vision_nav.vins_audit import load_calibration
    config = load_calibration(cfg)
    config.update(imu=1, td=.1, estimate_td=0)
    cfg.write_text(yaml.safe_dump(config))
    report.update(vio_ready=True, calibration_sha256=digest(cfg))
    audit.write_text(json.dumps(report))
    with pytest.raises(ValueError, match="td|time offset"):
        validate_run("vio", bag, cfg, audit)
    config.update(imu=0, td=0.)
    cfg.write_text(yaml.safe_dump(config))
    # Same metadata, different sensor payload must invalidate the previous audit.
    (bag / "trial_0.db3").write_bytes(b"changed bag payload")
    with pytest.raises(ValueError, match="payload|bag"):
        validate_run("stereo", bag, cfg, audit)


def test_fingerprint_rejects_files_outside_the_bag(tmp_path):
    from wheeltec_vision_nav.vins_experiment import bag_fingerprint
    bag = tmp_path / "bag"
    bag.mkdir()
    (tmp_path / "outside.db3").write_bytes(b"outside")
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information:\n  relative_file_paths: [../outside.db3]\n")
    with pytest.raises(ValueError, match="within|relative"):
        bag_fingerprint(bag)


def test_live_runtime_profile_reused_in_baseline_without_changing_audited_vins():
    from wheeltec_vision_nav.runtime_config import load_runtime_config
    package = Path(__file__).parents[1]
    runtime = load_runtime_config(package / "config/runtime.yaml")
    runtime["perception"]["max_features"] = 900
    runtime["safety"]["min_inliers"] = 15
    runtime["safety"]["wheel_fresh_s"] = .2
    baseline = experiment_nodes("baseline", package, runtime=runtime)
    odom = next(n for n in baseline if n["executable"] == "rgbd_odometry")
    assert any(p.get("Vis/MaxFeatures") == "900" for p in odom["parameters"] if isinstance(p, dict))
    inputs = next(n for n in baseline if n["executable"] == "ekf_inputs")
    assert any(p.get("min_inliers") == 15 and p.get("wheel_prior_used") is True for p in inputs["parameters"])
    assert any(p.get("wheel_timeout_s") == .2 for p in inputs["parameters"])
    for mode in ("stereo", "vio"):
        assert experiment_nodes(mode, package, "/audited/config.yaml", runtime=runtime) == experiment_nodes(
            mode, package, "/audited/config.yaml")
