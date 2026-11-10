"""ROS-independent experiment contract, input whitelist and replay gates."""
import hashlib
import json
import math
from pathlib import Path

RAW_TOPICS = (
    "/camera/infra1/image_rect_raw", "/camera/infra2/image_rect_raw",
    "/camera/infra1/camera_info", "/camera/infra2/camera_info",
    "/camera/depth/image_rect_raw", "/camera/depth/camera_info",
    "/camera/infra1/metadata", "/camera/infra2/metadata", "/camera/depth/metadata",
    "/imu/data_raw", "/wheel/odom", "/tf_static",
)
RECORD_TOPICS = RAW_TOPICS + ("/tf", "/odom", "/odom_vo", "/odom_info_lite", "/vision/emitter_pair/status",
                             "/vision/emitter_pair/provenance", "/vision/ekf_inputs/status", "/vision/health/status")


def digest(path):
    value = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            value.update(block)
    return value.hexdigest()


def bag_fingerprint(bag):
    """Bind reports to bag payloads too; metadata alone does not identify samples."""
    import yaml
    bag = Path(bag).resolve()
    metadata = yaml.safe_load((bag / "metadata.yaml").read_text(encoding="utf-8-sig"))
    paths = metadata.get("rosbag2_bagfile_information", {}).get("relative_file_paths")
    if not isinstance(paths, list) or not paths or any(not isinstance(p, str) or not p for p in paths):
        raise ValueError("bag metadata requires nonempty relative_file_paths")
    if len(set(paths)) != len(paths):
        raise ValueError("bag metadata has duplicate relative_file_paths")
    files = []
    for name in sorted(paths):
        path = (bag / name).resolve()
        if Path(name).is_absolute() or not path.is_relative_to(bag) or not path.is_file():
            raise ValueError("bag payload paths must be files within the bag directory")
        files.append({"path": name, "size_bytes": path.stat().st_size, "sha256": digest(path)})
    return files


def playback_command(bag, rate=1.):
    if not math.isfinite(rate) or rate <= 0:
        raise ValueError("replay rate must be positive and finite")
    return ["ros2", "bag", "play", str(bag), "--clock", "100", "--rate", str(rate),
            "--delay", "3", "--topics", *RAW_TOPICS]


def validate_run(mode, bag, config=None, audit=None):
    if mode not in {"baseline", "stereo", "vio"}:
        raise ValueError("mode must be baseline, stereo or vio")
    bag = Path(bag).resolve()
    if not (bag / "metadata.yaml").is_file():
        raise ValueError("bag must contain metadata.yaml")
    if mode == "baseline":
        return
    if not config or not Path(config).is_file() or not audit or not Path(audit).is_file():
        raise ValueError("stereo/vio require a VINS config and sensor audit")
    report = json.loads(Path(audit).read_text(encoding="utf-8"))
    key = "vio_ready" if mode == "vio" else "stereo_ready"
    if (report.get("schema_version") != 1 or report.get("kind") != "vins_sensor_audit"
            or report.get(key) is not True or Path(report.get("bag_uri", "")).resolve() != bag
            or report.get("bag_metadata_sha256") != digest(bag / "metadata.yaml")):
        raise ValueError("sensor audit is failed or bound to another bag; rerun audit")
    if report.get("bag_files") != bag_fingerprint(bag):
        raise ValueError("bag payload differs from sensor audit; rerun audit")
    if mode == "vio" and report.get("calibration_sha256") != digest(config):
        raise ValueError("calibration digest differs from VIO audit; rerun audit")
    from wheeltec_vision_nav.vins_audit import load_calibration
    cfg = load_calibration(config)
    if cfg.get("imu") != int(mode == "vio") or cfg.get("num_of_cam") != 2:
        raise ValueError("VINS config must select matching mode and exactly two cameras")
    if cfg.get("estimate_extrinsic") != 0:
        raise ValueError("evaluation requires fixed, previously calibrated extrinsics")
    if mode == "vio" and (cfg.get("estimate_td") != 0 or cfg.get("td", 0.) != 0.):
        raise ValueError("pinned RTAB-Map wrapper requires td=0 and estimate_td=0; synchronize sensor clocks first")
    for side, key in (("left", "cam0_calib"), ("right", "cam1_calib")):
        camera = load_calibration(Path(config).resolve().parent / cfg.get(key, ""))
        model = report["camera_models"][side]
        p = model["p"]
        want = {"fx": p[0], "fy": p[5], "cx": p[2], "cy": p[6]}
        if (camera.get("image_width") != model["width"] or camera.get("image_height") != model["height"]
                or camera.get("model_type") != "PINHOLE"
                or any(not math.isclose(float(camera.get("projection_parameters", {}).get(k, float("nan"))), v,
                                        rel_tol=1e-6, abs_tol=1e-6) for k, v in want.items())
                or any(float(v) != 0 for v in camera.get("distortion_parameters", {}).values())):
            raise ValueError("camera config must match audited rectified CameraInfo")
    if mode == "vio":
        from wheeltec_vision_nav.vins_tf import validate_bag_extrinsics
        validate_bag_extrinsics(bag, config, report)


def experiment_nodes(mode, share, config="", runtime=None):
    """Return concrete Node kwargs, also inspected by non-ROS tests."""
    def node(package, executable, name, params=None, remaps=None):
        return dict(package=package, executable=executable, name=name, output="screen",
                    parameters=[*(params or []), {"use_sim_time": True}], remappings=remaps or [])

    if mode == "baseline":
        # Existing A pipeline without starting the chassis or playing captured dynamic TF.
        nodes = [
            node("wheeltec_vision_nav", "emitter_pair", "vision_emitter_pair"),
            node("rtabmap_sync", "rgbd_sync", "vision_rgbd_sync", [{"approx_sync": True,
                 "approx_sync_max_interval": .10, "qos": 2}], [
                ("rgb/image", "/vision/paired/infra1/image_rect_raw"),
                ("rgb/camera_info", "/vision/paired/infra1/camera_info"),
                ("depth/image", "/vision/paired/depth/image_rect_raw"), ("rgbd_image", "/rtabmap/rgbd_image")]),
            node("rtabmap_odom", "rgbd_odometry", "vision_rgbd_odometry", [
                str(Path(share) / "config/rtabmap_visual.yaml"), {"frame_id": "base_footprint",
                "odom_frame_id": "odom", "publish_tf": False, "subscribe_rgbd": True,
                "approx_sync": False, "qos": 2, "guess_frame_id": "wheel_odom",
                "guess_min_translation": 0., "guess_min_rotation": 0., "wait_for_transform": .5}],
                [("rgbd_image", "/rtabmap/rgbd_image"), ("odom", "/odom_vo")]),
            node("wheeltec_vision_nav", "wheel_odom_tf", "vision_wheel_odom_tf"),
            node("wheeltec_vision_nav", "ekf_inputs", "vision_ekf_inputs"),
            node("robot_localization", "ekf_node", "vision_ekf", [str(Path(share) / "config/ekf_visual.yaml")],
                 [("odometry/filtered", "/odom")]),
        ]
        if runtime is not None:
            from wheeltec_vision_nav.runtime_config import validate_runtime_config
            validate_runtime_config(runtime)
            for spec in nodes:
                if spec["executable"] == "rgbd_odometry":
                    spec["parameters"].insert(-1, {"Vis/MaxFeatures": str(runtime["perception"]["max_features"])})
                elif spec["executable"] == "ekf_inputs":
                    spec["parameters"].insert(-1, {"wheel_prior_used": True,
                        "min_inliers": runtime["safety"]["min_inliers"],
                        "wheel_timeout_s": runtime["safety"]["wheel_fresh_s"],
                        "quality_timeout_s": runtime["safety"]["vo_timeout_s"]})
        return nodes
    if mode not in {"stereo", "vio"}:
        raise ValueError("unknown experiment mode")
    params = {"frame_id": "base_footprint", "odom_frame_id": "vins_odom", "publish_tf": False,
              "approx_sync": False, "qos": 2, "qos_imu": 2, "queue_size": 60,
              "wait_for_transform": .5, "wait_imu_to_init": mode == "vio",
              "Odom/Strategy": "9", "OdomVINSFusion/ConfigPath": str(config),
              "Rtabmap/ImagesAlreadyRectified": "true", "Odom/ResetCountdown": "0"}
    spec = node("rtabmap_odom", "stereo_odometry", "vins_odometry", [params], [
        ("left/image_rect", "/vins/stereo/left/image_rect"),
        ("right/image_rect", "/vins/stereo/right/image_rect"),
        ("left/camera_info", "/vins/stereo/left/camera_info"),
        ("right/camera_info", "/vins/stereo/right/camera_info"),
        ("imu", "/imu/data_raw" if mode == "vio" else "/vins/unused_imu"),
        ("odom", "/odom_vins"), ("odom_info", "/vins/odom_info_full"),
        ("odom_info_lite", "/vins/odom_info")])
    # Keep derived parameter dict last, making explicit values override common defaults.
    spec["parameters"].reverse()
    return [node("wheeltec_vision_nav", "vins_stereo", "vins_stereo"), spec]
