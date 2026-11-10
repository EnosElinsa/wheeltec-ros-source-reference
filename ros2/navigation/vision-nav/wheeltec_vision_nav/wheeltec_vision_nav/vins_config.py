"""Generate OpenCV configs from audited rectified intrinsics, never from D435i defaults."""
import argparse
import json
from pathlib import Path
import yaml
from wheeltec_vision_nav.vins_audit import load_calibration
from wheeltec_vision_nav.vins_stereo_logic import valid_rectified_pair


class Matrix(dict):
    pass


class Dumper(yaml.SafeDumper):
    pass


Dumper.add_representer(Matrix, lambda dumper, value:
    dumper.represent_mapping("tag:yaml.org,2002:opencv-matrix", value))
Dumper.add_representer(list, lambda dumper, value:
    dumper.represent_sequence("tag:yaml.org,2002:seq", value, flow_style=True))


def matrix(values):
    if isinstance(values, dict):
        values = values["data"]
    elif values and isinstance(values[0], (list, tuple)):
        values = [v for row in values for v in row]
    return Matrix(rows=4, cols=4, dt="d", data=[float(v) for v in values])


def write_opencv(path, value):
    # OpenCV FileStorage requires the version directive and tagged matrices.
    text = "%YAML:1.0\n---\n" + yaml.dump(value, Dumper=Dumper, sort_keys=False)
    with Path(path).open("x", encoding="utf-8", newline="\n") as f:
        f.write(text)


def make_config(audit_path, output_dir, mode, calibration=None):
    if mode not in {"stereo", "vio"}:
        raise ValueError("mode must be stereo or vio")
    if mode == "vio" and not calibration:
        raise ValueError("VIO requires measured camera-IMU calibration and noise parameters")
    report = json.loads(Path(audit_path).read_text(encoding="utf-8"))
    if report.get("stereo_ready") is not True:
        raise ValueError("stereo sensor audit must pass first")
    models = report["camera_models"]
    if not valid_rectified_pair(models["left"], models["right"]):
        raise ValueError("invalid rectified stereo calibration")
    cfg = load_calibration(calibration) if calibration else {}
    if mode == "vio":
        # The sensor audit performs full VIO gating after generation. Fail early on missing fields.
        required = ("body_T_cam0", "body_T_cam1", "acc_n", "gyr_n", "acc_w", "gyr_w", "g_norm")
        if any(k not in cfg for k in required) or cfg.get("estimate_extrinsic") != 0:
            raise ValueError("calibration must contain fixed extrinsics and measured IMU noise")
        if cfg.get("td", 0.) != 0.:
            raise ValueError("pinned RTAB-Map wrapper cannot apply nonzero td; synchronize input clocks before VIO")
    p = models["right"]["p"]
    identity = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
    right_t = identity.copy()
    right_t[3] = -p[3] / p[0]
    cfg.update({"imu": int(mode == "vio"), "num_of_cam": 2,
        "cam0_calib": "left.yaml", "cam1_calib": "right.yaml",
        "image_width": models["left"]["width"], "image_height": models["left"]["height"],
        "estimate_extrinsic": 0, "multiple_thread": 0, "max_cnt": 200, "min_dist": 25,
        "show_track": 0, "flow_back": 1, "max_solver_time": .04, "max_num_iterations": 8,
        "keyframe_parallax": 10., "estimate_td": 0, "td": float(cfg.get("td", 0.)),
        "acc_n": cfg.get("acc_n", .1), "gyr_n": cfg.get("gyr_n", .01),
        "acc_w": cfg.get("acc_w", .001), "gyr_w": cfg.get("gyr_w", .0001),
        "g_norm": cfg.get("g_norm", 9.81)})
    # With imu=0 RTAB-Map replaces these with its received CameraInfo + base TF.
    cfg["body_T_cam0"] = matrix(cfg["body_T_cam0"] if mode == "vio" else identity)
    cfg["body_T_cam1"] = matrix(cfg["body_T_cam1"] if mode == "vio" else right_t)
    output_dir = Path(output_dir).resolve()
    output_dir.mkdir(parents=True, exist_ok=False)
    (output_dir / "solver_output").mkdir()
    cfg["output_path"] = (output_dir / "solver_output").as_posix()
    for side in ("left", "right"):
        model = models[side]
        p = model["p"]
        write_opencv(output_dir / f"{side}.yaml", {"model_type": "PINHOLE", "camera_name": side,
            "image_width": model["width"], "image_height": model["height"],
            "distortion_parameters": {"k1": 0., "k2": 0., "p1": 0., "p2": 0.},
            "projection_parameters": {"fx": p[0], "fy": p[5], "cx": p[2], "cy": p[6]}})
    write_opencv(output_dir / "vins.yaml", cfg)
    return output_dir / "vins.yaml"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--audit", required=True)
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--mode", choices=("stereo", "vio"), default="stereo")
    parser.add_argument("--calibration")
    args = parser.parse_args(argv)
    try:
        print(make_config(args.audit, args.output_dir, args.mode, args.calibration))
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.exit(2, f"vins_make_config: {exc}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
