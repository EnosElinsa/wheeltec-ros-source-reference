"""Planar A/B/C evaluation. Wheel odometry is never implicitly used as truth."""
import argparse
import bisect
import csv
import json
import math
from pathlib import Path


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def rmse(values):
    return math.sqrt(sum(v * v for v in values) / len(values)) if values else None


def finite_pose(row):
    return all(math.isfinite(float(row[k])) for k in ("x", "y", "yaw"))


def distribution(values):
    values = sorted(v for v in values if math.isfinite(v))
    if not values:
        return {"count": 0, "mean": None, "p95": None, "p99": None, "max": None}
    def percentile(fraction):
        index = fraction * (len(values) - 1)
        low, high = math.floor(index), math.ceil(index)
        return values[low] + (values[high] - values[low]) * (index - low)
    return {"count": len(values), "mean": sum(values) / len(values),
            "p95": percentile(.95), "p99": percentile(.99), "max": values[-1]}


class VisualHealth:
    def __init__(self, timeout_s=.5, min_inliers=10):
        self.timeout = timeout_s
        self.min_inliers = min_inliers
        self.last_stamp = -1
        self.last_wall = None
        self.lost = False

    def note_visual(self, stamp, lost, inliers, now):
        if stamp <= self.last_stamp:
            return
        self.last_stamp = stamp
        self.lost = bool(lost) or inliers < self.min_inliers
        if not self.lost:
            self.last_wall = now

    def status(self, now):
        if self.lost:
            return "lost"
        if self.last_wall is None:
            return "initializing"
        return "tracking" if now - self.last_wall <= self.timeout else "stale"


def summarize(rows, infos, frames, min_inliers=10):
    if not math.isfinite(min_inliers) or min_inliers <= 0:
        raise ValueError("min_inliers must be finite and positive")
    valid = [r for r in rows if finite_pose(r)]
    valid = sorted(valid, key=lambda r: r["stamp_ns"])
    frame_stamps = sorted(set(frames))
    by_stamp = {r["stamp_ns"]: r for r in infos}
    good = {s for s in frame_stamps if s in by_stamp and not by_stamp[s]["lost"] and by_stamp[s]["inliers"] >= min_inliers}
    first_good = min(good) if good else None
    lost_start, recoveries = None, []
    for s in frame_stamps:
        if first_good is None or s < first_good:
            continue
        if s not in good and lost_start is None:
            lost_start = s
        elif s in good and lost_start is not None:
            recoveries.append((s - lost_start) / 1e9)
            lost_start = None
    steps = [math.hypot(b["x"] - a["x"], b["y"] - a["y"]) for a, b in zip(valid, valid[1:])]
    duration = (valid[-1]["stamp_ns"] - valid[0]["stamp_ns"]) / 1e9 if len(valid) > 1 else 0.
    return {"schema_version": 1, "odometry_samples": len(rows), "invalid_odometry_samples": len(rows) - len(valid),
        "duration_s": duration, "odometry_hz": (len(valid) - 1) / duration if duration > 0 else None,
        "visual_input_frames": len(frame_stamps), "visual_valid_frames": len(good),
        "visual_min_inliers": min_inliers,
        "visual_lost_fraction": 1 - len(good) / len(frame_stamps) if frame_stamps else None,
        "initialization_delay_s": (first_good - frame_stamps[0]) / 1e9 if first_good is not None else None,
        "initialized": first_good is not None, "recovery_seconds": recoveries,
        "unrecovered_tail_s": ((frame_stamps[-1] - lost_start) / 1e9 if lost_start is not None else
                               (frame_stamps[-1] - frame_stamps[0]) / 1e9 if frame_stamps and not good else 0.),
        "path_length_m": sum(steps), "max_pose_step_m": max(steps, default=0.),
        "max_heading_step_rad": max((abs(angle(b["yaw"] - a["yaw"])) for a, b in zip(valid, valid[1:])), default=0.),
        "latency_s": distribution([r["latency_s"] for r in rows if "latency_s" in r]),
        "processing_s": distribution([r["processing_s"] for r in infos if "processing_s" in r]),
        "endpoint_displacement_m": math.hypot(valid[-1]["x"] - valid[0]["x"], valid[-1]["y"] - valid[0]["y"]) if valid else None,
        "endpoint_heading_change_rad": angle(valid[-1]["yaw"] - valid[0]["yaw"]) if valid else None}


def _relative(a, b):
    dx, dy = b["x"] - a["x"], b["y"] - a["y"]
    c, s = math.cos(a["yaw"]), math.sin(a["yaw"])
    return c * dx + s * dy, -s * dx + c * dy, angle(b["yaw"] - a["yaw"])


def reference_metrics(rows, reference, max_dt_ns=20_000_000):
    invalid_estimate = sum(not finite_pose(r) for r in rows)
    invalid_reference = sum(not finite_pose(r) for r in reference)
    counts = {"invalid_estimate_samples": invalid_estimate, "invalid_reference_samples": invalid_reference}
    rows = [r for r in rows if finite_pose(r)]
    reference = sorted((r for r in reference if finite_pose(r)), key=lambda r: r["stamp_ns"])
    stamps = [r["stamp_ns"] for r in reference]
    matches = []
    used = set()
    for row in sorted(rows, key=lambda r: r["stamp_ns"]):
        index = bisect.bisect_left(stamps, row["stamp_ns"])
        candidates = [i for i in (index - 1, index) if 0 <= i < len(stamps)]
        if not candidates:
            continue
        best = min(candidates, key=lambda i: abs(stamps[i] - row["stamp_ns"]))
        if best not in used and abs(stamps[best] - row["stamp_ns"]) <= max_dt_ns:
            matches.append((row, reference[best]))
            used.add(best)
    if not matches:
        return {**counts, "matched_samples": 0, "ate_position_rmse_m": None, "rpe_1s_translation_rmse_m": None}
    origin, ref_origin = matches[0]
    errors, rpe, heading = [], [], []
    aligned_yaw = ref_origin["yaw"] - origin["yaw"]
    c, s = math.cos(aligned_yaw), math.sin(aligned_yaw)
    for row, ref in matches:
        dx, dy = row["x"] - origin["x"], row["y"] - origin["y"]
        errors.append(math.hypot(c * dx - s * dy + ref_origin["x"] - ref["x"],
                                 s * dx + c * dy + ref_origin["y"] - ref["y"]))
        heading.append(angle(row["yaw"] + aligned_yaw - ref["yaw"]))
    est_stamps = [row["stamp_ns"] for row, _ in matches]
    for i, (row, ref) in enumerate(matches):
        j = bisect.bisect_left(est_stamps, row["stamp_ns"] + 1_000_000_000)
        if j >= len(matches) or abs(est_stamps[j] - row["stamp_ns"] - 1_000_000_000) > max_dt_ns:
            continue
        er, rr = _relative(row, matches[j][0]), _relative(ref, matches[j][1])
        rpe.append(math.hypot(er[0] - rr[0], er[1] - rr[1]))
    return {**counts, "matched_samples": len(matches), "alignment": "initial_pose_SE2_no_scale",
            "association_tolerance_ns": max_dt_ns, "ate_position_rmse_m": rmse(errors),
            "ate_heading_rmse_rad": rmse(heading), "rpe_1s_translation_rmse_m": rmse(rpe), "rpe_pairs": len(rpe)}


def read_csv(path):
    with Path(path).open(newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    for r in rows:
        for k in list(r):
            if k in {"stamp_ns", "inliers"}:
                r[k] = int(r[k])
            elif k == "lost":
                r[k] = r[k].lower() in {"true", "1"}
            else:
                r[k] = float(r[k])
    return rows


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--reference", type=Path, help="Independent time-synchronized base pose CSV: stamp_ns,x,y,yaw")
    parser.add_argument("--reference-source", help="Name of independent ground-truth system; wheel/RTABMap are not truth")
    parser.add_argument("--closed-loop", action="store_true", help="Assert physical trajectory returns to start")
    args = parser.parse_args(argv)
    try:
        if args.reference and not args.reference_source:
            raise ValueError("--reference-source must identify independent ground truth")
        rows = read_csv(args.run / "odometry.csv")
        infos = read_csv(args.run / "visual.csv")
        frames = read_csv(args.run / "frames.csv")
        manifest = json.loads((args.run / "manifest.json").read_text(encoding="utf-8"))
        result = summarize(rows, infos, [r["stamp_ns"] for r in frames],
                           min_inliers=manifest.get("metric_min_inliers", 10))
        result["manifest"] = manifest
        if args.closed_loop:
            result["closed_loop_endpoint_error_m"] = result["endpoint_displacement_m"]
        if args.reference:
            result["reference"] = reference_metrics(rows, read_csv(args.reference))
            result["reference_source"] = args.reference_source
        if (args.run / "resources.csv").exists():
            resources = read_csv(args.run / "resources.csv")
            result["resources"] = {"peak_tracked_rss_mb": max((r["rss_mb"] for r in resources), default=None),
                "peak_tracked_cpu_percent": max((r["cpu_percent"] for r in resources), default=None)}
        result["evaluation_status"] = "initialized" if result["initialized"] and result["odometry_samples"] else "no_visual_initialization"
        payload = json.dumps(result, indent=2, allow_nan=False) + "\n"
        with args.output.open("x", encoding="utf-8") as f:
            f.write(payload)
    except (OSError, ValueError, KeyError, TypeError) as exc:
        parser.exit(2, f"vins_metrics: {exc}\n")
    return 0 if result["evaluation_status"] == "initialized" else 1


if __name__ == "__main__":
    raise SystemExit(main())
