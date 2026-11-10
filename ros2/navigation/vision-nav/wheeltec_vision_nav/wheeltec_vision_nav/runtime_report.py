"""Summarize live-run evidence; reuse the VINS evaluator's metric definitions."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

from wheeltec_vision_nav.vins_metrics import distribution, read_csv, summarize, reference_metrics


def digest_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def summarize_topics(rows, control_period_s=.1):
    groups = {}
    for row in rows:
        groups.setdefault(row["topic"], []).append(row)
    result = {}
    for topic, samples in groups.items():
        stamps = [float(r["received_s"]) for r in samples]
        intervals = [b - a for a, b in zip(stamps, stamps[1:])]
        age = [float(r["age_s"]) for r in samples]
        result[topic] = {"messages": len(samples), "age_s": distribution(age),
                         "interval_s": distribution(intervals),
                         "timestamp_regressions": sum(int(b["stamp_ns"]) <= int(a["stamp_ns"])
                             for a, b in zip(samples, samples[1:])
                             if int(a["stamp_ns"]) > 0 and int(b["stamp_ns"]) > 0)}
        if topic in ("/cmd_vel", "/cmd_vel_nav"):
            result[topic]["control_interval_overrun_fraction"] = (
                sum(v > control_period_s * 1.5 for v in intervals) / len(intervals) if intervals else None)
    return result


def summarize_health(rows):
    durations, reasons = {}, {}
    for a, b in zip(rows, rows[1:]):
        duration = max(0.0, float(b["received_s"]) - float(a["received_s"]))
        durations[a["state"]] = durations.get(a["state"], 0.0) + duration
    for row in rows:
        reasons[row["reason"]] = reasons.get(row["reason"], 0) + 1
    episodes = []
    previous = None
    for row in rows:
        if row["state"] == "STOPPED" and previous != "STOPPED":
            episodes.append({"received_s": float(row["received_s"]), "reason": row["reason"]})
        previous = row["state"]
    return {"state_duration_s": durations, "reason_samples": reasons, "stops": episodes}


def optional_csv(directory, filename):
    path = directory / filename
    if not path.is_file():
        return []
    if filename in ("odometry.csv", "visual.csv", "frames.csv"):
        return read_csv(path)
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def summarize_memory(rows):
    samples = [(float(row["received_s"]), float(row["rss_mb"])) for row in rows
               if row.get("received_s") not in (None, "") and row.get("rss_mb") not in (None, "")]
    if len(samples) < 2:
        return {"change_mb": None, "slope_mb_per_min": None}
    origin = samples[0][0]
    times = [t - origin for t, _ in samples]
    mean_t = sum(times) / len(times)
    mean_rss = sum(rss for _, rss in samples) / len(samples)
    denominator = sum((t - mean_t)**2 for t in times)
    slope = sum((t - mean_t)*(rss - mean_rss) for t, (_, rss) in zip(times, samples))
    return {"change_mb": samples[-1][1] - samples[0][1],
            "slope_mb_per_min": 60*slope/denominator if denominator > 0 else None}


def summarize_corrections(rows):
    linear, angular = [], []
    for a, b in zip(rows, rows[1:]):
        linear.append(math.hypot(float(b["x"])-float(a["x"]), float(b["y"])-float(a["y"])))
        delta = float(b["yaw"])-float(a["yaw"])
        angular.append(abs(math.atan2(math.sin(delta), math.cos(delta))))
    return {"translation_step_m": distribution(linear), "yaw_step_rad": distribution(angular)}


def build_report(directory, reference=None, reference_source=None):
    directory = Path(directory)
    manifest_path = directory / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8")) if manifest_path.exists() else {}
    rows = optional_csv(directory, "odometry.csv")
    visual = optional_csv(directory, "visual.csv")
    frames = optional_csv(directory, "frames.csv")
    result = {"schema_version": 1, "manifest": manifest,
              "verification": {"local_tests": "see implementation report",
                               "ros_integration": "not_verified_by_this_report",
                               "physical_stop_distance": "requires_measured_acceptance",
                               "navigation_success": "requires_operator_trial_annotations"},
              "odometry": summarize(rows, visual, [int(r["stamp_ns"]) for r in frames],
                                    min_inliers=manifest.get("metric_min_inliers", 10)),
              "topics": summarize_topics(optional_csv(directory, "topics.csv")),
              "health": summarize_health(optional_csv(directory, "health.csv"))}
    resources = optional_csv(directory, "resources.csv")
    for field in ("cpu_percent", "rss_mb", "temperature_c", "gpu_load_percent"):
        values = [float(r[field]) for r in resources if r.get(field) not in (None, "")]
        result.setdefault("resources", {})[field] = distribution(values)
    result["resources"]["memory_trend"] = summarize_memory(resources)
    processes = {}
    for row in optional_csv(directory, "processes.csv"):
        processes.setdefault(f"{row['pid']}:{row['name']}", []).append(row)
    result["processes"] = {name: {
        "rss_mb": distribution([float(r["rss_mb"]) for r in samples]),
        "cpu_percent": distribution([float(r["cpu_percent"]) for r in samples]),
        "memory_trend": summarize_memory(samples)} for name, samples in processes.items()}
    result["map_corrections"] = summarize_corrections(optional_csv(directory, "map_corrections.csv"))
    provenance = directory / "provenance.jsonl"
    deltas = []
    if provenance.exists():
        for line in provenance.read_text(encoding="utf-8").splitlines():
            row = json.loads(line)
            if row.get("delta_ns") is not None:
                deltas.append(abs(int(row["delta_ns"])) / 1e9)
    result["pairing_original_delta_s"] = distribution(deltas)
    trials = directory / "trials.json"
    if trials.exists():
        from wheeltec_vision_nav.acceptance import summarize_trials
        result["operator_trials"] = summarize_trials(json.loads(trials.read_text(encoding="utf-8")))
    if reference is not None:
        if not reference_source or reference_source.lower() in ("wheel", "wheel_odom", "odometry"):
            raise ValueError("reference-source must identify independent synchronized ground truth")
        result["reference"] = {"source": reference_source,
                               **reference_metrics(rows, read_csv(reference))}
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--reference", type=Path)
    parser.add_argument("--reference-source")
    args = parser.parse_args(argv)
    result = build_report(args.run, args.reference, args.reference_source)
    payload = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    if args.output:
        with args.output.open("x", encoding="utf-8") as stream:
            stream.write(payload)
    else:
        print(payload, end="")


if __name__ == "__main__":
    main()
