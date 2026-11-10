import csv
import json

import pytest

from wheeltec_vision_nav.runtime_report import build_report, summarize_topics


def write_csv(path, header, rows):
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        writer.writerows(rows)


def test_control_overruns_and_stale_stamp_measure_actual_input_intervals():
    rows = [{"topic": "/cmd_vel", "received_s": t, "stamp_ns": 0, "age_s": 0}
            for t in (0, .1, .2, .5)]
    result = summarize_topics(rows)["/cmd_vel"]
    assert result["control_interval_overrun_fraction"] == pytest.approx(1/3)
    assert result["interval_s"]["max"] == .3


def test_runtime_report_preserves_unknown_acceptance_and_original_pair_delta(tmp_path):
    write_csv(tmp_path / "odometry.csv", ["stamp_ns", "x", "y", "yaw", "latency_s"],
              [(1_000_000_000, 0, 0, 0, .03), (2_000_000_000, 1, 0, 0, .04)])
    write_csv(tmp_path / "visual.csv", ["stamp_ns", "lost", "inliers", "processing_s"],
              [(1_000_000_000, 0, 40, .02), (2_000_000_000, 0, 40, .03)])
    write_csv(tmp_path / "frames.csv", ["stamp_ns"], [(1_000_000_000,), (2_000_000_000,)])
    write_csv(tmp_path / "health.csv", ["received_s", "state", "reason", "output_vx", "output_wz"],
              [(1, "READY", "healthy", .2, 0), (2, "STOPPED", "depth_stale", 0, 0)])
    write_csv(tmp_path / "resources.csv", ["rss_mb", "cpu_percent", "temperature_c", "gpu_load_percent"],
              [(100, 50, "", ""), (110, 40, 45, "")])
    (tmp_path / "provenance.jsonl").write_text(json.dumps({"delta_ns": 33_333_333}) + "\n")
    report = build_report(tmp_path)
    assert report["odometry"]["visual_lost_fraction"] == 0
    assert report["pairing_original_delta_s"]["mean"] == pytest.approx(.033333333)
    assert report["health"]["stops"][0]["reason"] == "depth_stale"
    assert report["resources"]["temperature_c"]["count"] == 1
    assert report["resources"]["gpu_load_percent"]["mean"] is None
    assert "requires" in report["verification"]["physical_stop_distance"]
    assert "reference" not in report


def test_no_wheel_odometry_can_be_silently_used_as_truth(tmp_path):
    with pytest.raises(ValueError, match="independent"):
        build_report(tmp_path, tmp_path / "wheel.csv", "wheel")


def test_resource_trend_and_map_yaw_wrap_are_measured(tmp_path):
    from wheeltec_vision_nav.runtime_report import summarize_memory, summarize_corrections
    import math
    rows = [{"received_s": t, "rss_mb": rss} for t, rss in ((10, 100), (40, 110), (70, 120))]
    assert summarize_memory(rows) == {"change_mb": 20, "slope_mb_per_min": 20}
    corrections = [{"x": 0, "y": 0, "yaw": math.pi-.01},
                   {"x": .3, "y": .4, "yaw": -math.pi+.01}]
    result = summarize_corrections(corrections)
    assert result["translation_step_m"]["max"] == .5
    assert result["yaw_step_rad"]["max"] == pytest.approx(.02)


def test_runtime_report_uses_recorded_quality_threshold(tmp_path):
    (tmp_path / "manifest.json").write_text(json.dumps({"metric_min_inliers": 20}))
    write_csv(tmp_path / "frames.csv", ["stamp_ns"], [(1_000_000_000,), (2_000_000_000,)])
    write_csv(tmp_path / "visual.csv", ["stamp_ns", "lost", "inliers", "processing_s"],
              [(1_000_000_000, 0, 15, .02), (2_000_000_000, 0, 30, .03)])
    result = build_report(tmp_path)["odometry"]
    assert result["visual_min_inliers"] == 20
    assert result["visual_lost_fraction"] == .5
