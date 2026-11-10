import math
import pytest
from wheeltec_vision_nav.vins_metrics import summarize, reference_metrics, VisualHealth


def test_distributions_include_p99_for_empty_singleton_and_finite_series():
    from wheeltec_vision_nav.vins_metrics import distribution
    assert distribution([float("nan"), float("inf")]) == {
        "count": 0, "mean": None, "p95": None, "p99": None, "max": None}
    assert distribution([3.]) == {"count": 1, "mean": 3., "p95": 3., "p99": 3., "max": 3.}
    result = distribution(list(range(101)))
    assert result["p95"] == 95 and result["p99"] == 99


def test_visual_summary_records_effective_threshold_without_changing_default():
    infos = [{"stamp_ns": 1, "lost": False, "inliers": 15}]
    assert summarize([], infos, [1])["visual_valid_frames"] == 1
    strict = summarize([], infos, [1], min_inliers=20)
    assert strict["visual_valid_frames"] == 0 and strict["visual_min_inliers"] == 20


def test_imu_or_ekf_output_does_not_count_as_successful_visual_tracking():
    rows = [{"stamp_ns": s, "x": 0., "y": 0., "yaw": 0.} for s in (1, 2, 3, 4)]
    infos = [{"stamp_ns": 1, "lost": False, "inliers": 50},
             {"stamp_ns": 2, "lost": True, "inliers": 50}]
    result = summarize(rows, infos, [1, 2, 3, 4])
    assert result["visual_valid_frames"] == 1
    assert result["visual_lost_fraction"] == .75
    assert result["odometry_samples"] == 4


def test_recovery_and_closed_loop_endpoints_are_reported_separately():
    stamps = [1_000_000_000, 2_000_000_000, 3_000_000_000, 4_000_000_000]
    rows = [{"stamp_ns": s, "x": x, "y": 0., "yaw": 0.} for s, x in zip(stamps, (0., 1., 2., .1))]
    infos = [{"stamp_ns": s, "lost": i in (1, 2), "inliers": 30} for i, s in enumerate(stamps)]
    result = summarize(rows, infos, stamps)
    assert result["recovery_seconds"] == [2.]
    assert result["endpoint_displacement_m"] == pytest.approx(.1)
    assert "ate_position_rmse_m" not in result


def test_initial_pose_alignment_does_not_hide_scale_error():
    ref = [{"stamp_ns": i * 1_000_000_000, "x": float(i), "y": 0., "yaw": 0.} for i in (1, 2, 3)]
    est = [{"stamp_ns": i * 1_000_000_000, "x": 2.*i + 10., "y": 5., "yaw": 0.} for i in (1, 2, 3)]
    result = reference_metrics(est, ref)
    assert result["ate_position_rmse_m"] == pytest.approx(math.sqrt(5./3.))
    assert result["rpe_1s_translation_rmse_m"] == pytest.approx(1.)


def test_repeated_info_or_continued_odometry_cannot_refresh_health():
    h = VisualHealth(timeout_s=.5)
    h.note_visual(100, False, 30, now=0.)
    assert h.status(.1) == "tracking"
    h.note_visual(100, False, 30, now=.6)
    assert h.status(.6) == "stale"
    h.note_visual(200, True, 0, now=.7)
    assert h.status(.7) == "lost"


def test_reference_association_requires_close_timestamps():
    assert reference_metrics([{"stamp_ns": 1, "x": 0., "y": 0., "yaw": 0.}],
                             [{"stamp_ns": 1_000_000_000, "x": 0., "y": 0., "yaw": 0.}])["matched_samples"] == 0


def test_invalid_reference_and_estimate_samples_are_counted_without_nan_output():
    import json
    rows = [{"stamp_ns": s * 1_000_000_000, "x": float(s), "y": 0., "yaw": 0.} for s in range(4)]
    truth = [dict(r) for r in rows]
    rows[0]["yaw"] = float("nan")
    truth[1]["x"] = float("inf")
    result = reference_metrics(rows, truth)
    assert result["matched_samples"] == 2
    assert result["invalid_estimate_samples"] == 1
    assert result["invalid_reference_samples"] == 1
    assert result["ate_position_rmse_m"] == 0.
    json.dumps(result, allow_nan=False)


def test_summarize_latency_processing_and_never_initialized_tail():
    rows = [{"stamp_ns": s, "x": 0., "y": 0., "yaw": 0., "latency_s": l}
            for s, l in ((1, .1), (1_000_000_001, .2))]
    infos = [{"stamp_ns": 1, "lost": True, "inliers": 0, "processing_s": .04}]
    result = summarize(rows, infos, [1, 1_000_000_001])
    assert result["latency_s"]["max"] == .2
    assert result["processing_s"]["mean"] == .04
    assert result["unrecovered_tail_s"] == 1.
    assert result["max_heading_step_rad"] == 0.


@pytest.mark.parametrize("initialized,exit_code", [(True, 0), (False, 1)])
def test_metrics_cli_writes_reproducible_report_and_reports_initialization(tmp_path, initialized, exit_code):
    import json
    from wheeltec_vision_nav.vins_metrics import main
    run = tmp_path / "run"
    run.mkdir()
    (run / "manifest.json").write_text(json.dumps({"mode": "stereo", "bag_uri": "fixed-bag"}))
    (run / "odometry.csv").write_text("stamp_ns,x,y,yaw,latency_s\n1000000000,0,0,0,.1\n2000000000,.1,0,0,.2\n")
    (run / "visual.csv").write_text(f"stamp_ns,lost,inliers,processing_s\n1000000000,{not initialized},30,.04\n")
    (run / "frames.csv").write_text("stamp_ns\n1000000000\n2000000000\n")
    (run / "resources.csv").write_text("wall_s,rss_mb,cpu_percent\n1,10,120\n")
    output = tmp_path / "metrics.json"
    argv = ["--run", str(run), "--output", str(output), "--closed-loop"]
    assert main(argv) == exit_code
    report = json.loads(output.read_text())
    assert report["initialized"] is initialized
    assert report["manifest"]["bag_uri"] == "fixed-bag"
    assert report["resources"]["peak_tracked_cpu_percent"] == 120.
    assert report["closed_loop_endpoint_error_m"] == pytest.approx(.1)
    assert report["latency_s"]["max"] == .2
    with pytest.raises(SystemExit) as exc:
        main(argv)
    assert exc.value.code == 2
    assert json.loads(output.read_text()) == report
