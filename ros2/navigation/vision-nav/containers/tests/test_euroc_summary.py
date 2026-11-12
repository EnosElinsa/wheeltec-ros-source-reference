import importlib.util
import json
from pathlib import Path

import pytest


spec = importlib.util.spec_from_file_location('euroc_summary', Path(__file__).parents[1] / 'summarize_euroc.py')
summary = importlib.util.module_from_spec(spec)
spec.loader.exec_module(summary)


def test_summary_retains_failed_missing_runs_and_measures_original_age(tmp_path):
    (tmp_path / 'suite.json').write_text(json.dumps(dict(runs=[
        dict(run='good', exit_code=0, outcome='scored_partial', metrics=dict(coverage=.8)),
        dict(run='failed', exit_code=1, outcome='runtime_failed', metrics=None)])))
    good = tmp_path / 'good'
    good.mkdir()
    (good / 'trajectory.csv').write_text('data_age_ns\n10000000\n20000000\n-1000000\nnan\n')
    (good / 'playback.csv').write_text('kind,schedule_lateness_ns\nimu,900000000\nstereo,2000000\n')
    report = summary.summarize([tmp_path])
    measured, failed = report['runs']
    assert measured['metrics']['coverage'] == .8
    assert measured['odom_data_age_ms']['p50'] == 10
    assert measured['odom_data_age_ms']['negative'] == 1
    assert measured['odom_data_age_ms']['nonfinite'] == 1
    assert measured['image_dispatch_lateness_ms']['maximum'] == 2
    assert len(measured['artifacts']['trajectory.csv']) == 64
    assert failed['exit_code'] == 1 and failed['metrics'] is None and failed['runtime'] is None
    assert failed['odom_data_age_ms'] is None
    json.dumps(report, allow_nan=False)


def test_summary_cannot_read_outside_suite(tmp_path):
    (tmp_path / 'suite.json').write_text(json.dumps(dict(runs=[
        dict(run='../outside', exit_code=0, outcome='scored_complete')])))
    with pytest.raises(ValueError, match='directory name'):
        summary.summarize([tmp_path])
