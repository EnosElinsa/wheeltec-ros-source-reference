"""Fault-oriented schema 2 evidence tests, without ROS or a long-running soak."""
import importlib.util
import json
from pathlib import Path
import sys

import pytest

spec = importlib.util.spec_from_file_location('soak', Path(__file__).parents[1]/'record_soak.py')
soak = importlib.util.module_from_spec(spec)
spec.loader.exec_module(soak)


def healthy(now):
    return dict(state='READY', ready=True,
                sources={name: dict(valid=True, age_s=.01) for name in soak.REQUIRED_SOURCES})


def feed(evidence, now, omit=()):
    stamp = int((100+now)*1e9)
    if 'clock' not in omit: evidence.note_clock(stamp, now)
    if 'health' not in omit: evidence.note_health(healthy(now), now)
    if 'safe' not in omit: evidence.note_command([0.]*6, now)
    if 'wheel' not in omit: evidence.note_wheel(stamp, [0.]*6, now)


def running():
    evidence = soak.SoakEvidence()
    feed(evidence, 0.)
    feed(evidence, .05)
    evidence.begin(.05)
    return evidence


@pytest.mark.parametrize('missing', ['health', 'wheel', 'safe', 'clock'])
def test_warmup_requires_every_fresh_stream(missing):
    evidence = soak.SoakEvidence()
    feed(evidence, 0., (missing,))
    feed(evidence, .05, (missing,))
    with pytest.raises(RuntimeError, match='fresh READY'):
        evidence.begin(.05)


def test_begin_retains_real_receipts_and_excludes_warmup_violations():
    evidence = soak.SoakEvidence()
    evidence.note_command([1., 0., 0., 0., 0., 0.], 0.)
    feed(evidence, .05); feed(evidence, .1)
    evidence.begin(.1)
    assert evidence.check(.11)
    assert evidence.safe_last == .1 and evidence.safe_count == 0
    assert not evidence.violations


@pytest.mark.parametrize('missing,reason', [
    ('health', 'health_receipt_stale'), ('wheel', 'wheel_receipt_stale'),
    ('safe', 'safe_command_receipt_stale'), ('clock', 'ros_clock_not_advancing')])
def test_gap_is_latched_even_when_stream_recovers_before_next_resource_sample(missing, reason):
    evidence = running()
    for now in (.15, .25, .35, .45): feed(evidence, now, (missing,))
    feed(evidence, .46)
    assert evidence.check(.46)
    assert evidence.violations[reason] > 0
    assert all(event['elapsed_wall_s'] < 1. for event in evidence.events)


def test_receipt_callback_detects_gap_even_if_main_loop_did_not_run():
    evidence = running()
    evidence.note_command([0.]*6, .46)
    assert evidence.violations['safe_command_receipt_stale'] == 1


@pytest.mark.parametrize('stream', ['safe', 'wheel'])
@pytest.mark.parametrize('axis', range(6))
def test_nan_on_every_command_or_wheel_axis_fails_immediately(stream, axis):
    evidence = running()
    values = [0.]*6; values[axis] = float('nan')
    if stream == 'safe': evidence.note_command(values, .1)
    else: evidence.note_wheel(int(100.1e9), values, .1)
    assert evidence.violations['nonzero_or_invalid_command' if stream == 'safe'
                               else 'nonstationary_or_invalid_wheel'] == 1


def test_fresh_health_receipts_cannot_refresh_stale_source_age():
    evidence = running()
    health = healthy(.1); health['sources']['cloud']['age_s'] = .31
    evidence.note_health(health, .1)
    assert evidence.violations['health_source_stale_or_invalid:cloud'] == 1


def test_health_snapshot_interval_does_not_prove_source_stopped_between_reports():
    evidence = running()
    health = healthy(.1)
    health['sources']['obstacles']['age_s'] = .08335
    evidence.note_health(health, .1)
    # An actual smoke had .08335 s source age plus .22268 s old health receipt.
    # That does not observe source updates between diagnostic publications.
    evidence.note_clock(int(100.32268e9), .32268)
    evidence.note_wheel(int(100.32268e9), [0.]*6, .32268)
    evidence.note_command([0.]*6, .32268)
    assert evidence.check(.32268)
    assert not evidence.violations


@pytest.mark.parametrize('invalid', ['age', 'valid'])
def test_new_ready_health_snapshot_cannot_hide_invalid_reported_source(invalid):
    evidence = running()
    health = healthy(.1)
    if invalid == 'age': health['sources']['depth']['age_s'] = .301
    else: health['sources']['depth']['valid'] = False
    evidence.note_health(health, .1)
    feed(evidence, .15)
    assert evidence.check(.15)
    assert evidence.violations['health_source_stale_or_invalid:depth'] == 1


def test_clock_messages_must_advance_and_wheel_acquisition_cannot_repeat_forever():
    evidence = running()
    for now in (.15, .25, .35, .45):
        evidence.note_clock(int(100.05e9), now)
        evidence.note_wheel(int(100.05e9), [0.]*6, now)
        evidence.note_health(healthy(now), now)
        evidence.note_command([0.]*6, now)
    assert evidence.violations['ros_clock_not_advancing']
    assert evidence.violations['wheel_receipt_stale']


def test_health_not_ready_between_resource_samples_is_latched():
    evidence = running()
    evidence.note_health(dict(state='STOPPED', ready=False), .1)
    feed(evidence, .15)
    assert evidence.violations['health_not_ready']


def test_summary_covers_tail_gap_and_cannot_pass_incomplete_or_exception_run(tmp_path):
    evidence = running()
    feed(evidence, .1)
    report = soak.save_summary(tmp_path, evidence, .4, .5, True, [{'rss_sum_bytes': 12}])
    assert report['schema'] == 2 and not report['passed']
    assert report['safe_coverage_gap_s']['maximum'] == pytest.approx(.4)
    assert report['violations']['safe_command_receipt_stale']
    empty = soak.SoakEvidence()
    report = soak.save_summary(tmp_path, empty, 1800., 1., False, [], 'RuntimeError: startup failed')
    assert report == json.loads((tmp_path/'summary.json').read_text())
    assert not report['passed'] and report['sample_count'] == 0
    assert report['error'] == 'RuntimeError: startup failed'


def test_complete_healthy_window_passes_without_counting_warmup(tmp_path):
    evidence = running()
    for now in (.1, .2, .3, .4, .5): feed(evidence, now)
    report = soak.save_summary(tmp_path, evidence, .45, .51, True, [{'rss_sum_bytes': 12}])
    assert report['passed'] and report['safe_commands'] == 5
    assert report['wheel_callbacks'] == report['health_callbacks'] == 5
    trace = json.loads((tmp_path/'safe-receipts.json').read_text())
    assert trace['receipt_elapsed_wall_s'] == pytest.approx([.05, .15, .25, .35, .45])
    assert trace['last_pre_window_receipt_elapsed_s'] == 0.


def test_main_persists_failed_summary_even_when_ros_import_fails(tmp_path, monkeypatch):
    monkeypatch.setitem(sys.modules, 'rclpy', None)
    output = tmp_path/'run'
    result = soak.main(['--output', str(output), '--launch-manifest', str(tmp_path/'missing.json')])
    report = json.loads((output/'summary.json').read_text())
    assert result == 2 and not report['completed'] and not report['passed']
    assert report['schema'] == 2 and report['error'].startswith('ModuleNotFoundError:')


def test_stale_health_and_stale_acquisition_cannot_start_warmup():
    evidence = running()
    evidence.started = None
    # Fresh receipts do not authorize a cloud whose acquisition is already old.
    health = healthy(.2); health['sources']['cloud']['age_s'] = float('nan')
    evidence.note_health(health, .2)
    with pytest.raises(RuntimeError): evidence.begin(.2)
    feed(evidence, .2)
    evidence.note_wheel(int(99e9), [0.]*6, .2)
    with pytest.raises(RuntimeError): evidence.begin(.2)
