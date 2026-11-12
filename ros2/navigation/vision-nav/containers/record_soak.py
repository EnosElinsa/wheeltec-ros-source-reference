#!/usr/bin/env python3
"""Schema 2 stationary load evidence; never send vehicle commands.

Schema 1 recordings lack the callback coverage required here. They cannot be
upgraded to schema 2 by filling fields from assumptions or offline estimates.
"""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import signal
from statistics import median
import time


RECORDER_SOURCE = Path(__file__).read_bytes()
RECEIPT_TIMEOUT = .3
SOURCE_TIMEOUTS = {'vo': .5, 'visual': .5, 'map_tf': 5., 'localization': 5.}
REQUIRED_SOURCES = {'vo', 'visual', 'odom', 'tf', 'tf_authority', 'ownership',
                    'depth', 'cloud', 'obstacles'}


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def stats(values):
    if not values:
        return None
    ordered = sorted(values)
    def percentile(fraction):
        position = (len(ordered)-1)*fraction
        lower = int(position)
        upper = min(lower+1, len(ordered)-1)
        return ordered[lower]+(ordered[upper]-ordered[lower])*(position-lower)
    return dict(count=len(ordered), p95=percentile(.95), p99=percentile(.99), maximum=ordered[-1])


class SoakEvidence:
    """Watch callbacks and loop gaps; 1 Hz resource samples are not the verdict."""
    def __init__(self, event_sink=None):
        self.health = {}
        self.health_received = self.safe_last = self.wheel_received = None
        self.safe_valid = self.wheel_valid = False
        self.wheel_stamp = None
        self.wheel_initial_age = 0.
        self.ros_ns = self.clock_received = self.clock_advanced = None
        self.clock_advances = 0
        self.started = self.start_ros_ns = None
        self.safe_at_start = None
        self.violations = Counter()
        self.events = []
        self.current_failures = set()
        self.safe_times = []
        self.health_count = self.wheel_count = self.safe_count = 0
        self.health_states = Counter()
        self.event_sink = event_sink

    def failures(self, now):
        failed = set()
        def fresh(received):
            return received is not None and 0 <= now-received < RECEIPT_TIMEOUT
        if not fresh(self.safe_last):
            failed.add('safe_command_receipt_stale')
        if not self.safe_valid:
            failed.add('nonzero_or_invalid_command')
        if not fresh(self.wheel_received):
            failed.add('wheel_receipt_stale')
        if not self.wheel_valid:
            failed.add('nonstationary_or_invalid_wheel')
        if (self.wheel_stamp is None or self.ros_ns is None
                or not -.1 <= (self.ros_ns-self.wheel_stamp)*1e-9 < RECEIPT_TIMEOUT
                or self.wheel_received is None
                or now-self.wheel_received+self.wheel_initial_age >= RECEIPT_TIMEOUT):
            failed.add('wheel_acquisition_stale')
        if not fresh(self.health_received):
            failed.add('health_receipt_stale')
        if self.health.get('state') != 'READY' or self.health.get('ready') is not True:
            failed.add('health_not_ready')
        sources = self.health.get('sources', {})
        if not isinstance(sources, dict):
            sources = {}
        for name in REQUIRED_SOURCES | sources.keys():
            item = sources.get(name, {})
            age = item.get('age_s') if isinstance(item, dict) else None
            # This age describes the source observation at health publication.
            # Diagnostics run at 4-5 Hz while sources can update at 20 Hz. The
            # recorder cannot extrapolate that old observation into a claim that
            # no newer source arrived. Only health receipt expiry is continuous.
            if (not isinstance(item, dict) or item.get('valid') is not True or not finite(age)
                    or not 0 <= age < SOURCE_TIMEOUTS.get(name, RECEIPT_TIMEOUT)):
                failed.add('health_source_stale_or_invalid:'+name)
        if not fresh(self.clock_received) or not fresh(self.clock_advanced) or not self.clock_advances:
            failed.add('ros_clock_not_advancing')
        return failed

    def check(self, now):
        failed = self.failures(now)
        if self.started is not None:
            for reason in sorted(failed-self.current_failures):
                self.flag(reason, now)
            self.current_failures = failed
        return not failed

    def flag(self, reason, now):
        if self.started is None:
            return
        self.violations[reason] += 1
        event = dict(elapsed_wall_s=now-self.started, reason=reason)
        self.events.append(event)
        if self.event_sink:
            self.event_sink(event)

    def begin(self, now):
        if self.started is not None or not self.check(now):
            raise RuntimeError('soak requires fresh READY health, sources, zero command, stationary wheel and advancing clock')
        # Keep real receipts; reset counters so the first sample neither invents
        # missing commands nor counts warmup activity as measurement activity.
        self.started, self.start_ros_ns = now, self.ros_ns
        self.safe_at_start = self.safe_last
        self.violations.clear()
        self.events.clear()
        self.current_failures.clear()
        self.safe_times.clear()
        self.health_count = self.wheel_count = self.safe_count = 0
        self.health_states.clear()

    def note_health(self, payload, now):
        self.check(now)  # Detect a gap before this receipt can hide it.
        try:
            health = json.loads(payload) if isinstance(payload, str) else payload
            self.health = health if isinstance(health, dict) else {}
        except (ValueError, TypeError):
            self.health = {}
        self.health_received = now
        if self.started is not None:
            self.health_count += 1
            self.health_states[str(self.health.get('state', 'unknown'))] += 1
        self.check(now)

    def note_command(self, values, now):
        self.check(now)
        self.safe_valid = len(values) == 6 and all(finite(v) and abs(v) <= 1e-5 for v in values)
        self.safe_last = now
        if self.started is not None:
            self.safe_count += 1
            self.safe_times.append(now)
        self.check(now)

    def note_wheel(self, stamp, values, now):
        self.check(now)
        if self.started is not None:
            self.wheel_count += 1
        self.wheel_valid = (len(values) == 6 and all(finite(v) for v in values)
            and all(abs(v) <= .02 for v in values[:3]) and all(abs(v) <= .05 for v in values[3:]))
        if not isinstance(stamp, int) or stamp <= 0 or self.ros_ns is None:
            self.wheel_valid = False
        elif self.wheel_stamp is None or stamp > self.wheel_stamp:
            self.wheel_stamp, self.wheel_received = stamp, now
            self.wheel_initial_age = max(0., (self.ros_ns-stamp)*1e-9)
        elif stamp < self.wheel_stamp:
            self.wheel_valid = False
        # Duplicate acquisition cannot refresh its wall receipt deadline.
        self.check(now)

    def note_clock(self, ros_ns, now):
        self.check(now)
        if not isinstance(ros_ns, int) or ros_ns <= 0:
            self.flag('invalid_ros_clock', now)
            return
        if self.ros_ns is not None and ros_ns < self.ros_ns:
            self.flag('ros_clock_reversed', now)
            self.clock_advances = 0
            self.clock_advanced = None
        elif self.ros_ns is not None and ros_ns > self.ros_ns:
            self.clock_advances += 1
            self.clock_advanced = now
        self.ros_ns, self.clock_received = ros_ns, now
        self.check(now)

    def snapshot(self, now):
        self.check(now)
        def age(received):
            return None if received is None else now-received
        return dict(elapsed_wall_s=now-self.started, ros_ns=self.ros_ns,
            safe_receipt_age_s=age(self.safe_last), wheel_receipt_age_s=age(self.wheel_received),
            wheel_stamp_ns=self.wheel_stamp, health_receipt_age_s=age(self.health_received),
            clock_receipt_age_s=age(self.clock_received), clock_progress_age_s=age(self.clock_advanced),
            health=self.health, current_failures=sorted(self.current_failures))


def save_summary(output, evidence, requested, ended, completed, samples, error=None):
    """Save startup/callback/resource exceptions too; never infer completion."""
    if evidence.started is not None:
        evidence.check(ended)
    duration = 0. if evidence.started is None else ended-evidence.started
    boundaries = ([evidence.started, *evidence.safe_times, ended] if evidence.started is not None else [])
    coverage_gaps = [b-a for a, b in zip(boundaries, boundaries[1:])]
    intervals = [b-a for a, b in zip(evidence.safe_times, evidence.safe_times[1:])]
    path = output/'samples.jsonl'
    trace = dict(window_start_monotonic_s=evidence.started, window_duration_s=duration,
        last_pre_window_receipt_elapsed_s=(evidence.safe_at_start-evidence.started
                                          if evidence.started is not None else None),
        receipt_elapsed_wall_s=[value-evidence.started for value in evidence.safe_times])
    trace_bytes = (json.dumps(trace, allow_nan=False)+'\n').encode()
    (output/'safe-receipts.json').write_bytes(trace_bytes)
    report = dict(schema=2,
        kind='stationary sensor/estimation/rendering load; no navigation control load or vehicle acceptance',
        requested_wall_s=requested, duration_wall_s=duration, completed=completed,
        safe_commands=evidence.safe_count, wheel_callbacks=evidence.wheel_count,
        health_callbacks=evidence.health_count, health_callback_states=dict(evidence.health_states),
        safe_interval_s=stats(intervals), safe_coverage_gap_s=stats(coverage_gaps),
        safe_receipts_sha256=hashlib.sha256(trace_bytes).hexdigest(),
        interval_note='Interarrival statistics use measurement-window callbacks; coverage also includes window head and tail. Warmup receipt is retained for expiry checks.',
        safe_window_head_s=(evidence.safe_times[0]-evidence.started if evidence.safe_times
                            else duration if evidence.started is not None else None),
        safe_window_tail_s=(ended-max(evidence.started, evidence.safe_last)
                            if evidence.started is not None and evidence.safe_last is not None else None),
        safe_intervals_over_100ms=sum(value > .1 for value in intervals),
        violations=dict(evidence.violations), violation_events=evidence.events,
        sample_count=len(samples), monitoring='every callback and main loop; gap check precedes receipt updates',
        thresholds_s=dict(receipt=RECEIPT_TIMEOUT, health_sources_default=RECEIPT_TIMEOUT,
                          health_source_exceptions=SOURCE_TIMEOUTS),
        health_source_observation='Health callback snapshots only: reported valid/age are checked without extrapolation; no continuous direct source coverage is claimed.',
        measured_ros_seconds=((evidence.ros_ns-evidence.start_ros_ns)*1e-9
                              if evidence.started is not None else None),
        first_window_rss_sum_median_bytes=median([s['rss_sum_bytes'] for s in samples[:300]]) if samples else None,
        last_window_rss_sum_median_bytes=median([s['rss_sum_bytes'] for s in samples[-300:]]) if samples else None,
        memory_note='RSS sums include shared pages; windows contain at most 300 observations, not guaranteed seconds.',
        sample_sha256=hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None,
        recorder_sha256=hashlib.sha256(RECORDER_SOURCE).hexdigest(),
        schema1_note='Legacy runs lack callback receipt/source/clock coverage; do not backfill schema 2 evidence.',
        limits='Health/command/wheel/clock receipt gaps are monitored continuously. Other source validity/age are observed only in health snapshots; this is not continuous direct source validation, acquisition-to-actuator latency, or a moving/navigation soak.',
        error=error,
        passed=bool(completed and duration >= requested and samples and not error and not evidence.violations))
    (output/'summary.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--launch-manifest', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=1800.)
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error('duration must be positive and finite')
    args.output.mkdir(parents=True, exist_ok=False)
    evidence = SoakEvidence()
    samples = []
    node = rclpy = None
    error = None
    completed = False
    def interrupted(signum, frame):
        raise InterruptedError('recorder interrupted by signal '+str(signum))
    previous_term = signal.signal(signal.SIGTERM, interrupted)
    try:
        (args.output/'recorder.py').write_bytes(RECORDER_SOURCE)
        import rclpy
        from rclpy.parameter import Parameter
        from rclpy.qos import qos_profile_sensor_data
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import String
        from wheeltec_vision_sim.mission_logic import resource_snapshot
        manifest_bytes = args.launch_manifest.read_bytes()
        (args.output/'launch-manifest.json').write_bytes(manifest_bytes)
        root_pid = json.loads(manifest_bytes)['resource_root_pid']
        rclpy.init()
        node = rclpy.create_node('stationary_soak_recorder', parameter_overrides=[Parameter('use_sim_time', value=True)])
        def axes(msg):
            return [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z]
        def stamp(value):
            return int(value.sec)*1_000_000_000+int(value.nanosec)
        subscriptions = [
            node.create_subscription(String, '/vision/health/status',
                lambda m: evidence.note_health(m.data, time.monotonic()), 2),
            node.create_subscription(Twist, '/cmd_vel_safe',
                lambda m: evidence.note_command(axes(m), time.monotonic()), 10),
            node.create_subscription(Odometry, '/wheel/odom',
                lambda m: evidence.note_wheel(stamp(m.header.stamp), axes(m.twist.twist), time.monotonic()), qos_profile_sensor_data),
            node.create_subscription(Clock, '/clock',
                lambda m: evidence.note_clock(stamp(m.clock), time.monotonic()), qos_profile_sensor_data)]
        with (args.output/'events.jsonl').open('x') as events, (args.output/'samples.jsonl').open('x') as stream:
            def event_sink(event):
                events.write(json.dumps(event, allow_nan=False)+'\n'); events.flush()
            evidence.event_sink = event_sink
            deadline, stable = time.monotonic()+150., None
            while True:
                rclpy.spin_once(node, timeout_sec=.01)
                now = time.monotonic()
                if evidence.check(now):
                    if stable is None: stable = now
                    if now-stable >= .5: break
                else:
                    stable = None
                if now >= deadline:
                    raise RuntimeError('fresh READY, command, wheel and advancing clock warmup deadline exceeded')
            evidence.begin(now)
            next_sample = now
            while time.monotonic()-evidence.started < args.duration:
                rclpy.spin_once(node, timeout_sec=.01)
                now = time.monotonic()
                evidence.check(now)
                if now < next_sample:
                    continue
                next_sample = now+1.
                temperatures = {}
                for file in Path('/sys/class/thermal').glob('thermal_zone*/temp'):
                    try:
                        value = float(file.read_text())/1000.
                        if math.isfinite(value): temperatures[file.parent.name] = value
                    except (OSError, ValueError):
                        pass
                resources = resource_snapshot(root_pid)
                if not any(p['pid'] == root_pid for p in resources['processes']):
                    evidence.flag('launch_root_missing', now)
                sample = evidence.snapshot(time.monotonic())
                sample.update(resources=resources, rss_sum_bytes=sum(p['rss_bytes'] for p in resources['processes']),
                              temperature_c=temperatures)
                stream.write(json.dumps(sample, allow_nan=False)+'\n'); stream.flush()
                samples.append(sample)
            evidence.check(time.monotonic())
            completed = True
    except (Exception, KeyboardInterrupt) as failure:
        error = type(failure).__name__+': '+str(failure)
    finally:
        ended = time.monotonic()
        # Streams may be closed after an exception; final checks stay in summary.
        evidence.event_sink = None
        try:
            if node is not None: node.destroy_node()
            if rclpy is not None and rclpy.ok(): rclpy.shutdown()
        except Exception as failure:
            error = error or type(failure).__name__+': '+str(failure)
        signal.signal(signal.SIGTERM, previous_term)
        report = save_summary(args.output, evidence, args.duration, ended, completed, samples, error)
    return 0 if report['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
