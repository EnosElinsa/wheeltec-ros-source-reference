#!/usr/bin/env python3
"""Summarize saved simulator evidence; does not change scenario verdicts."""
import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path

import numpy as np


def distribution(values):
    values = [v for v in values if v is not None and math.isfinite(v)]
    if not values:
        return None
    return dict(count=len(values), median=float(np.median(values)),
        p95=float(np.percentile(values, 95)), p99=float(np.percentile(values, 99)),
        minimum=min(values), maximum=max(values))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('session')
    args = parser.parse_args()
    root = Path(args.session)
    sample_path = root / 'mission' / 'samples.jsonl'
    rows = [json.loads(line) for line in sample_path.read_text().splitlines()]
    if not rows:
        raise ValueError('empty observation file')
    active = [s for s in rows if s['phase'] != 'startup' and not s.get('pause_expected')]
    report = dict(schema=1, sample_sha256=hashlib.sha256(sample_path.read_bytes()).hexdigest(),
        samples=len(rows), active_samples=len(active), wall_duration_s=rows[-1]['wall']-rows[0]['wall'],
        sim_duration_s=(rows[-1]['ros_ns']-rows[0]['ros_ns'])*1e-9,
        states=dict(Counter(s['health'].get('state', 'unknown') for s in rows)),
        sim_per_wall=distribution(s.get('sim_per_wall') for s in active),
        safe_command_receipt_age_s=distribution(s.get('safe_age_s') for s in active),
        sensor_acquisition_age_s={name: distribution(
            (s['ros_ns']-s['sensors'][name]['stamp_ns'])*1e-9 for s in active if name in s['sensors'])
            for name in ('vision', 'depth', 'imu')},
        process_tree_rss_bytes=distribution(sum(p['rss_bytes'] for p in s.get('resources', {}).get('processes', [])) for s in active),
        memory_note='Sum of RSS can double-count shared pages; CPU data are raw per-process cumulative ticks in samples.',
        collision_events=max(s.get('collision_events', 0) for s in rows),
        interpretation='Descriptive recorder statistics; input/control deadlines and physical/route acceptance remain the scenario results. No 30-minute qualification implied.')
    results = root / 'mission' / 'results.json'
    if results.is_file():
        report['scenario_results'] = json.loads(results.read_text())
    (root / 'summary.json').write_text(json.dumps(report, indent=2) + '\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(11, 4))
    gt = [s['ground_truth']['xyz'] for s in rows if s.get('ground_truth')]
    if gt:
        points = np.array(gt)
        axes[0].plot(points[:, 0], points[:, 1], label='Gazebo independent truth')
        axes[0].scatter(points[0, 0], points[0, 1], marker='o', label='start')
        axes[0].set(xlabel='world x [m]', ylabel='world y [m]', title='Observed physical trajectory', aspect='equal')
        axes[0].legend()
    t = [s['wall']-rows[0]['wall'] for s in rows]
    axes[1].plot(t, [s['safe_v'] for s in rows], label='safe command')
    axes[1].plot(t, [s['wheel_v'] for s in rows], label='measured wheel')
    axes[1].set(xlabel='wall time [s]', ylabel='forward speed [m/s]', title='Command and measured response')
    axes[1].legend()
    fig.tight_layout()
    fig.savefig(root / 'trajectory.png', dpi=160)
    plt.close(fig)
    print(root / 'summary.json')


if __name__ == '__main__':
    main()
