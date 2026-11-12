#!/usr/bin/env python3
"""Summarize completed EuRoC suites without turning scores into acceptance."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path

import numpy as np


def distribution(values):
    values = list(values)
    finite = [v for v in values if math.isfinite(v)]
    return dict(count=len(values), nonfinite=len(values)-len(finite),
        negative=sum(v < 0 for v in finite),
        p50=float(np.percentile(finite, 50)) if finite else None,
        p95=float(np.percentile(finite, 95)) if finite else None,
        p99=float(np.percentile(finite, 99)) if finite else None,
        maximum=max(finite) if finite else None)


def csv_values(path, field, scale=1., kind=None):
    with path.open(newline='') as stream:
        for row in csv.DictReader(stream):
            if kind is None or row.get('kind') == kind:
                if row.get(field, ''):
                    yield float(row[field]) * scale


def summarize(groups):
    rows = []
    for group in groups:
        suite = json.loads((group / 'suite.json').read_text())
        for result in suite['runs']:
            name = result['run']
            if Path(name).name != name or name in ('.', '..'):
                raise ValueError('run must be a directory name')
            root = group / name
            row = dict(group=group.name, run=name, exit_code=result['exit_code'],
                outcome=result['outcome'], metrics=result.get('metrics'), artifacts={})
            for filename in ('manifest.json', 'runtime.json', 'metrics.json',
                             'trajectory.csv', 'visual_status.csv', 'playback.csv'):
                path = root / filename
                if path.is_file():
                    row['artifacts'][filename] = hashlib.sha256(path.read_bytes()).hexdigest()
            manifest = root / 'manifest.json'
            if manifest.is_file():
                value = json.loads(manifest.read_text())
                row['data_sha256'] = value['data']['sha256']
            for filename, key, field, scale, kind in (
                ('trajectory.csv', 'odom_data_age_ms', 'data_age_ns', 1e-6, None),
                ('visual_status.csv', 'estimation_ms', 'time_estimation_s', 1e3, None),
                ('playback.csv', 'image_dispatch_lateness_ms', 'schedule_lateness_ns', 1e-6, 'stereo')):
                path = root / filename
                row[key] = distribution(csv_values(path, field, scale, kind)) if path.is_file() else None
            runtime = root / 'runtime.json'
            row['runtime'] = json.loads(runtime.read_text()) if runtime.is_file() else None
            rows.append(row)
    return dict(schema=1, runs=rows,
        meaning='Measured execution, coverage, error and timing evidence; no accuracy or driving acceptance implied. Missing/failed runs are retained.')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('groups', nargs='+', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    result = summarize(args.groups)
    with args.output.open('x') as stream:
        json.dump(result, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(args.output)


if __name__ == '__main__':
    main()
