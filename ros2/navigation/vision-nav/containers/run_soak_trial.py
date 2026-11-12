#!/usr/bin/env python3
"""Run a stationary load recording and close its isolated launch process group."""
import argparse
import math
from pathlib import Path

from run_sim_trial import run_session


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--duration', type=float, default=1800.)
    parser.add_argument('--estimator', choices=('baseline', 'stereo', 'vio'), default='baseline')
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error('duration must be positive and finite')
    root = args.output.resolve()
    root.mkdir(parents=True, exist_ok=False)
    manifest = root/'launch'/'manifest.json'
    launch = ['ros2', 'launch', 'wheeltec_vision_sim', 'sim.launch.py',
              f'output:={root / "launch"}', f'database_path:={root / "room.db"}',
              'mode:=mapping', f'estimator:={args.estimator}']
    recorder = ['python3', str(Path(__file__).with_name('record_soak.py')),
                '--output', str(root/'observations'), '--launch-manifest', str(manifest),
                '--duration', str(args.duration)]
    # Includes startup/warmup allowances; shutdown remains bounded by run_session.
    return run_session(root, launch, recorder, manifest, args.duration+210.)


if __name__ == '__main__':
    raise SystemExit(main())
