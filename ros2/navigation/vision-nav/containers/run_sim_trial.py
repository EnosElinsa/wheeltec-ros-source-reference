#!/usr/bin/env python3
"""Launch and record one simulation session and three trials in one container."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time


def stop(process, timeouts=(20., 5.)):
    outcome = dict(exited_before_teardown=process.poll() is not None,
                   signals=[], escalated=False, exit_code=process.returncode)
    if outcome['exited_before_teardown']:
        return outcome
    for name, timeout in zip(('SIGINT', 'SIGTERM', 'SIGKILL'), (*timeouts, None)):
        try:
            os.killpg(process.pid, getattr(signal, name))
            outcome['signals'].append(name)
        except ProcessLookupError:
            # A launch may die between the last boundary check and the signal.
            if not outcome['signals']:
                outcome['exited_before_teardown'] = True
        try:
            process.wait(timeout=timeout)
            break
        except subprocess.TimeoutExpired:
            continue
    outcome.update(exit_code=process.returncode,
                   escalated=any(s != 'SIGINT' for s in outcome['signals']))
    return outcome


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--database', required=True)
    parser.add_argument('--mode', choices=['mapping', 'localization'], default='mapping')
    parser.add_argument('--estimator', choices=['baseline', 'stereo', 'vio'], default='baseline')
    parser.add_argument('--scenario', default='all')
    parser.add_argument('--timeout', type=float, default=1200)
    args = parser.parse_args()
    out = Path(args.output).resolve()
    out.mkdir(parents=True, exist_ok=False)
    launch_args = ['ros2', 'launch', 'wheeltec_vision_sim', 'sim.launch.py',
        f'output:={out / "launch"}', f'database_path:={Path(args.database).resolve()}',
        f'mode:={args.mode}', f'estimator:={args.estimator}']
    mission_args = ['ros2', 'run', 'wheeltec_vision_sim', 'mission_runner', 'run',
        '--output', str(out / 'mission'), '--mode', args.mode, '--estimator', args.estimator,
        '--scenario', args.scenario, '--repeats', '3',
        '--launch-manifest', str(out / 'launch' / 'manifest.json'),
        '--ros-args', '-p', 'use_sim_time:=true']
    return run_session(out, launch_args, mission_args, out / 'launch' / 'manifest.json', args.timeout)


def run_session(out, launch_args, mission_args, manifest, timeout, shutdown_timeouts=(20., 5.)):
    evidence = dict(launch_command=launch_args, mission_command=mission_args,
        unexpected_launch_exit=False,
        started_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
        environment={k: os.environ.get(k) for k in ('ROS_DOMAIN_ID', 'IGN_PARTITION',
            'WHEELTEC_SIM_LEASE_ROOT', 'OMP_NUM_THREADS', 'OPENCV_FOR_THREADS_NUM', 'OPENBLAS_NUM_THREADS')},
        cpu_affinity=sorted(os.sched_getaffinity(0)) if hasattr(os, 'sched_getaffinity') else None)
    launch = mission = None
    started = time.monotonic()
    try:
        with (out / 'launch.log').open('x') as log, (out / 'mission.log').open('x') as mlog:
            launch = subprocess.Popen(launch_args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            while not manifest.is_file():
                if launch.poll() is not None:
                    evidence['unexpected_launch_exit'] = True
                    raise RuntimeError('launch exited before manifest; inspect launch.log')
                if time.monotonic() - started > 30:
                    raise TimeoutError('launch did not prepare its manifest in 30 seconds')
                time.sleep(.1)
            mission = subprocess.Popen(mission_args, stdout=mlog, stderr=subprocess.STDOUT, start_new_session=True)
            while mission.poll() is None:
                if launch.poll() is not None:
                    evidence['unexpected_launch_exit'] = True
                    raise RuntimeError('launch exited during experiment; inspect launch.log')
                if time.monotonic() - started > timeout:
                    raise TimeoutError('experiment wall-clock deadline exceeded')
                time.sleep(.2)
            evidence['mission_exit_code'] = mission.returncode
            # Close the poll boundary: the mission can exit in the same interval
            # as a crashing launch. Mission success alone is insufficient.
            if launch.poll() is not None:
                evidence['unexpected_launch_exit'] = True
                raise RuntimeError('launch exited before intentional teardown; inspect launch.log')
    except (Exception, KeyboardInterrupt) as error:
        evidence['error'] = f'{type(error).__name__}: {error}'
    finally:
        if mission is not None:
            evidence['mission_shutdown'] = stop(mission, shutdown_timeouts)
            evidence['mission_exit_code'] = mission.returncode
        if launch is not None:
            evidence['launch_shutdown'] = stop(launch, shutdown_timeouts)
            evidence['unexpected_launch_exit'] |= evidence['launch_shutdown']['exited_before_teardown']
            evidence['launch_exit_code'] = launch.returncode
        evidence['session_success'] = (evidence.get('mission_exit_code') == 0 and 'error' not in evidence
            and not evidence['unexpected_launch_exit'] and evidence.get('launch_exit_code') == 0
            and not evidence.get('launch_shutdown', {}).get('escalated', True))
        evidence['elapsed_wall_s'] = time.monotonic() - started
        (out / 'session.json').write_text(json.dumps(evidence, indent=2) + '\n')
    return 0 if evidence['session_success'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
