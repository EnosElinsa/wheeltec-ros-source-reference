"""Session success requires both observed mission success and clean launch teardown."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

spec = importlib.util.spec_from_file_location('sim_trial', Path(__file__).parents[1] / 'run_sim_trial.py')
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)


@pytest.mark.skipif(os.name == 'nt', reason='POSIX process-group signal teardown')
@pytest.mark.parametrize('behavior,code,signal_path', [
    ('clean', 0, ['SIGINT']), ('shutdown_error', 7, ['SIGINT']),
    ('term_clean', 0, ['SIGINT', 'SIGTERM']),
    ('ignore_int', -15, ['SIGINT', 'SIGTERM']),
    ('ignore_all', -9, ['SIGINT', 'SIGTERM', 'SIGKILL']),
])
def test_successful_mission_requires_clean_launch_shutdown(tmp_path, behavior, code, signal_path):
    source = """
import json, pathlib, signal, sys, time
behavior, path = sys.argv[1:]
def interrupt(*_):
    raise SystemExit(7 if behavior == 'shutdown_error' else 0)
signal.signal(signal.SIGINT, signal.SIG_IGN if behavior in ('ignore_int','ignore_all','term_clean') else interrupt)
if behavior == 'term_clean': signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
if behavior == 'ignore_all': signal.signal(signal.SIGTERM, signal.SIG_IGN)
pathlib.Path(path).write_text('{}')
while True: time.sleep(.02)
"""
    manifest = tmp_path / 'manifest.json'
    result = runner.run_session(tmp_path,
        [sys.executable, '-c', source, behavior, str(manifest)],
        [sys.executable, '-c', 'raise SystemExit(0)'], manifest,
        timeout=5., shutdown_timeouts=(.2, .2))
    evidence = json.loads((tmp_path / 'session.json').read_text())
    assert evidence['mission_exit_code'] == 0
    assert evidence['launch_exit_code'] == code
    assert evidence['launch_shutdown']['signals'] == signal_path
    assert evidence['launch_shutdown']['escalated'] == (len(signal_path) > 1)
    assert evidence['unexpected_launch_exit'] is False
    assert result == (0 if behavior == 'clean' else 2)


@pytest.mark.skipif(os.name == 'nt', reason='POSIX process-group signal teardown')
def test_mission_success_does_not_hide_launch_death_at_loop_boundary(tmp_path):
    manifest, exited = tmp_path / 'manifest.json', tmp_path / 'launch-exiting'
    launch = "import pathlib,sys,time; pathlib.Path(sys.argv[1]).write_text('{}'); time.sleep(.05); pathlib.Path(sys.argv[2]).touch(); raise SystemExit(8)"
    mission = "import pathlib,sys,time; p=pathlib.Path(sys.argv[1]);\nwhile not p.exists(): time.sleep(.01)"
    result = runner.run_session(tmp_path,
        [sys.executable, '-c', launch, str(manifest), str(exited)],
        [sys.executable, '-c', mission, str(exited)], manifest, timeout=5.)
    evidence = json.loads((tmp_path / 'session.json').read_text())
    assert evidence['mission_exit_code'] == 0
    assert evidence['launch_exit_code'] == 8
    assert evidence['unexpected_launch_exit'] is True
    assert evidence['launch_shutdown']['signals'] == []
    assert result == 2


@pytest.mark.skipif(os.name == 'nt', reason='POSIX runtime wrapper')
def test_runtime_wrapper_exposes_shared_lease_directory_and_preserves_affinity(tmp_path):
    docker = tmp_path / 'docker'
    docker.write_text(f'#!{sys.executable}\nimport json,os,sys\nopen(os.environ["CAPTURE"],"w").write(json.dumps(sys.argv[1:]))\n')
    docker.chmod(0o755)
    capture, root = tmp_path / 'args.json', tmp_path / 'shared-work'
    env = dict(os.environ, PATH=str(tmp_path) + os.pathsep + os.environ['PATH'],
               CAPTURE=str(capture), WHEELTEC_SIM_ROOT=str(root), WHEELTEC_SIM_CPUSET='0-7',
               WHEELTEC_SIM_OMP_THREADS='3', WHEELTEC_SIM_BLAS_THREADS='2', WHEELTEC_SIM_OPENCV_THREADS='4')
    subprocess.run(['bash', str(Path(__file__).parents[1] / 'run.sh'), 'true'], env=env, check=True)
    arguments = json.loads(capture.read_text())
    assert (root / 'leases').is_dir()
    assert f'{root}:/work' in arguments
    assert 'WHEELTEC_SIM_LEASE_ROOT=/work/leases' in arguments
    assert arguments[arguments.index('--cpuset-cpus')+1] == '0-7'
    assert all(item in arguments for item in ('OMP_NUM_THREADS=3', 'OPENBLAS_NUM_THREADS=2', 'OPENCV_FOR_THREADS_NUM=4'))
