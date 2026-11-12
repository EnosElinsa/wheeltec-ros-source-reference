"""The same shared database is exclusive across independent launcher processes."""
import os
from pathlib import Path
import subprocess
import sys

import pytest

from wheeltec_vision_nav.startup import ResourceConflict


def test_shared_database_and_partition_world_leases_across_processes(tmp_path, monkeypatch):
    from wheeltec_vision_sim.leases import acquire_simulation_leases
    monkeypatch.setenv('WHEELTEC_SIM_LEASE_ROOT', str(tmp_path / 'shared' / 'leases'))
    monkeypatch.setenv('IGN_PARTITION', 'second-container')
    database = tmp_path / 'shared' / 'room.db'
    child_home = tmp_path / 'disposable-rootfs'
    child_home.mkdir()
    nav_root = Path(__file__).resolve().parents[2] / 'wheeltec_vision_nav'
    sim_root = Path(__file__).resolve().parents[1]
    env = dict(os.environ, HOME=str(child_home), USERPROFILE=str(child_home), IGN_PARTITION='first-container',
               PYTHONPATH=os.pathsep.join((str(nav_root), str(sim_root))))
    source = ('from wheeltec_vision_sim.leases import acquire_simulation_leases; import sys; '
              'lease=acquire_simulation_leases(sys.argv[1]); print("held",flush=True); sys.stdin.readline()')
    child = subprocess.Popen([sys.executable, '-c', source, str(database)], env=env,
                             stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        assert child.stdout.readline().strip() == 'held'
        # Different partition/container still cannot use the same shared DB.
        with pytest.raises(ResourceConflict, match='database:'):
            acquire_simulation_leases(database)
        with acquire_simulation_leases(tmp_path / 'shared' / 'other.db'):
            pass
        # Different databases in one transport partition still share world ownership.
        monkeypatch.setenv('IGN_PARTITION', 'first-container')
        with pytest.raises(ResourceConflict, match='gazebo:'):
            acquire_simulation_leases(tmp_path / 'shared' / 'other.db')
        child.stdin.write('exit\n')
        child.stdin.flush()
        child.wait(timeout=5)
        with acquire_simulation_leases(database):
            pass  # stale metadata must not retain ownership or require unlinking
    finally:
        if child.poll() is None:
            child.kill()
        child.communicate(timeout=5)
