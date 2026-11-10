"""Real subprocess ownership and SQLite preservation at the startup boundary."""
import os
from pathlib import Path
import sqlite3
import subprocess
import sys

import pytest

from wheeltec_vision_nav.startup import ReadinessDeadline, ResourceConflict, ResourceLeases, database_resource, prepare_database

ROOT = Path(__file__).resolve().parents[1]


def child_env():
    return dict(os.environ, PYTHONPATH=str(ROOT))


def test_another_process_owns_resources_until_exit_and_stale_metadata_is_harmless(tmp_path):
    code = ("from wheeltec_vision_nav.startup import ResourceLeases; import sys; "
            "lease=ResourceLeases(sys.argv[1]).acquire(['camera:d435c']); "
            "print('held',flush=True); sys.stdin.readline()")
    child = subprocess.Popen([sys.executable, "-c", code, str(tmp_path)], stdin=subprocess.PIPE,
                             stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env=child_env())
    try:
        assert child.stdout.readline().strip() == "held"
        with pytest.raises(ResourceConflict, match="pid"):
            ResourceLeases(tmp_path).acquire(["camera:d435c"])
        child.stdin.write("exit\n")
        child.stdin.flush()
        child.wait(timeout=10)
        with ResourceLeases(tmp_path).acquire(["camera:d435c"]):
            pass
    finally:
        if child.poll() is None:
            child.kill()
        child.communicate(timeout=10)


def test_failed_multi_resource_acquisition_releases_only_its_own_resources(tmp_path):
    with ResourceLeases(tmp_path).acquire(["z:camera"]):
        with pytest.raises(ResourceConflict):
            ResourceLeases(tmp_path).acquire(["a:database", "z:camera"])
        with ResourceLeases(tmp_path).acquire(["a:database"]):
            pass
        with pytest.raises(ResourceConflict):
            ResourceLeases(tmp_path).acquire(["z:camera"])


def test_database_defaults_preserve_content_and_navigation_requires_saved_map(tmp_path):
    path = tmp_path / "map.db"
    with pytest.raises(FileNotFoundError, match="Saved"):
        prepare_database(path, localization=True)
    with sqlite3.connect(path) as connection:
        connection.execute("create table retained(value)")
        connection.execute("insert into retained values (42)")
    original = path.read_bytes()
    assert prepare_database(path) is None
    assert path.read_bytes() == original
    assert database_resource(path) == database_resource(path.parent / "." / path.name)
    with pytest.raises(ValueError, match="mapping"):
        prepare_database(path, localization=True, wipe=True)


def test_wipe_backup_preserves_committed_wal_before_exact_database_deletion(tmp_path):
    path = tmp_path / "map.db"
    code = ("import sqlite3,sys,os; db=sqlite3.connect(sys.argv[1]); "
            "db.execute('PRAGMA journal_mode=WAL'); db.execute('PRAGMA wal_autocheckpoint=0'); "
            "db.execute('create table retained(value)'); db.execute('insert into retained values (99)'); "
            "db.commit(); os._exit(0)")
    subprocess.run([sys.executable, "-c", code, str(path)], check=True, timeout=10)
    assert Path(str(path) + "-wal").is_file()
    unrelated = tmp_path / "unrelated.db"
    unrelated.write_bytes(b"untouched")
    backup = prepare_database(path, wipe=True)
    assert not path.exists() and not Path(str(path) + "-wal").exists()
    with sqlite3.connect(backup) as db:
        assert db.execute("select value from retained").fetchone() == (99,)
        assert db.execute("pragma integrity_check").fetchone() == ("ok",)
    assert unrelated.read_bytes() == b"untouched"


def test_wipe_rejects_active_sqlite_reader_without_deletion(tmp_path):
    path = tmp_path / "map.db"
    reader = sqlite3.connect(path)
    try:
        reader.execute("create table retained(value)")
        reader.commit()
        reader.execute("begin")
        reader.execute("select * from retained").fetchall()
        with pytest.raises(ResourceConflict, match="in use"):
            prepare_database(path, wipe=True)
        assert path.is_file()
    finally:
        reader.close()


def test_monotonic_readiness_deadline_only_accepts_current_true_state():
    deadline = ReadinessDeadline(30., now=100.)
    deadline.observe(False)
    assert deadline.result(129.99) is None
    assert deadline.result(130.) == 2
    deadline = ReadinessDeadline(30., now=0.)
    deadline.observe(True)
    assert deadline.result(1.) == 0
    deadline.observe(False)
    assert deadline.result(1.) is None
