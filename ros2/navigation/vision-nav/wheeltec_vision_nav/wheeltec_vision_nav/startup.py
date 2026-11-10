"""Process-owned resource leases, database preservation and startup deadlines.

Locks are advisory OS locks, retained by the launch process until shutdown. A
stale metadata file never counts as an owner: the operating system releases the
lock when a process dies. Never unlink lock files (another process may hold one).
"""
from __future__ import annotations

import hashlib
import json
import math
import os
from pathlib import Path
import sqlite3
import time
import uuid


class ResourceConflict(RuntimeError):
    pass


class ResourceLeases:
    def __init__(self, root=None):
        self.root = Path(root or Path.home() / ".ros/wheeltec_vision_nav/leases")
        self._files = []

    def acquire(self, resources):
        self.root.mkdir(parents=True, exist_ok=True)
        try:
            for resource in sorted(set(resources)):
                path = self.root / (hashlib.sha256(resource.encode()).hexdigest() + ".lock")
                stream = path.open("a+b")
                # Windows byte-range locks require a byte to exist.
                if path.stat().st_size == 0:
                    stream.write(b" ")
                    stream.flush()
                try:
                    if os.name == "nt":
                        import msvcrt
                        stream.seek(0)
                        msvcrt.locking(stream.fileno(), msvcrt.LK_NBLCK, 1)
                    else:
                        import fcntl
                        fcntl.flock(stream.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
                except OSError as exc:
                    # Windows also denies reads of the locked byte.
                    stream.seek(1)
                    owner = stream.read().decode("utf-8", errors="replace").strip()
                    stream.close()
                    raise ResourceConflict(f"Resource {resource!r} is owned by {owner or 'another launch'}") from exc
                self._files.append(stream)
                stream.seek(1)
                stream.truncate()
                stream.write(json.dumps({"pid": os.getpid(), "resource": resource,
                                         "started": time.time()}).encode())
                stream.flush()
        except BaseException:
            self.release()
            raise
        return self

    def release(self, *args, **kwargs):
        while self._files:
            self._files.pop().close()
        return []

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.release()


def database_resource(path):
    return "database:" + os.path.normcase(str(Path(path).expanduser().resolve()))


def prepare_database(path, *, localization=False, wipe=False):
    """Check the saved map or explicitly back up then clear a mapping database.

    SQLite's backup API includes committed WAL content in a consistent standalone
    database. An exclusive connection rejects concurrent readers/writers, including
    WAL readers. The caller must also retain the database resource lease.
    """
    path = Path(path).expanduser().resolve()
    if localization:
        if wipe:
            raise ValueError("wipe_db is only supported for mapping")
        if not path.is_file():
            raise FileNotFoundError(f"Saved RTAB-Map database does not exist: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.exists() or not wipe:
        return None
    backup = path.with_name(path.name + ".backup-" + time.strftime("%Y%m%d-%H%M%S") + "-" + uuid.uuid4().hex[:8])
    source = sqlite3.connect(str(path), timeout=0.0)
    try:
        source.execute("PRAGMA locking_mode=EXCLUSIVE")
        source.execute("BEGIN EXCLUSIVE")
        source.commit()
        with sqlite3.connect(str(backup)) as destination:
            source.backup(destination)
        with sqlite3.connect(f"file:{backup.as_posix()}?mode=ro", uri=True) as check:
            if check.execute("PRAGMA integrity_check").fetchone()[0] != "ok":
                raise RuntimeError(f"Database backup failed integrity check: {backup}")
    except sqlite3.OperationalError as exc:
        raise ResourceConflict(f"Database is in use or cannot be backed up: {path}: {exc}") from exc
    finally:
        source.close()
    # Only remove the exact database and its SQLite sidecars after backup succeeds.
    for candidate in (path, Path(str(path) + "-wal"), Path(str(path) + "-shm")):
        candidate.unlink(missing_ok=True)
    return str(backup)


class ReadinessDeadline:
    """A monotonic startup deadline, unaffected by paused or changed ROS time."""
    def __init__(self, timeout_s=30.0, now=None):
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("readiness timeout must be finite and positive")
        self.deadline = (time.monotonic() if now is None else now) + timeout_s
        self.ready = False

    def observe(self, ready):
        self.ready = bool(ready)

    def result(self, now=None):
        if (time.monotonic() if now is None else now) >= self.deadline:
            return 2
        if self.ready:
            return 0
        return None
