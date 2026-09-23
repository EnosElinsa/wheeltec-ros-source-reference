import math
import sqlite3
import struct
import zlib

from wheeltec_vision_nav.map_db_logic import load_graph, summarize, verdict


def _transform(x: float, y: float, yaw: float) -> bytes:
    c, s = math.cos(yaw), math.sin(yaw)
    return struct.pack("12f", c, -s, 0.0, x, s, c, 0.0, y, 0.0, 0.0, 1.0, 0.0)


def _info(diag: list[float]) -> bytes:
    values = [0.0] * 36
    for i, v in enumerate(diag):
        values[i * 6 + i] = v
    return zlib.compress(struct.pack("36d", *values)) + struct.pack("3i", 6, 6, 6)


def _compressed(fmt: str, values: list, rows: int, cols: int, kind: int) -> bytes:
    return zlib.compress(struct.pack(fmt, *values)) + struct.pack("3i", rows, cols, kind)


def _database(path, yaw_info: float, false_closure: bool, optimized_twist: float = 0.0):
    db = sqlite3.connect(path)
    db.execute("create table Node (id integer, pose blob, stamp real)")
    db.execute("create table Link (from_id integer, to_id integer, type integer, information_matrix blob, transform blob)")
    db.execute("create table Admin (opt_ids blob, opt_poses blob)")
    poses = {i: (0.1 * i, 0.0, 0.0) for i in range(1, 41)}
    for nid, pose in poses.items():
        db.execute("insert into Node values (?, ?, ?)", (nid, _transform(*pose), float(nid)))
    for nid in range(1, 40):
        db.execute("insert into Link values (?, ?, 0, ?, ?)",
                   (nid, nid + 1, _info([1e3, 1e3, 1e-6, 1e-6, 1e-6, yaw_info]), _transform(0.1, 0.0, 0.0)))
    # A correct closure: node 10 -> node 30 is 2 m straight ahead.
    db.execute("insert into Link values (10, 30, 1, ?, ?)", (_info([500] * 6), _transform(2.0, 0.0, 0.0)))
    if false_closure:
        # A closure registered on the projector speckle: "no motion" over 1.8 m.
        db.execute("insert into Link values (12, 30, 1, ?, ?)", (_info([500] * 6), _transform(0.0, 0.0, 0.0)))
        db.execute("insert into Link values (13, 31, 1, ?, ?)", (_info([500] * 6), _transform(0.0, 0.0, 0.0)))
        db.execute("insert into Link values (14, 32, 1, ?, ?)", (_info([500] * 6), _transform(0.0, 0.0, 0.0)))
    ids = sorted(poses)
    flat = []
    for nid in ids:
        x, y, yaw = poses[nid]
        yaw += optimized_twist if nid >= 20 else 0.0
        c, s = math.cos(yaw), math.sin(yaw)
        flat += [c, -s, 0.0, x, s, c, 0.0, y, 0.0, 0.0, 1.0, 0.0]
    db.execute(
        "insert into Admin values (?, ?)",
        (
            _compressed("%di" % len(ids), ids, 1, len(ids), 4),
            _compressed("%df" % len(flat), flat, 1, len(flat), 5),
        ),
    )
    db.commit()
    db.close()


def test_consistent_graph_passes(tmp_path):
    path = str(tmp_path / "good.db")
    _database(path, yaw_info=2000.0, false_closure=False)
    summary = summarize(load_graph(path))
    assert summary["nodes"] == 40
    assert summary["links"] == {"neighbor": 39, "global_closure": 1}
    assert summary["closures_checked"] == 1
    assert summary["closures_inconsistent"] == 0
    assert summary["correction_jump_max_deg"] < 1.0
    assert verdict(summary) == []


def test_speckle_closures_and_free_heading_are_reported(tmp_path):
    path = str(tmp_path / "bad.db")
    _database(path, yaw_info=1e-3, false_closure=True, optimized_twist=math.radians(60))
    summary = summarize(load_graph(path))
    assert summary["closures_inconsistent"] == 3
    assert summary["inconsistent"][0]["dxy_m"] > 1.5
    assert summary["neighbor_yaw_information_median"] < 1.0
    assert summary["correction_jump_max_deg"] > 50.0
    problems = verdict(summary)
    assert len(problems) == 3
