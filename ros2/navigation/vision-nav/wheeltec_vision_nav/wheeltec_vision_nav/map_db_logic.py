"""Read an RTAB-Map database and report how consistent the map graph is. No ROS imports."""

from __future__ import annotations

import math
import sqlite3
import struct
import zlib
from dataclasses import dataclass, field

# RTAB-Map Link.type
LINK_NEIGHBOR = 0
LINK_GLOBAL_CLOSURE = 1
LINK_LOCAL_SPACE = 2
LINK_NAMES = {
    0: "neighbor",
    1: "global_closure",
    2: "proximity",
    3: "local_time",
    4: "user",
    5: "virtual",
    6: "neighbor_merged",
}

Pose2 = tuple[float, float, float]


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def compose(a: Pose2, b: Pose2) -> Pose2:
    c, s = math.cos(a[2]), math.sin(a[2])
    return a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1], wrap(a[2] + b[2])


def inverse(a: Pose2) -> Pose2:
    c, s = math.cos(a[2]), math.sin(a[2])
    return -c * a[0] - s * a[1], s * a[0] - c * a[1], wrap(-a[2])


def between(a: Pose2, b: Pose2) -> Pose2:
    return compose(inverse(a), b)


def pose_from_transform(blob: bytes) -> Pose2:
    """RTAB-Map stores a Transform as 12 float32 (3x4 row-major)."""
    v = struct.unpack("12f", blob[:48])
    return v[3], v[7], math.atan2(v[4], v[0])


def uncompress_matrix(blob: bytes) -> tuple[int, int, int, bytes]:
    """RTAB-Map compressData2: zlib payload followed by rows, cols, type (int32)."""
    rows, cols, kind = struct.unpack("3i", blob[-12:])
    return rows, cols, kind, zlib.decompress(blob[:-12])


def information_diagonal(blob: bytes) -> list[float]:
    try:
        _, _, _, data = uncompress_matrix(blob)
    except (zlib.error, struct.error):
        data = blob
    count = len(data) // 8
    values = struct.unpack("%dd" % count, data[: count * 8])
    if count < 36:
        return list(values)
    return [values[i * 6 + i] for i in range(6)]


@dataclass
class Link:
    source: int
    target: int
    kind: int
    transform: Pose2
    info_diag: list[float] = field(default_factory=list)


@dataclass
class MapGraph:
    odom: dict[int, Pose2]
    optimized: dict[int, Pose2]
    links: list[Link]


def load_graph(path: str) -> MapGraph:
    db = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
    try:
        cur = db.cursor()
        odom = {
            nid: pose_from_transform(blob)
            for nid, blob in cur.execute("select id, pose from Node")
            if blob is not None and len(blob) >= 48
        }
        links = [
            Link(src, dst, kind, pose_from_transform(tb), information_diagonal(ib) if ib else [])
            for src, dst, kind, tb, ib in cur.execute(
                "select from_id, to_id, type, transform, information_matrix from Link"
            )
            if tb is not None and len(tb) >= 48
        ]
        optimized: dict[int, Pose2] = {}
        row = cur.execute("select opt_ids, opt_poses from Admin").fetchone()
        if row and row[0] and row[1]:
            _, _, _, ids_raw = uncompress_matrix(row[0])
            _, _, _, poses_raw = uncompress_matrix(row[1])
            ids = struct.unpack("%di" % (len(ids_raw) // 4), ids_raw)
            poses = struct.unpack("%df" % (len(poses_raw) // 4), poses_raw)
            for i, nid in enumerate(ids):
                v = poses[i * 12 : (i + 1) * 12]
                if len(v) == 12:
                    optimized[nid] = (v[3], v[7], math.atan2(v[4], v[0]))
    finally:
        db.close()
    return MapGraph(odom=odom, optimized=optimized, links=links)


@dataclass
class ClosureCheck:
    link: Link
    predicted: Pose2
    dxy: float
    dyaw_deg: float


def check_closures(
    graph: MapGraph,
    max_span: int = 100,
    max_dxy: float = 0.5,
    max_dyaw_deg: float = 20.0,
) -> tuple[int, list[ClosureCheck]]:
    """Compare each loop closure with the motion odometry predicts between its two nodes.

    Only closures spanning at most ``max_span`` nodes are checked, since odometry
    drifts over longer spans. Returns (checked count, inconsistent closures).
    """
    checked = 0
    bad: list[ClosureCheck] = []
    for link in graph.links:
        if link.kind not in (LINK_GLOBAL_CLOSURE, LINK_LOCAL_SPACE):
            continue
        if link.source not in graph.odom or link.target not in graph.odom:
            continue
        if abs(link.source - link.target) > max_span:
            continue
        checked += 1
        predicted = between(graph.odom[link.source], graph.odom[link.target])
        dxy = math.hypot(link.transform[0] - predicted[0], link.transform[1] - predicted[1])
        dyaw = abs(math.degrees(wrap(link.transform[2] - predicted[2])))
        if dxy > max_dxy or dyaw > max_dyaw_deg:
            bad.append(ClosureCheck(link, predicted, dxy, dyaw))
    return checked, bad


def correction_jumps(graph: MapGraph) -> list[tuple[int, float, float]]:
    """Change of the map correction (optimized * odom^-1) between consecutive nodes.

    Returns (node id, translation change m, heading change deg).
    """
    ids = sorted(set(graph.odom) & set(graph.optimized))
    out: list[tuple[int, float, float]] = []
    previous: Pose2 | None = None
    for nid in ids:
        correction = compose(graph.optimized[nid], inverse(graph.odom[nid]))
        if previous is not None:
            out.append(
                (
                    nid,
                    math.hypot(correction[0] - previous[0], correction[1] - previous[1]),
                    abs(math.degrees(wrap(correction[2] - previous[2]))),
                )
            )
        previous = correction
    return out


def median(values: list[float]) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    mid = len(ordered) // 2
    return ordered[mid] if len(ordered) % 2 else 0.5 * (ordered[mid - 1] + ordered[mid])


def summarize(graph: MapGraph) -> dict:
    counts: dict[str, int] = {}
    for link in graph.links:
        name = LINK_NAMES.get(link.kind, str(link.kind))
        counts[name] = counts.get(name, 0) + 1
    neighbor_yaw_info = [
        link.info_diag[5] for link in graph.links if link.kind == LINK_NEIGHBOR and len(link.info_diag) >= 6
    ]
    checked, bad = check_closures(graph)
    jumps = correction_jumps(graph)
    return {
        "nodes": len(graph.odom),
        "optimized_nodes": len(graph.optimized),
        "links": counts,
        "neighbor_yaw_information_median": median(neighbor_yaw_info),
        "closures_checked": checked,
        "closures_inconsistent": len(bad),
        "inconsistent": [
            {
                "from": c.link.source,
                "to": c.link.target,
                "type": LINK_NAMES.get(c.link.kind, str(c.link.kind)),
                "dxy_m": round(c.dxy, 3),
                "dyaw_deg": round(c.dyaw_deg, 1),
            }
            for c in bad[:20]
        ],
        "correction_jump_max_m": round(max((j[1] for j in jumps), default=0.0), 3),
        "correction_jump_max_deg": round(max((j[2] for j in jumps), default=0.0), 1),
    }


def verdict(summary: dict) -> list[str]:
    """Human-readable problems. Empty list means the graph looks consistent."""
    problems = []
    yaw_info = summary.get("neighbor_yaw_information_median")
    if yaw_info is not None and yaw_info < 10.0:
        problems.append(
            f"odometry links carry almost no heading constraint (yaw information {yaw_info:g})"
        )
    checked = summary.get("closures_checked", 0)
    if checked and summary["closures_inconsistent"] > max(2, 0.05 * checked):
        problems.append(
            f"{summary['closures_inconsistent']} of {checked} loop closures disagree with odometry"
        )
    if summary.get("correction_jump_max_deg", 0.0) > 20.0 or summary.get("correction_jump_max_m", 0.0) > 0.5:
        problems.append(
            "map correction jumps by "
            f"{summary['correction_jump_max_m']} m / {summary['correction_jump_max_deg']} deg between nodes"
        )
    return problems
