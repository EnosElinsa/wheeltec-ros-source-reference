"""Pair emitter-off infrared with the nearest emitter-on depth frame."""

from __future__ import annotations

import bisect
import json
import math
from dataclasses import dataclass
from typing import Iterable

EMITTER_OFF = 0
EMITTER_ON = 1

_EMITTER_KEYS = (
    "frame_emitter_mode",
    "Frame Emitter Mode",
    "emitter_mode",
    "FRAME_EMITTER_MODE",
)


@dataclass(frozen=True)
class FrameRef:
    stamp: float
    emitter: int | None
    payload: object = None
    info: object = None
    stamp_ns: int | None = None
    source: str = "unknown"


def parse_emitter_mode(metadata_json: str | None) -> int | None:
    if not metadata_json:
        return None
    try:
        data = json.loads(metadata_json)
    except (json.JSONDecodeError, TypeError):
        return None
    if not isinstance(data, dict):
        return None
    for key in _EMITTER_KEYS:
        if key not in data:
            continue
        try:
            value = data[key]
            return int(value) if value in (0, 1, "0", "1") else None
        except (TypeError, ValueError, OverflowError):
            return None
    return None


def nearest_emitter(
    samples: Iterable[tuple[float, int]],
    image_stamp: float,
    slop: float = 0.02,
) -> int | None:
    best: tuple[float, int] | None = None
    conflict = False
    for stamp, mode in samples:
        dt = abs(stamp - image_stamp)
        if dt > slop:
            continue
        if best is not None and dt == best[0] and mode != best[1]:
            conflict = True
        if best is None or dt < best[0]:
            best = (dt, mode)
            conflict = False
    return None if best is None or conflict else best[1]


def _is_off(emitter: int | None) -> bool:
    return emitter == EMITTER_OFF


def _is_on(emitter: int | None) -> bool:
    return emitter == EMITTER_ON


def find_pair(
    ir_frames: Iterable[FrameRef],
    depth_frames: Iterable[FrameRef],
    *,
    max_interval: float = 0.05,
    min_interval: float = 0.02,
    allow_unlabeled: bool = False,
) -> tuple[FrameRef, FrameRef] | None:
    """Return (infra, depth) for a clean IR frame and a projected depth frame.

    Infra must be confidently off and depth confidently on. ``allow_unlabeled``
    remains a compatibility argument, but uncertain exposures are never paired.
    """
    ir_list = list(ir_frames)
    depth_list = list(depth_frames)
    ir_candidates = [
        frame
        for frame in ir_list
        if _is_off(frame.emitter)
    ]
    if not ir_candidates or not depth_list:
        return None
    ir = ir_candidates[-1]
    best: tuple[float, FrameRef] | None = None
    for depth in depth_list:
        if ir.stamp_ns is not None and depth.stamp_ns is not None:
            dt = abs(depth.stamp_ns - ir.stamp_ns) / 1_000_000_000
        else:
            dt = abs(depth.stamp - ir.stamp)
        if not math.isfinite(dt):
            continue
        if dt > max_interval:
            continue
        if not _is_on(depth.emitter):
            continue
        if best is None or dt < best[0]:
            best = (dt, depth)
    if best is None:
        return None
    return ir, best[1]


def pair_provenance(ir: FrameRef, depth: FrameRef) -> dict:
    """Describe original exposures; integer ROS stamps must be supplied by the node."""
    if ir.stamp_ns is None or depth.stamp_ns is None:
        raise ValueError("original nanosecond stamps are required for provenance")
    return {
        "ir_stamp_ns": ir.stamp_ns,
        "depth_stamp_ns": depth.stamp_ns,
        "output_stamp_ns": ir.stamp_ns,
        "delta_ns": abs(depth.stamp_ns - ir.stamp_ns),
        "ir": {"stamp_ns": ir.stamp_ns, "emitter": ir.emitter, "source": ir.source},
        "depth": {"stamp_ns": depth.stamp_ns, "emitter": depth.emitter, "source": depth.source},
    }


def camera_info_error(info, image) -> str | None:
    """Check the calibration actually describes this image, using duck-typed ROS messages."""
    if info is None:
        return "missing_camera_info"
    try:
        info_stamp = info.header.stamp
        image_stamp = image.header.stamp
        if (int(info_stamp.sec) * 1_000_000_000 + int(info_stamp.nanosec)
                != int(image_stamp.sec) * 1_000_000_000 + int(image_stamp.nanosec)):
            return "camera_info_stamp"
        if (info.width, info.height) != (image.width, image.height):
            return "camera_info_dimensions"
        if not image.header.frame_id or info.header.frame_id != image.header.frame_id:
            return "camera_info_frame"
        if len(info.k) != 9 or len(info.r) != 9 or len(info.p) != 12:
            return "camera_info_calibration"
        if not all(math.isfinite(v) for v in (*info.k, *info.r, *info.p, *info.d)):
            return "camera_info_calibration"
        if (info.k[0] <= 0 or info.k[4] <= 0 or info.p[0] <= 0 or info.p[5] <= 0
                or abs(info.k[8] - 1) > 1e-3 or abs(info.p[10] - 1) > 1e-3):
            return "camera_info_calibration"
        if any(abs(info.k[i]) > 1e-3 for i in (6, 7)):
            return "camera_info_calibration"
        if any(abs(info.p[i]) > 1e-3 for i in (8, 9, 11)):
            return "camera_info_calibration"
        rows = [info.r[i:i + 3] for i in (0, 3, 6)]
        if any(abs(sum(a * b for a, b in zip(rows[i], rows[j])) - (i == j)) > 1e-2
               for i in range(3) for j in range(3)):
            return "camera_info_calibration"
        r = info.r
        determinant = (r[0] * (r[4] * r[8] - r[5] * r[7])
                       - r[1] * (r[3] * r[8] - r[5] * r[6])
                       + r[2] * (r[3] * r[7] - r[4] * r[6]))
        if abs(determinant - 1) > 1e-2:
            return "camera_info_calibration"
    except (AttributeError, TypeError, ValueError):
        return "camera_info_calibration"
    return None


def image_array(image, *, depth: bool = False):
    """Decode a padded mono image or return None for malformed buffers/encodings."""
    import numpy as np

    try:
        encodings = ("16UC1", "mono16") if depth else ("mono8", "8UC1")
        size = 2 if depth else 1
        if image.encoding not in encodings or image.width <= 0 or image.height <= 0:
            return None
        if image.step < image.width * size or image.step % size:
            return None
        if image.is_bigendian not in (0, 1):
            return None
        data = memoryview(image.data)
        if data.nbytes != image.height * image.step:
            return None
        dtype = np.dtype(">u2" if image.is_bigendian else "<u2") if depth else np.dtype("u1")
        return np.frombuffer(data, dtype=dtype).reshape(image.height, image.step // size)[:, :image.width]
    except (AttributeError, TypeError, ValueError, BufferError, OverflowError):
        return None


def output_stamp_is_new(last_output_stamp: float | None, ir_stamp: float) -> bool:
    """Paired output uses the IR stamp. VO drops a stamp that does not increase."""
    if last_output_stamp is None:
        return True
    return ir_stamp > last_output_stamp


# --- Emitter state from the images themselves -------------------------------
#
# The D435C on this robot publishes metadata without a frame emitter mode, so
# the on/off state of each frame is read from the images. With emitter_on_off
# the projector toggles every frame, and infrared and depth of one frame share
# the same stamp. A frame taken with the projector on shows many isolated
# bright dots in the infrared image and has more valid pixels in its depth
# image than its neighbours.

DOT_CONTRAST = 25.0


def ir_dot_density(image, contrast: float = DOT_CONTRAST) -> float:
    """Fraction of pixels brighter than the mean of their four neighbours by ``contrast``.

    ``image`` is a 2-D uint8 numpy array. Every second row is sampled.
    """
    import numpy as np

    a = image.astype(np.int16)
    if a.ndim != 2 or a.shape[0] < 4 or a.shape[1] < 3:
        return 0.0
    centre = a[1:-1:2, 1:-1]
    up = a[0:-2:2, 1:-1]
    down = a[2::2, 1:-1][: centre.shape[0]]
    left = a[1:-1:2, 0:-2]
    right = a[1:-1:2, 2:]
    neighbours = (up + down + left + right) / 4.0
    return float(np.count_nonzero(centre - neighbours > contrast)) / float(centre.size)


def depth_fill_ratio(depth, step: int = 4) -> float:
    """Fraction of non-zero pixels in a 2-D depth array sampled every ``step`` pixels."""
    import numpy as np

    sub = depth[::step, ::step]
    if sub.size == 0:
        return 0.0
    return float(np.count_nonzero(sub)) / float(sub.size)


@dataclass
class FrameScore:
    stamp: float
    dots: float | None = None
    fill: float | None = None


def _vote(older: float | None, newer: float | None, margin: float) -> int:
    """+1 when ``newer`` is clearly larger, -1 when clearly smaller, else 0."""
    if older is None or newer is None:
        return 0
    hi, lo = max(older, newer), min(older, newer)
    if hi <= 0.0:
        return 0
    if lo > 0.0 and hi / lo < 1.0 + margin:
        return 0
    return 1 if newer > older else -1


def compare_adjacent(
    older: FrameScore,
    newer: FrameScore,
    dot_margin: float = 0.5,
    fill_margin: float = 0.08,
) -> int:
    """+1: ``newer`` was taken with the projector on. -1: ``older`` was. 0: undecided.

    The infrared dot density and the depth fill ratio vote separately. A clear
    vote from one decides when the other is undecided. Opposite votes leave the
    pair undecided.
    """
    by_dots = _vote(older.dots, newer.dots, dot_margin)
    by_fill = _vote(older.fill, newer.fill, fill_margin)
    if by_dots and by_fill and by_dots != by_fill:
        return 0
    return by_dots or by_fill


def stamp_key(stamp: float) -> int:
    return int(round(stamp * 1e4))


class EmitterClassifier:
    """Label each frame stamp as projector on or off by comparing it with adjacent frames."""

    def __init__(
        self,
        frame_gap: float = 0.05,
        keep: int = 32,
        dot_margin: float = 0.5,
        fill_margin: float = 0.08,
    ) -> None:
        self.frame_gap = frame_gap
        self.keep = keep
        self.dot_margin = dot_margin
        self.fill_margin = fill_margin
        self._keys: list[int] = []
        self._scores: dict[int, FrameScore] = {}
        self._labels: dict[int, int | None] = {}

    def update(self, stamp: float, dots: float | None = None, fill: float | None = None) -> None:
        key = stamp_key(stamp)
        score = self._scores.get(key)
        if score is None:
            score = FrameScore(stamp=stamp)
            self._scores[key] = score
            bisect.insort(self._keys, key)
        if dots is not None:
            score.dots = dots
        if fill is not None:
            score.fill = fill
        self._trim()
        # Rebuild from current evidence: a late IR/depth callback can invalidate
        # an earlier comparison. Never publish provisional image-only labels.
        self._labels.clear()
        for older_key, newer_key in zip(self._keys, self._keys[1:]):
            older, newer = self._scores[older_key], self._scores[newer_key]
            if (newer.stamp - older.stamp <= self.frame_gap
                    and all(value is not None for value in
                            (older.dots, older.fill, newer.dots, newer.fill))):
                self._apply(older, newer)

    def _apply(self, older: FrameScore, newer: FrameScore) -> None:
        dots = _vote(older.dots, newer.dots, self.dot_margin)
        fill = _vote(older.fill, newer.fill, self.fill_margin)
        if dots and fill and dots != fill:
            self._labels[stamp_key(older.stamp)] = None
            self._labels[stamp_key(newer.stamp)] = None
            return
        vote = compare_adjacent(older, newer, self.dot_margin, self.fill_margin)
        if vote == 0:
            return
        self._set(newer.stamp, EMITTER_ON if vote > 0 else EMITTER_OFF)
        self._set(older.stamp, EMITTER_OFF if vote > 0 else EMITTER_ON)

    def _set(self, stamp: float, label: int) -> None:
        key = stamp_key(stamp)
        if key in self._labels and self._labels[key] is not None and self._labels[key] != label:
            self._labels[key] = None
            return
        if key in self._labels and self._labels[key] is None:
            return
        self._labels[key] = label

    def _trim(self) -> None:
        while len(self._keys) > self.keep:
            key = self._keys.pop(0)
            self._scores.pop(key, None)
            self._labels.pop(key, None)

    def label(self, stamp: float) -> int | None:
        return self._labels.get(stamp_key(stamp))

    def score(self, stamp: float) -> FrameScore | None:
        return self._scores.get(stamp_key(stamp))
