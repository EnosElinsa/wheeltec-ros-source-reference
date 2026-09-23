"""Pair emitter-off infrared with the nearest emitter-on depth frame."""

from __future__ import annotations

import bisect
import json
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
            return int(data[key])
        except (TypeError, ValueError):
            return None
    return None


def nearest_emitter(
    samples: Iterable[tuple[float, int]],
    image_stamp: float,
    slop: float = 0.02,
) -> int | None:
    best: tuple[float, int] | None = None
    for stamp, mode in samples:
        dt = abs(stamp - image_stamp)
        if dt > slop:
            continue
        if best is None or dt < best[0]:
            best = (dt, mode)
    return None if best is None else best[1]


def _is_off(emitter: int | None) -> bool:
    return emitter == EMITTER_OFF


def _is_on(emitter: int | None) -> bool:
    return emitter is not None and emitter != EMITTER_OFF


def find_pair(
    ir_frames: Iterable[FrameRef],
    depth_frames: Iterable[FrameRef],
    *,
    max_interval: float = 0.12,
    min_interval: float = 0.02,
    allow_unlabeled: bool = True,
) -> tuple[FrameRef, FrameRef] | None:
    """Return (infra, depth) for a clean IR frame and a projected depth frame.

    Labelled frames: infra must be off, depth must be on.
    Unlabelled frames are used only when ``allow_unlabeled`` is True: skip
    hardware-synced (same-time) pairs and take the nearest depth within
    ``max_interval``. That fallback cannot tell which of the two frames carried
    the speckle, so the node disables it once image-based labels are available.
    """
    ir_list = list(ir_frames)
    depth_list = list(depth_frames)
    ir_candidates = [
        frame
        for frame in ir_list
        if _is_off(frame.emitter) or (allow_unlabeled and frame.emitter is None)
    ]
    if not ir_candidates or not depth_list:
        return None
    ir = ir_candidates[-1]
    best: tuple[float, FrameRef] | None = None
    for depth in depth_list:
        dt = abs(depth.stamp - ir.stamp)
        if dt > max_interval:
            continue
        if depth.emitter is not None:
            if not _is_on(depth.emitter):
                continue
        elif not allow_unlabeled or dt < min_interval:
            continue
        if best is None or dt < best[0]:
            best = (dt, depth)
    if best is None:
        return None
    return ir, best[1]


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
        index = self._keys.index(key)
        for other_index in (index - 1, index + 1):
            if 0 <= other_index < len(self._keys):
                other = self._scores[self._keys[other_index]]
                older, newer = (other, score) if other_index < index else (score, other)
                if newer.stamp - older.stamp <= self.frame_gap:
                    self._apply(older, newer)
        self._trim()

    def _apply(self, older: FrameScore, newer: FrameScore) -> None:
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
