"""Pair emitter-off infrared with the nearest emitter-on depth frame."""

from __future__ import annotations

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
) -> tuple[FrameRef, FrameRef] | None:
    """Return (infra, depth) for a clean IR frame and a projected depth frame.

    With emitter metadata: infra must be off, depth must be on.
    Without metadata: skip hardware-synced (same-time) pairs and take the
    nearest depth within ``max_interval``, which matches on/off interleave.
    """
    ir_list = list(ir_frames)
    depth_list = list(depth_frames)
    ir_candidates = [frame for frame in ir_list if frame.emitter is None or _is_off(frame.emitter)]
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
        elif dt < min_interval:
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
