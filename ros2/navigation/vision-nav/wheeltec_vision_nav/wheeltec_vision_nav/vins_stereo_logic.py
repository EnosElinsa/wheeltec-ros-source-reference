"""Exact-exposure stereo selection. No timestamp adjustment or unlabeled fallback."""
from collections import OrderedDict
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class StereoPair:
    stamp_ns: int
    left: object
    right: object
    left_info: object
    right_info: object
    source: str


def valid_rectified_pair(left, right):
    try:
        lp, rp = left["p"], right["p"]
        if len(lp) != 12 or len(rp) != 12 or not all(math.isfinite(v) for v in (*lp, *rp)):
            return False
        return (left["width"] > 0 and left["height"] > 0
                and (left["width"], left["height"]) == (right["width"], right["height"])
                and lp[0] > 0 and lp[5] > 0 and rp[0] > 0 and rp[5] > 0
                and abs(lp[3]) < 1e-6 and -rp[3] / rp[0] > 0
                and abs(lp[0] - rp[0]) < 1e-3 and abs(lp[5] - rp[5]) < 1e-3
                and abs(lp[2] - rp[2]) < 1e-3 and abs(lp[6] - rp[6]) < 1e-3)
    except (KeyError, TypeError, ValueError):
        return False


class StereoBuffer:
    def __init__(self, capacity=60):
        if capacity < 1:
            raise ValueError("capacity must be positive")
        self.capacity = capacity
        self.frames = {k: OrderedDict() for k in ("left", "right", "left_info", "right_info")}
        self.labels = OrderedDict()
        self.last_output = -1

    def _put(self, mapping, stamp, value):
        mapping[stamp] = value
        while len(mapping) > self.capacity:
            mapping.popitem(last=False)

    def add(self, side, stamp_ns, payload):
        if stamp_ns > self.last_output and stamp_ns > 0:
            self._put(self.frames[side], stamp_ns, payload)

    def label(self, stamp_ns, mode, source):
        if stamp_ns <= self.last_output or mode not in (0, 1):
            return
        previous = self.labels.get(stamp_ns)
        if previous and previous[1] == "metadata":
            if source != "metadata":
                return
            if previous[0] != mode:
                mode = None  # contradictory hardware metadata never becomes an off frame
        self._put(self.labels, stamp_ns, (mode, source))

    def ready(self):
        common = set.intersection(*(set(v) for v in self.frames.values()))
        result = []
        for stamp in sorted(common):
            mode, source = self.labels.get(stamp, (None, "unknown"))
            if stamp <= self.last_output or mode != 0:
                continue
            li, ri = self.frames["left_info"][stamp], self.frames["right_info"][stamp]
            if not valid_rectified_pair(li, ri):
                continue
            result.append(StereoPair(stamp, self.frames["left"][stamp], self.frames["right"][stamp],
                                     li.get("message", li), ri.get("message", ri), source))
            self.last_output = stamp
        for mapping in (*self.frames.values(), self.labels):
            for stamp in list(mapping):
                if stamp <= self.last_output:
                    del mapping[stamp]
        return result
