"""Raw-depth validity and bounded exact CameraInfo pairing, independent of ROS."""
from collections import OrderedDict
import math

import numpy as np


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def depth_has_valid_returns(msg):
    """Weak emitter-off frames return False; malformed frames raise ValueError."""
    if msg.encoding not in ("16UC1", "mono16") or msg.width <= 0 or msg.height <= 0:
        raise ValueError("depth must be nonempty uint16")
    if msg.step < msg.width * 2 or msg.step % 2 or len(msg.data) != msg.height * msg.step:
        raise ValueError("malformed depth buffer")
    dtype = np.dtype(">u2" if msg.is_bigendian else "<u2")
    data = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.step // 2)[:, :msg.width]
    sample = data[::4, ::4]
    return np.count_nonzero((sample >= 200) & (sample <= 3000)) / sample.size >= .02


class ExactDepthGate:
    """Release original messages only, never retime or substitute calibration.

    Missing/weak input emits nothing, so it cannot clear the obstacle layer.
    The watchdog independently expires the last valid acquisition at .3 s.
    """
    def __init__(self, max_pending=8, timeout_s=.3, future_tolerance_s=.1):
        self.images, self.infos = OrderedDict(), OrderedDict()
        self.max_pending = max_pending
        self.timeout_ns = int(timeout_s * 1e9)
        self.future_ns = int(future_tolerance_s * 1e9)
        self.last_stamp = -1
        self.last_now_ns = None

    def _add(self, msg, now_ns, images):
        if self.last_now_ns is not None and now_ns < self.last_now_ns:
            self.images.clear()
            self.infos.clear()
            self.last_stamp = -1
        self.last_now_ns = now_ns
        stamp = stamp_ns(msg.header.stamp)
        if stamp <= 0 or stamp <= self.last_stamp or not -self.future_ns <= now_ns - stamp < self.timeout_ns:
            return None
        if images:
            try:
                if not depth_has_valid_returns(msg):
                    return None
            except (ValueError, TypeError, BufferError):
                return None
        cache = self.images if images else self.infos
        cache[stamp] = msg
        while len(cache) > self.max_pending:
            cache.popitem(last=False)
        for pending in (self.images, self.infos):
            for old in list(pending):
                if now_ns - old >= self.timeout_ns or old <= self.last_stamp:
                    pending.pop(old)
        if stamp not in self.images or stamp not in self.infos:
            return None
        image, info = self.images.pop(stamp), self.infos.pop(stamp)
        if (image.header.frame_id != info.header.frame_id or not image.header.frame_id
                or (image.width, image.height) != (info.width, info.height)
                or len(info.k) != 9 or not all(math.isfinite(v) for v in info.k)
                or info.k[0] <= 0 or info.k[4] <= 0):
            return None
        self.last_stamp = stamp
        return image, info

    def image(self, msg, now_ns):
        return self._add(msg, now_ns, True)

    def camera_info(self, msg, now_ns):
        return self._add(msg, now_ns, False)
