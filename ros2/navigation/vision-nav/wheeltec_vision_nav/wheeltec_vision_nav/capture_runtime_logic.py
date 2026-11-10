"""Pure helpers for runtime capture: PNG previews and a YAML summary."""

from __future__ import annotations

import struct
import zlib
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path


class BoundedWriter:
    """One disk/encoding worker with a bounded queue; callbacks never wait."""
    def __init__(self, capacity=2):
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="vision-preview")
        self._pending = []
        self.capacity = capacity
        self.errors = []

    def collect(self):
        remaining = []
        for future in self._pending:
            if future.done():
                try:
                    future.result()
                except Exception as error:
                    self.errors.append(str(error))
            else:
                remaining.append(future)
        self._pending = remaining

    def submit(self, callback, *args):
        self.collect()
        if len(self._pending) >= self.capacity:
            return False
        self._pending.append(self._executor.submit(callback, *args))
        return True

    def close(self):
        self._executor.shutdown(wait=True)
        self.collect()


def write_image_preview(path, width, height, encoding, step, big_endian, data):
    bytes_per_pixel = 1 if encoding in ("mono8", "8UC1") else 2
    if encoding not in ("mono8", "8UC1", "16UC1", "mono16"):
        raise ValueError("unsupported preview encoding")
    row_bytes = width * bytes_per_pixel
    if width <= 0 or height <= 0 or step < row_bytes or len(data) != step * height:
        raise ValueError("malformed preview buffer")
    payload = b"".join(bytes(data[y*step:y*step+row_bytes]) for y in range(height))
    png = (encode_gray8_png(width, height, payload) if bytes_per_pixel == 1 else
           encode_depth16_preview_png(width, height, payload, little_endian=not big_endian))
    Path(path).write_bytes(png)


def write_map_preview(path, width, height, data):
    Path(path).write_bytes(encode_occupancy_png(width, height, data))


def stamp_to_sec(sec: int, nanosec: int) -> float:
    return float(sec) + float(nanosec) * 1e-9


def image_sample(
    encoding: str,
    width: int,
    height: int,
    frame_id: str,
    stamp_sec: int,
    stamp_nsec: int,
    step: int,
    data_len: int,
) -> dict:
    return {
        "encoding": encoding,
        "width": int(width),
        "height": int(height),
        "frame_id": frame_id,
        "stamp_sec": stamp_to_sec(stamp_sec, stamp_nsec),
        "step": int(step),
        "data_len": int(data_len),
    }


def occupancy_rgb(value: int) -> tuple[int, int, int]:
    if value == 0:
        return (255, 255, 255)
    if value == 100:
        return (0, 0, 0)
    return (128, 128, 128)


def tf_parents(pairs: list[tuple[str, str]]) -> dict[str, str]:
    parents: dict[str, str] = {}
    for parent, child in pairs:
        parents[child] = parent
    return parents


def parse_png_size(png: bytes) -> tuple[int, int]:
    if png[:8] != b"\x89PNG\r\n\x1a\n":
        raise ValueError("not a PNG")
    length = struct.unpack(">I", png[8:12])[0]
    if png[12:16] != b"IHDR" or length < 8:
        raise ValueError("missing IHDR")
    width, height = struct.unpack(">II", png[16:24])
    return int(width), int(height)


def encode_gray8_png(width: int, height: int, pixels: bytes) -> bytes:
    if len(pixels) != width * height:
        raise ValueError("gray pixel count does not match size")
    raw = bytearray()
    for y in range(height):
        start = y * width
        raw.append(0)
        raw.extend(pixels[start : start + width])
    return _png_bytes(width, height, color_type=0, raw=bytes(raw))


def encode_rgb8_png(width: int, height: int, pixels: bytes) -> bytes:
    if len(pixels) != width * height * 3:
        raise ValueError("rgb pixel count does not match size")
    raw = bytearray()
    row = width * 3
    for y in range(height):
        start = y * row
        raw.append(0)
        raw.extend(pixels[start : start + row])
    return _png_bytes(width, height, color_type=2, raw=bytes(raw))


def encode_depth16_preview_png(width: int, height: int, data: bytes, little_endian: bool = True) -> bytes:
    needed = width * height * 2
    if len(data) < needed:
        raise ValueError("depth buffer shorter than width*height")
    endian = "<" if little_endian else ">"
    values = struct.unpack(f"{endian}{width * height}H", data[:needed])
    clip_mm = 4000
    pixels = bytes(
        0 if value == 0 else min(255, round(min(value, clip_mm) * 255 / clip_mm)) for value in values
    )
    return encode_gray8_png(width, height, pixels)


def encode_occupancy_png(width: int, height: int, data: list[int]) -> bytes:
    if len(data) < width * height:
        raise ValueError("occupancy buffer shorter than width*height")
    rgb = bytearray()
    for y in range(height - 1, -1, -1):
        start = y * width
        for value in data[start : start + width]:
            rgb.extend(occupancy_rgb(int(value)))
    return encode_rgb8_png(width, height, bytes(rgb))


def build_summary(parts: dict) -> dict:
    infra = parts.get("infra") or {}
    depth = parts.get("depth") or {}
    paired_infra = parts.get("paired_infra") or {}
    paired_depth = parts.get("paired_depth") or {}
    info = parts.get("map") or {}
    odom = parts.get("odom") or {}
    odom_info = parts.get("odom_info") or {}
    params = parts.get("params") or {}
    parents = parts.get("tf_parents") or {}
    paired_delta = None
    if "stamp_sec" in paired_infra and "stamp_sec" in paired_depth:
        paired_delta = abs(float(paired_infra["stamp_sec"]) - float(paired_depth["stamp_sec"]))
    k = parts.get("camera_info_k") or []
    fx = fy = cx = cy = None
    if len(k) >= 6:
        fx, cx, fy, cy = float(k[0]), float(k[2]), float(k[4]), float(k[5])
    return {
        "infra_encoding": infra.get("encoding"),
        "infra_frame_id": infra.get("frame_id"),
        "infra_size": [infra.get("width"), infra.get("height")] if infra else None,
        "depth_encoding": depth.get("encoding"),
        "depth_frame_id": depth.get("frame_id"),
        "depth_size": [depth.get("width"), depth.get("height")] if depth else None,
        "paired_stamp_delta_sec": paired_delta,
        "paired_stamp_delta_meaning": "output stamps; not acquisition synchronization",
        "original_pair_delta_sec": (parts["pair_provenance"]["delta_ns"] / 1e9
                                     if parts.get("pair_provenance") else None),
        "camera_fx": fx,
        "camera_fy": fy,
        "camera_cx": cx,
        "camera_cy": cy,
        "map_resolution": info.get("resolution"),
        "map_size": [info.get("width"), info.get("height")] if info else None,
        "map_frame_id": info.get("frame_id"),
        "odom_frame_id": odom.get("frame_id"),
        "odom_child_frame_id": odom.get("child_frame_id"),
        "odom_quat_norm": odom.get("quat_norm"),
        "odom_lost": odom_info.get("lost"),
        "odom_inliers": odom_info.get("inliers"),
        "odom_features": odom_info.get("features"),
        "tf_base_footprint_parent": parents.get("base_footprint"),
        "guess_frame_id": params.get("vision_rgbd_odometry.guess_frame_id"),
        "publish_tf": params.get("vision_rgbd_odometry.publish_tf"),
        "vx_min": params.get("controller_server.FollowPath.vx_min"),
        "hz": parts.get("hz") or {},
        "emitter_keys": parts.get("emitter_keys") or [],
        "images": list(parts.get("images") or []),
    }


def write_yaml(path: Path, payload: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(_yaml(payload), encoding="utf-8")


def _png_bytes(width: int, height: int, color_type: int, raw: bytes) -> bytes:
    ihdr = struct.pack(">IIBBBBB", width, height, 8, color_type, 0, 0, 0)
    return b"\x89PNG\r\n\x1a\n" + _chunk(b"IHDR", ihdr) + _chunk(b"IDAT", zlib.compress(raw, 9)) + _chunk(b"IEND", b"")


def _chunk(tag: bytes, data: bytes) -> bytes:
    crc = zlib.crc32(tag + data) & 0xFFFFFFFF
    return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", crc)


def _yaml(value, indent: int = 0) -> str:
    pad = "  " * indent
    if isinstance(value, dict):
        if not value:
            return "{}\n"
        lines = []
        for key, item in value.items():
            if isinstance(item, (dict, list)):
                rendered = _yaml(item, indent + 1)
                if rendered == "{}\n" or rendered == "[]\n":
                    lines.append(f"{pad}{key}: {rendered}")
                else:
                    lines.append(f"{pad}{key}:\n{rendered}")
            else:
                lines.append(f"{pad}{key}: {_scalar(item)}\n")
        return "".join(lines)
    if isinstance(value, list):
        if not value:
            return "[]\n"
        lines = []
        for item in value:
            if isinstance(item, (dict, list)):
                inner = _yaml(item, indent + 1)
                lines.append(f"{pad}- \n{inner}")
            else:
                lines.append(f"{pad}- {_scalar(item)}\n")
        return "".join(lines)
    return f"{pad}{_scalar(value)}\n"


def _scalar(value) -> str:
    if value is None:
        return "null"
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float):
        return repr(value)
    if isinstance(value, int):
        return str(value)
    text = str(value)
    if text == "" or any(ch in text for ch in ":#{}[]&*?,'\"\n"):
        return '"' + text.replace("\\", "\\\\").replace('"', '\\"') + '"'
    return text
