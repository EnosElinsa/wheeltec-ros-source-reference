import struct
import zlib

from wheeltec_vision_nav.capture_runtime_logic import (
    build_summary,
    encode_depth16_preview_png,
    encode_gray8_png,
    encode_occupancy_png,
    image_sample,
    occupancy_rgb,
    parse_png_size,
    stamp_to_sec,
    tf_parents,
)


def test_stamp_to_sec():
    assert abs(stamp_to_sec(1, 500_000_000) - 1.5) < 1e-9


def test_gray8_png_is_valid_and_round_trips_size():
    pixels = bytes([0, 255, 128, 64])
    png = encode_gray8_png(2, 2, pixels)
    assert png.startswith(b"\x89PNG\r\n\x1a\n")
    assert parse_png_size(png) == (2, 2)
    ihdr = _chunk_data(png, b"IHDR")
    bit_depth, color_type = ihdr[8], ihdr[9]
    assert bit_depth == 8
    assert color_type == 0


def test_depth16_maps_zero_to_black_and_farther_brighter():
    data = struct.pack("<HHHH", 0, 1000, 2000, 0)
    png = encode_depth16_preview_png(2, 2, data)
    raw = _png_gray_pixels(png, 2, 2)
    assert raw[0] == 0
    assert raw[3] == 0
    assert raw[1] < raw[2]


def test_occupancy_rgb_unknown_free_occupied():
    assert occupancy_rgb(-1) == (128, 128, 128)
    assert occupancy_rgb(0) == (255, 255, 255)
    assert occupancy_rgb(100) == (0, 0, 0)
    assert occupancy_rgb(50) == (128, 128, 128)


def test_occupancy_png_size():
    png = encode_occupancy_png(2, 1, [-1, 100])
    assert parse_png_size(png) == (2, 1)


def test_image_sample_omits_pixel_payload():
    sample = image_sample(
        encoding="mono8",
        width=848,
        height=480,
        frame_id="camera_infra1_optical_frame",
        stamp_sec=10,
        stamp_nsec=20,
        step=848,
        data_len=848 * 480,
    )
    assert sample == {
        "encoding": "mono8",
        "width": 848,
        "height": 480,
        "frame_id": "camera_infra1_optical_frame",
        "stamp_sec": 10.00000002,
        "step": 848,
        "data_len": 407040,
    }
    assert "data" not in sample


def test_tf_parents_keep_last_parent_per_child():
    parents = tf_parents(
        [
            ("odom", "base_footprint"),
            ("wheel_odom", "base_footprint"),
            ("base_link", "camera_link"),
        ]
    )
    assert parents["base_footprint"] == "wheel_odom"
    assert parents["camera_link"] == "base_link"


def test_summary_collects_doc_fields():
    summary = build_summary(
        {
            "infra": image_sample("mono8", 848, 480, "camera_infra1_optical_frame", 1, 0, 848, 100),
            "depth": image_sample("16UC1", 848, 480, "camera_depth_optical_frame", 1, 0, 1696, 200),
            "paired_infra": image_sample("mono8", 848, 480, "camera_infra1_optical_frame", 1, 50, 848, 100),
            "paired_depth": image_sample("16UC1", 848, 480, "camera_depth_optical_frame", 1, 50, 1696, 200),
            "camera_info_k": [425.1, 0.0, 424.0, 0.0, 425.2, 240.0],
            "map": {"resolution": 0.05, "width": 120, "height": 80, "frame_id": "map"},
            "odom": {
                "frame_id": "odom",
                "child_frame_id": "base_footprint",
                "quat_norm": 1.0,
            },
            "odom_info": {"lost": False, "inliers": 87, "features": 210},
            "params": {
                "vision_rgbd_odometry.guess_frame_id": "",
                "vision_rgbd_odometry.publish_tf": True,
                "controller_server.FollowPath.vx_min": -0.15,
            },
            "tf_parents": {"base_footprint": "odom", "camera_link": "base_link"},
            "hz": {"/camera/infra1/image_rect_raw": 14.8, "/odom": 14.7},
            "emitter_keys": ["frame_emitter_mode"],
            "images": ["infra1_000.png"],
        }
    )
    assert summary["infra_encoding"] == "mono8"
    assert summary["depth_encoding"] == "16UC1"
    assert summary["infra_frame_id"] == "camera_infra1_optical_frame"
    assert summary["depth_frame_id"] == "camera_depth_optical_frame"
    assert summary["paired_stamp_delta_sec"] == 0.0
    assert summary["map_resolution"] == 0.05
    assert summary["map_size"] == [120, 80]
    assert summary["tf_base_footprint_parent"] == "odom"
    assert summary["vx_min"] == -0.15
    assert "infra1_000.png" in summary["images"]


def _chunk_data(png: bytes, tag: bytes) -> bytes:
    offset = 8
    while offset < len(png):
        length = struct.unpack(">I", png[offset : offset + 4])[0]
        name = png[offset + 4 : offset + 8]
        data = png[offset + 8 : offset + 8 + length]
        if name == tag:
            return data
        offset += 12 + length
    raise AssertionError(tag)


def _png_gray_pixels(png: bytes, width: int, height: int) -> bytes:
    data = zlib.decompress(_chunk_data(png, b"IDAT"))
    pixels = bytearray()
    stride = 1 + width
    assert len(data) == stride * height
    for y in range(height):
        row = data[y * stride : (y + 1) * stride]
        assert row[0] == 0
        pixels.extend(row[1:])
    return bytes(pixels)
