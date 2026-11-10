"""Catch adjacent-frame pairing, guessed emitter state and header rewriting."""
from wheeltec_vision_nav.vins_stereo_logic import StereoBuffer


def info(stamp, right=False):
    return {"stamp_ns": stamp, "width": 848, "height": 480,
            "p": [420., 0., 420., -21. if right else 0., 0., 420., 240., 0., 0., 0., 1., 0.]}


def fill(buffer, stamp, right_stamp=None):
    right_stamp = stamp if right_stamp is None else right_stamp
    left, right = object(), object()
    buffer.add("left", stamp, left)
    buffer.add("right", right_stamp, right)
    buffer.add("left_info", stamp, info(stamp))
    buffer.add("right_info", right_stamp, info(right_stamp, True))
    return left, right


def test_adjacent_images_are_never_promoted_to_synchronized_pair():
    b = StereoBuffer()
    fill(b, 1_000_000_000, 1_033_333_333)
    b.label(1_000_000_000, 0, "metadata")
    assert b.ready() == []


def test_unknown_emitter_is_not_off_and_headers_are_preserved():
    b = StereoBuffer()
    left, right = fill(b, 1_000_000_001)
    assert b.ready() == []
    b.label(1_000_000_001, 0, "metadata")
    pairs = b.ready()
    assert len(pairs) == 1
    assert pairs[0].stamp_ns == 1_000_000_001
    assert pairs[0].left is left and pairs[0].right is right
    assert pairs[0].source == "metadata"
    assert b.ready() == []


def test_conflicting_metadata_and_on_frames_are_rejected():
    b = StereoBuffer()
    fill(b, 10)
    b.label(10, 1, "metadata")
    b.label(10, 0, "image")
    assert b.ready() == []


def test_bad_camera_baseline_and_missing_info_block_pair():
    b = StereoBuffer()
    fill(b, 10)
    b.label(10, 0, "metadata")
    b.add("right_info", 10, info(10, False))
    assert b.ready() == []


def test_buffers_are_bounded_and_out_of_order_old_frames_do_not_republish():
    b = StereoBuffer(capacity=2)
    for stamp in (10, 20, 30):
        fill(b, stamp)
        b.label(stamp, 0, "metadata")
    assert [p.stamp_ns for p in b.ready()] == [20, 30]
    fill(b, 20)
    b.label(20, 0, "metadata")
    assert b.ready() == []
