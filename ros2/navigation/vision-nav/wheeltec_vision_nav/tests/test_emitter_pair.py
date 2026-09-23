from pathlib import Path

from wheeltec_vision_nav.emitter_pair_logic import (
    FrameRef,
    find_pair,
    nearest_emitter,
    output_stamp_is_new,
    parse_emitter_mode,
)


def test_parse_emitter_mode_reads_common_keys():
    assert parse_emitter_mode('{"frame_emitter_mode": 0}') == 0
    assert parse_emitter_mode('{"Frame Emitter Mode": 1}') == 1
    assert parse_emitter_mode("{") is None
    assert parse_emitter_mode(None) is None


def test_nearest_emitter_uses_closest_stamp_within_slop():
    samples = ((1.00, 0), (1.07, 1))
    assert nearest_emitter(samples, 1.005, slop=0.02) == 0
    assert nearest_emitter(samples, 1.07, slop=0.02) == 1
    assert nearest_emitter(samples, 1.20, slop=0.02) is None


def test_find_pair_prefers_emitter_off_ir_and_on_depth():
    ir = [
        FrameRef(stamp=1.00, emitter=1, payload="ir-on"),
        FrameRef(stamp=1.07, emitter=0, payload="ir-off"),
    ]
    depth = [
        FrameRef(stamp=1.00, emitter=1, payload="depth-on"),
        FrameRef(stamp=1.07, emitter=0, payload="depth-off"),
    ]
    pair = find_pair(ir, depth, max_interval=0.12)
    assert pair is not None
    infra, dep = pair
    assert infra.payload == "ir-off"
    assert dep.payload == "depth-on"
    assert abs(dep.stamp - infra.stamp) <= 0.12


def test_find_pair_without_metadata_skips_same_stamp():
    ir = [FrameRef(stamp=2.00, emitter=None, payload="ir")]
    same = [FrameRef(stamp=2.00, emitter=None, payload="depth-same")]
    assert find_pair(ir, same, max_interval=0.12, min_interval=0.02) is None
    interleaved = [FrameRef(stamp=2.07, emitter=None, payload="depth-next")]
    pair = find_pair(ir, interleaved, max_interval=0.12, min_interval=0.02)
    assert pair is not None
    assert pair[1].payload == "depth-next"


def test_find_pair_rejects_beyond_sync_window():
    ir = [FrameRef(stamp=3.00, emitter=0, payload="ir")]
    depth = [FrameRef(stamp=3.50, emitter=1, payload="depth")]
    assert find_pair(ir, depth, max_interval=0.12) is None


def test_output_stamp_must_strictly_increase():
    # Paired depth is rewritten to the IR stamp. A newer depth with the same IR
    # would otherwise republish the same RGB-D stamp and rgbd_odometry drops it.
    assert output_stamp_is_new(None, 1.00)
    assert output_stamp_is_new(1.00, 1.07)
    assert not output_stamp_is_new(1.00, 1.00)
    assert not output_stamp_is_new(1.00, 0.99)
    same_ir, newer_depth = 1.07, 1.13
    pair_a = find_pair(
        [FrameRef(stamp=same_ir, emitter=0, payload="ir")],
        [FrameRef(stamp=1.00, emitter=1, payload="depth-on")],
        max_interval=0.12,
    )
    pair_b = find_pair(
        [FrameRef(stamp=same_ir, emitter=0, payload="ir")],
        [FrameRef(stamp=newer_depth, emitter=1, payload="depth-newer")],
        max_interval=0.12,
    )
    assert pair_a is not None and pair_b is not None
    assert pair_a[0].stamp == pair_b[0].stamp
    assert pair_a[1].stamp != pair_b[1].stamp
    assert not output_stamp_is_new(pair_a[0].stamp, pair_b[0].stamp)
    node = (Path(__file__).parents[1] / "wheeltec_vision_nav" / "emitter_pair.py").read_text(
        encoding="utf-8"
    )
    assert "output_stamp_is_new(self._last_output_stamp, ir.stamp)" in node
