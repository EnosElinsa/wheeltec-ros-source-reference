from dataclasses import replace
from types import SimpleNamespace as NS

import numpy as np
import pytest

from wheeltec_vision_nav.emitter_pair_logic import (
    EMITTER_OFF,
    EMITTER_ON,
    EmitterClassifier,
    FrameRef,
    FrameScore,
    camera_info_error,
    compare_adjacent,
    depth_fill_ratio,
    find_pair,
    ir_dot_density,
    image_array,
    nearest_emitter,
    output_stamp_is_new,
    parse_emitter_mode,
    pair_provenance,
)


def _scene(seed: int = 0) -> np.ndarray:
    rng = np.random.default_rng(seed)
    base = np.tile(np.linspace(60, 180, 848, dtype=np.float32), (480, 1))
    base += rng.normal(0.0, 3.0, base.shape)
    base[200:260, 300:500] = 40.0
    return np.clip(base, 0, 255).astype(np.uint8)


def _speckled(image: np.ndarray, seed: int = 1) -> np.ndarray:
    rng = np.random.default_rng(seed)
    out = image.astype(np.int16) + 8
    rows = rng.integers(1, image.shape[0] - 1, 4000)
    cols = rng.integers(1, image.shape[1] - 1, 4000)
    out[rows, cols] += 90
    return np.clip(out, 0, 255).astype(np.uint8)


def _depth(fill: float, seed: int = 2) -> np.ndarray:
    rng = np.random.default_rng(seed)
    depth = np.full((480, 848), 1500, dtype=np.uint16)
    depth[rng.random(depth.shape) > fill] = 0
    return depth


def test_dot_density_separates_speckled_infrared():
    clean = _scene()
    assert ir_dot_density(_speckled(clean)) > 2.0 * max(ir_dot_density(clean), 1e-4)


def test_depth_fill_ratio_counts_valid_pixels():
    assert abs(depth_fill_ratio(_depth(0.9)) - 0.9) < 0.03
    assert depth_fill_ratio(np.zeros((8, 8), dtype=np.uint16)) == 0.0


def test_compare_adjacent_votes_and_conflicts():
    off = FrameScore(stamp=1.000, dots=0.0006, fill=0.70)
    on = FrameScore(stamp=1.033, dots=0.0020, fill=0.90)
    assert compare_adjacent(off, on) == 1
    assert compare_adjacent(on, FrameScore(stamp=1.066, dots=0.0006, fill=0.70)) == -1
    # Dots undecided, fill decides.
    assert compare_adjacent(FrameScore(1.0, 0.001, 0.60), FrameScore(1.033, 0.0011, 0.90)) == 1
    # Opposite votes leave the pair undecided.
    assert compare_adjacent(FrameScore(1.0, 0.0006, 0.95), FrameScore(1.033, 0.0020, 0.60)) == 0


def test_classifier_labels_alternating_frames_from_images():
    classifier = EmitterClassifier()
    clean = _scene()
    stamps = [10.0 + i / 30.0 for i in range(6)]
    for i, stamp in enumerate(stamps):
        on = i % 2 == 1
        image = _speckled(clean, seed=i) if on else clean
        classifier.update(stamp, dots=ir_dot_density(image))
        classifier.update(stamp, fill=depth_fill_ratio(_depth(0.92 if on else 0.75, seed=i)))
    labels = [classifier.label(s) for s in stamps]
    assert labels == [EMITTER_OFF, EMITTER_ON] * 3


def test_classifier_ignores_frames_that_are_not_adjacent():
    classifier = EmitterClassifier(frame_gap=0.05)
    classifier.update(1.0, dots=0.0006)
    classifier.update(1.2, dots=0.0020)
    assert classifier.label(1.0) is None
    assert classifier.label(1.2) is None


def test_classifier_waits_for_complete_evidence_and_revokes_a_conflicting_label():
    classifier = EmitterClassifier()
    classifier.update(1., dots=.001, fill=.9)
    classifier.update(1.033, dots=.003)
    assert classifier.label(1.) is None
    assert classifier.label(1.033) is None
    classifier.update(1.033, fill=.99)
    assert classifier.label(1.) == EMITTER_OFF
    assert classifier.label(1.033) == EMITTER_ON
    classifier.update(1.033, fill=.4)
    assert classifier.label(1.) is None
    assert classifier.label(1.033) is None


def test_conflicting_edge_overrides_a_label_supported_by_another_neighbor():
    classifier = EmitterClassifier()
    classifier.update(1., dots=.003, fill=.9)
    classifier.update(1.033, dots=.001, fill=.4)
    assert classifier.label(1.033) == EMITTER_OFF
    classifier.update(1.066, dots=.003, fill=.2)
    assert classifier.label(1.033) is None
    assert classifier.label(1.066) is None


def test_unlabelled_frames_are_not_paired_when_labels_are_required():
    ir = [FrameRef(stamp=2.00, emitter=None, payload="ir")]
    depth = [FrameRef(stamp=2.033, emitter=None, payload="depth")]
    assert find_pair(ir, depth, allow_unlabeled=False) is None
    labelled = find_pair(
        [FrameRef(stamp=2.00, emitter=EMITTER_OFF, payload="ir")],
        [FrameRef(stamp=2.033, emitter=EMITTER_ON, payload="depth")],
        allow_unlabeled=False,
    )
    assert labelled is not None and labelled[1].payload == "depth"


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


def test_find_pair_without_metadata_never_guesses_even_for_adjacent_exposures():
    ir = [FrameRef(stamp=2.00, emitter=None, payload="ir")]
    same = [FrameRef(stamp=2.00, emitter=None, payload="depth-same")]
    assert find_pair(ir, same, max_interval=0.12, min_interval=0.02) is None
    interleaved = [FrameRef(stamp=2.07, emitter=None, payload="depth-next")]
    pair = find_pair(ir, interleaved, max_interval=0.12, min_interval=0.02)
    assert pair is None
    assert find_pair(ir, interleaved, allow_unlabeled=True) is None


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
    # Large epoch seconds collapse to the same float; integer stamps retain ordering.
    stamp = 1_800_000_000_000_000_001
    assert output_stamp_is_new(stamp, stamp + 1)


def test_default_pair_window_rejects_adjacent_gap_over_50ms():
    ir = FrameRef(1.0, EMITTER_OFF, stamp_ns=1_000_000_000)
    depth = FrameRef(1.06, EMITTER_ON, stamp_ns=1_060_000_000)
    assert find_pair([ir], [depth]) is None
    assert find_pair([ir], [replace(depth, stamp_ns=1_050_000_000)]) is not None


def test_provenance_retains_integer_epoch_stamps_and_individual_sources():
    stamp = 1_800_000_000_000_000_001
    ir = FrameRef(1.8e9, EMITTER_OFF, stamp_ns=stamp, source="metadata")
    depth = FrameRef(1.8e9, EMITTER_ON, stamp_ns=stamp + 33_333_333, source="image")
    provenance = pair_provenance(ir, depth)
    assert provenance["ir_stamp_ns"] == stamp
    assert provenance["depth_stamp_ns"] == stamp + 33_333_333
    assert provenance["delta_ns"] == 33_333_333
    assert provenance["ir"]["source"] == "metadata"
    assert provenance["depth"]["source"] == "image"
    with pytest.raises(ValueError):
        pair_provenance(FrameRef(1.0, 0), depth)


@pytest.mark.parametrize("value", [2, -1, 0.5, "on", None])
def test_unknown_metadata_modes_are_not_treated_as_projector_on(value):
    import json
    assert parse_emitter_mode(json.dumps({"frame_emitter_mode": value})) is None


def test_conflicting_nearest_metadata_is_uncertain_but_exact_metadata_can_resolve_it():
    assert nearest_emitter([(10, 0), (10, 1)], 10, slop=0) is None
    assert nearest_emitter([(9, 0), (11, 1), (10, 0)], 10, slop=2) == 0


def _image(*, depth=False, endian=0):
    return NS(width=3, height=2, step=8 if depth else 4,
              encoding="16UC1" if depth else "mono8", is_bigendian=endian,
              data=bytes(16 if depth else 8),
              header=NS(frame_id="infra1", stamp=NS(sec=1, nanosec=0)))


def _info(image):
    return NS(width=image.width, height=image.height,
              header=NS(frame_id="infra1", stamp=NS(**vars(image.header.stamp))),
              k=[100., 0., 1., 0., 100., 1., 0., 0., 1.], r=np.eye(3).ravel().tolist(),
              p=[100., 0., 1., 0., 0., 100., 1., 0., 0., 0., 1., 0.], d=[])


def test_padded_and_big_endian_images_decode_and_bad_buffers_fail_without_throwing():
    image = _image()
    assert image_array(image).shape == (2, 3)
    depth = _image(depth=True, endian=1)
    depth.data = np.array([[1, 256, 1024, 0], [2, 257, 1025, 0]], dtype=">u2").tobytes()
    assert image_array(depth, depth=True).tolist() == [[1, 256, 1024], [2, 257, 1025]]
    for field, value in [("data", b"short"), ("step", 5), ("width", 0),
                         ("is_bigendian", 2), ("encoding", "rgb8")]:
        malformed = NS(**vars(depth))
        setattr(malformed, field, value)
        assert image_array(malformed, depth=True) is None


def test_camera_info_checks_dimensions_frames_and_finite_calibration():
    image = _image()
    info = _info(image)
    assert camera_info_error(info, image) is None
    assert camera_info_error(None, image) == "missing_camera_info"
    info.width += 1
    assert camera_info_error(info, image) == "camera_info_dimensions"
    info.width = image.width
    info.header.frame_id = "other"
    assert camera_info_error(info, image) == "camera_info_frame"
    info.header.frame_id = "infra1"
    info.p[0] = float("nan")
    assert camera_info_error(info, image) == "camera_info_calibration"
    info.p[0], info.k[0] = 100., 0.
    assert camera_info_error(info, image) == "camera_info_calibration"


def test_camera_info_requires_the_exact_original_integer_exposure_stamp():
    image = _image()
    image.header.stamp = NS(sec=1_800_000_000, nanosec=1)
    info = _info(image)
    assert camera_info_error(info, image) is None
    info.header.stamp.nanosec = 2
    assert camera_info_error(info, image) == "camera_info_stamp"
