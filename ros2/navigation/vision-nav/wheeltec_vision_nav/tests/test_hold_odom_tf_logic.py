from wheeltec_vision_nav.hold_odom_tf_logic import (
    OdomSample,
    OdomStitch,
    gated_odom_sample,
    identity_odom_sample,
    odom_sample_is_valid,
    stitch_odom_sample,
    wheel_relative_sample,
    wheel_source_is_fresh,
)


def test_zero_quaternion_is_invalid():
    sample = identity_odom_sample(1, 0)
    lost = OdomSample(
        stamp_sec=1,
        stamp_nsec=0,
        x=0.0,
        y=0.0,
        z=0.0,
        qx=0.0,
        qy=0.0,
        qz=0.0,
        qw=0.0,
        cov0=1e-5,
    )
    assert odom_sample_is_valid(sample)
    assert not odom_sample_is_valid(lost)


def test_gate_passes_valid_pose():
    incoming = identity_odom_sample(2, 0)
    assert gated_odom_sample(None, incoming) is incoming


def test_gate_reuses_last_valid_at_new_stamp():
    last = OdomSample(1, 0, 0.4, -0.1, 0.0, 0.0, 0.0, 0.0, 1.0, 1e-5)
    lost = OdomSample(2, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9999.0)
    out = gated_odom_sample(last, lost)
    assert odom_sample_is_valid(out)
    assert out.stamp_sec == 2
    assert out.x == 0.4
    assert out.y == -0.1


def test_gate_identity_when_never_valid():
    lost = OdomSample(3, 1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9999.0)
    out = gated_odom_sample(None, lost)
    assert odom_sample_is_valid(out)
    assert out.qw == 1.0
    assert out.x == 0.0
    assert out.stamp_sec == 3
    assert out.stamp_nsec == 1


def test_stitch_keeps_small_steps():
    state = OdomStitch()
    first = OdomSample(1, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1e-5)
    second = OdomSample(2, 0, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1e-5)
    assert stitch_odom_sample(state, first).x == 0.0
    out = stitch_odom_sample(state, second)
    assert abs(out.x - 0.05) < 1e-9


def test_stitch_continues_after_vo_reset_to_origin():
    state = OdomStitch()
    stitch_odom_sample(state, OdomSample(1, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1e-5))
    traveled = None
    for i, x in enumerate((0.2, 0.4, 0.6, 0.8, 1.0, 1.2), start=2):
        traveled = stitch_odom_sample(
            state, OdomSample(i, 0, x, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1e-5)
        )
    assert traveled is not None
    assert abs(traveled.x - 1.2) < 1e-9
    reset = stitch_odom_sample(state, OdomSample(20, 0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1e-5))
    assert abs(reset.x - 1.2) < 1e-6
    continued = stitch_odom_sample(state, OdomSample(21, 0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1e-5))
    assert abs(continued.x - 1.3) < 1e-6


def test_wheel_first_sample_is_odom_origin():
    incoming = OdomSample(1, 0, 4.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1e-5)
    origin, published = wheel_relative_sample(None, incoming)
    assert origin.x == 4.0
    assert origin.y == -1.0
    assert published.x == 0.0
    assert published.y == 0.0
    assert published.qw == 1.0


def test_wheel_relative_sample_tracks_one_meter():
    first = OdomSample(1, 0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1e-5)
    origin, _ = wheel_relative_sample(None, first)
    second = OdomSample(2, 0, 11.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1e-5)
    same_origin, published = wheel_relative_sample(origin, second)
    assert same_origin is origin
    assert abs(published.x - 1.0) < 1e-9
    assert abs(published.y) < 1e-9
    assert published.z == 0.0


def test_wheel_source_is_fresh_within_timeout():
    assert not wheel_source_is_fresh(1.0, 0.0)
    assert wheel_source_is_fresh(1.0, 0.8)
    assert not wheel_source_is_fresh(1.0, 0.69)
