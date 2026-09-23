from wheeltec_vision_nav.ekf_inputs_logic import VX, VYAW, GyroBias, floor_twist_covariance


def test_bias_is_learned_only_after_the_wheels_settle():
    bias = GyroBias(settle_s=0.5)
    bias.note_wheel(0.0, 0.0, 0.0)
    assert bias.correct(0.2, 0.0008) == 0.0008  # not settled yet
    for i in range(100):
        t = 0.6 + 0.05 * i
        bias.note_wheel(t, 0.0, 0.0)
        bias.correct(t, 0.0008)
    assert abs(bias.bias - 0.0008) < 1e-9
    assert abs(bias.correct(6.0, 0.0008)) < 1e-9


def test_bias_is_frozen_while_driving():
    bias = GyroBias(settle_s=0.0)
    bias.note_wheel(0.0, 0.0, 0.0)
    bias.correct(0.0, 0.0008)
    bias.note_wheel(1.0, 0.25, 0.3)
    corrected = bias.correct(1.0, 0.3008)
    assert abs(corrected - 0.3) < 1e-9
    assert abs(bias.bias - 0.0008) < 1e-12


def test_vo_twist_covariance_gets_a_floor():
    cov = [0.0] * 36
    cov[VX] = 7.75e-7
    cov[VYAW] = 3e-8
    out = floor_twist_covariance(cov, {VX: 2.5e-3, VYAW: 1e-3})
    assert out[VX] == 2.5e-3 and out[VYAW] == 1e-3
    lost = [9999.0] * 36
    assert floor_twist_covariance(lost, {VX: 2.5e-3})[VX] == 9999.0
    nan = [float("nan")] * 36
    assert floor_twist_covariance(nan, {VX: 2.5e-3})[VX] == 2.5e-3
