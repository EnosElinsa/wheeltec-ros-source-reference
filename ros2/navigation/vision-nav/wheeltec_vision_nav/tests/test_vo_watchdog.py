from wheeltec_vision_nav.vo_watchdog_logic import (
    STATUS_LOST,
    STATUS_OK,
    STATUS_RECOVERING,
    WatchdogConfig,
    WatchdogState,
    evaluate_sample,
    note_map_correction,
    note_wheel,
    quaternion_valid,
    should_zero_cmd,
)


def test_invalid_quaternion_trips_immediately():
    state = WatchdogState()
    cfg = WatchdogConfig()
    evaluate_sample(state, cfg, quat=(0.0, 0.0, 0.0, 0.0))
    assert state.lost
    assert state.status == STATUS_LOST
    assert should_zero_cmd(state)


def test_lost_flag_trips_immediately():
    state = WatchdogState()
    cfg = WatchdogConfig()
    evaluate_sample(state, cfg, lost_flag=True, inliers=80)
    assert state.status == STATUS_LOST
    assert should_zero_cmd(state)


def test_low_inliers_need_consecutive_frames():
    state = WatchdogState()
    cfg = WatchdogConfig(min_inliers=20, consecutive_bad=3)
    evaluate_sample(state, cfg, lost_flag=False, inliers=5)
    evaluate_sample(state, cfg, lost_flag=False, inliers=5)
    assert not state.lost
    evaluate_sample(state, cfg, lost_flag=False, inliers=5)
    assert state.lost
    assert should_zero_cmd(state)


def test_good_inliers_recover_then_ok():
    state = WatchdogState()
    cfg = WatchdogConfig()
    evaluate_sample(state, cfg, lost_flag=True, inliers=0)
    assert state.status == STATUS_LOST
    evaluate_sample(state, cfg, lost_flag=False, inliers=40)
    assert state.status == STATUS_RECOVERING
    assert not should_zero_cmd(state)
    evaluate_sample(state, cfg, lost_flag=False, inliers=40)
    assert state.status == STATUS_OK


def test_valid_odom_alone_does_not_clear_lost():
    state = WatchdogState()
    cfg = WatchdogConfig()
    evaluate_sample(state, cfg, lost_flag=True, inliers=0)
    evaluate_sample(state, cfg, quat=(0.0, 0.0, 0.0, 1.0))
    assert state.status == STATUS_LOST
    assert should_zero_cmd(state)


def test_quaternion_valid_threshold():
    assert quaternion_valid(0.0, 0.0, 0.0, 1.0)
    assert not quaternion_valid(0.0, 0.0, 0.0, 0.0)


def _lost_state() -> WatchdogState:
    state = WatchdogState()
    evaluate_sample(state, WatchdogConfig(), lost_flag=True, inliers=0)
    assert state.lost
    return state


def test_wheel_backup_keeps_driving_while_wheel_and_map_are_fresh():
    cfg = WatchdogConfig(wheel_backup=True, wheel_fresh_s=0.3, map_stale_s=5.0)
    state = _lost_state()
    note_wheel(state, 100.0)
    note_map_correction(state, 99.0)
    assert not should_zero_cmd(state, cfg, now=100.1)


def test_wheel_backup_stops_when_wheel_stream_stops():
    cfg = WatchdogConfig(wheel_backup=True, wheel_fresh_s=0.3, map_stale_s=5.0)
    state = _lost_state()
    note_wheel(state, 100.0)
    note_map_correction(state, 100.0)
    assert should_zero_cmd(state, cfg, now=100.5)


def test_wheel_backup_stops_when_map_correction_is_stale():
    cfg = WatchdogConfig(wheel_backup=True, wheel_fresh_s=0.3, map_stale_s=5.0)
    state = _lost_state()
    note_wheel(state, 110.0)
    note_map_correction(state, 100.0)
    assert should_zero_cmd(state, cfg, now=110.1)


def test_wheel_backup_requires_a_first_map_correction():
    cfg = WatchdogConfig(wheel_backup=True)
    state = _lost_state()
    note_wheel(state, 10.0)
    assert should_zero_cmd(state, cfg, now=10.1)


def test_without_wheel_backup_loss_stops_immediately():
    cfg = WatchdogConfig(wheel_backup=False)
    state = _lost_state()
    note_wheel(state, 10.0)
    note_map_correction(state, 10.0)
    assert should_zero_cmd(state, cfg, now=10.1)
    assert should_zero_cmd(state)


def test_healthy_vo_never_zeroes_regardless_of_backup():
    cfg = WatchdogConfig(wheel_backup=True)
    state = WatchdogState()
    evaluate_sample(state, cfg, lost_flag=False, inliers=50)
    assert not should_zero_cmd(state, cfg, now=1.0)
