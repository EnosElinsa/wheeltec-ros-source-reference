import math

import pytest

from wheeltec_vision_nav.runtime_safety import (
    DEGRADED, INITIALIZING, READY, RECOVERING, STOPPED, Cancellation, SafetyConfig,
    SafetyController, covariance_valid, observed_forward_range, quaternion_valid, swept_clearance,
)


def feed(controller, now, *, visual=True, omit=(), speed=0.0, lost=False):
    stamp = int((1000 + now) * 1e9)
    for source in controller._required():
        if source not in omit:
            if source == "wheel":
                controller.note_wheel(stamp, now, speed)
            else:
                controller.note(source, stamp, now)
    if visual:
        controller.note("vo", stamp, now)
        controller.note_visual(stamp, now, lost, 0 if lost else 40)


def ready(**kwargs):
    controller = SafetyController(SafetyConfig(wheel_backup=True, **kwargs))
    for index in range(14):
        now = index / 10
        feed(controller, now)
        controller.step(now)
    assert controller.state == READY
    return controller, now


def drive(controller, now, vx=.3, wz=0, **kwargs):
    controller.note_command(vx, wz, now)
    return controller.step(now, observed_range_m=2.0, **kwargs)


def test_startup_without_evidence_is_fail_closed():
    controller = SafetyController(SafetyConfig())
    controller.note_command(.3, 0, 0)
    result = controller.step(0)
    assert result.state == INITIALIZING and result.vx == 0


def test_fixed_3d_cloud_transform_matches_independent_quaternion_rotation():
    from types import SimpleNamespace as NS
    import numpy as np
    from wheeltec_vision_nav.runtime_safety import transform_points
    angle=.7;c,s=math.cos(angle),math.sin(angle)
    points=np.random.default_rng(17).normal(size=(20000,3))
    tf=NS(translation=NS(x=2.,y=-1.,z=.3),rotation=NS(x=0.,y=0.,z=math.sin(angle/2),w=math.cos(angle/2)))
    expected=np.column_stack((c*points[:,0]-s*points[:,1]+2,s*points[:,0]+c*points[:,1]-1,points[:,2]+.3))
    assert np.allclose(transform_points(points,tf),expected,rtol=1e-12,atol=1e-12)


@pytest.mark.parametrize("source", ["wheel", "imu", "odom", "depth", "cloud", "obstacles", "tf", "tf_authority", "ownership", "localization", "map_tf"])
def test_critical_failure_stops_even_with_healthy_vo(source):
    controller, now = ready()
    assert drive(controller, now).vx > 0
    later = now + controller.timeout(source) + .01
    feed(controller, later, omit=(source,))
    result = drive(controller, later)
    assert result.state == STOPPED and result.vx == 0 and result.cancel_required


def test_completely_silent_visual_output_uses_bounded_backup():
    controller, now = ready()
    later = now + .51
    feed(controller, later, visual=False)
    result = drive(controller, later)
    assert result.state == DEGRADED and 0 < result.vx <= .1
    for index in range(1, 12):
        current = later + index * .1
        feed(controller, current, visual=False)
        result = drive(controller, current)
    assert result.state == STOPPED and result.vx == 0


def test_degraded_wheel_distance_exhausts_before_time_budget():
    controller, now = ready()
    feed(controller, now + .1, lost=True)
    assert drive(controller, now + .1).state == DEGRADED
    for index in range(2, 7):
        feed(controller, now + index / 10, visual=False, speed=.3)
        result = drive(controller, now + index / 10)
    assert result.state == STOPPED


def test_recovery_requires_duration_samples_cancellation_and_new_goal():
    controller, now = ready()
    feed(controller, now + .1, omit=("depth",))
    controller.sources["depth"].valid = False
    assert drive(controller, now + .1).state == STOPPED
    for index in range(2, 16):
        current = now + index / 10
        feed(controller, current)
        result = drive(controller, current)
        assert result.vx == 0
    assert result.state == READY and result.new_goal_required
    assert not controller.rearm()
    controller.cancel_required = False
    assert controller.rearm()
    assert drive(controller, current).vx > 0


def test_a_single_good_frame_cannot_recover():
    controller, now = ready()
    feed(controller, now + .1, lost=True)
    drive(controller, now + .1)
    feed(controller, now + .2)
    result = drive(controller, now + .2)
    assert result.state == RECOVERING and result.vx == 0


def test_duplicates_and_delayed_packets_cannot_extend_freshness():
    controller, now = ready()
    item = controller.sources["vo"]
    assert not controller.note("vo", item.stamp_ns, now + .4)
    assert item.received == now
    assert not controller.note("vo", item.stamp_ns + 1, now + .4,
                               ros_now_ns=item.stamp_ns + 1_000_000_000)
    assert not item.valid


def test_future_and_backwards_stamps_are_rejected():
    controller = SafetyController(SafetyConfig())
    assert not controller.note("vo", 2_000_000_000, 1, ros_now_ns=1_000_000_000)
    assert controller.note("vo", 2_000_000_000, 1)
    assert not controller.note("vo", 1_999_999_999, 1.1)
    assert not controller.sources["vo"].valid


def test_transport_delay_consumes_the_freshness_budget():
    controller = SafetyController(SafetyConfig())
    assert controller.note("depth", 1_000_000_000, 10, ros_now_ns=1_250_000_000)
    assert controller.sources["depth"].fresh(10.04, .3)
    assert not controller.sources["depth"].fresh(10.06, .3)


@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_nonfinite_quaternion_and_command_fail_closed(bad):
    assert not quaternion_valid(bad, 0, 0, 1)
    controller, now = ready()
    result = drive(controller, now, vx=bad)
    assert result.vx == 0 and result.state in (STOPPED, RECOVERING)


def test_covariance_rejects_indefinite_and_asymmetric_used_submatrix():
    covariance = [0.] * 36
    covariance[0] = covariance[7] = covariance[35] = 1.
    assert covariance_valid(covariance)
    covariance[1] = covariance[6] = 2.
    assert not covariance_valid(covariance)
    covariance[1], covariance[6] = .1, .2
    assert not covariance_valid(covariance)


def test_command_timeout_clears_old_motion():
    controller, now = ready()
    assert drive(controller, now).vx > 0
    feed(controller, now + .51)
    result = controller.step(now + .51, observed_range_m=2)
    assert result.state == READY and result.vx == 0


def test_mapping_requires_no_map_localization_but_does_require_obstacles():
    controller, now = ready(mode="mapping")
    assert "localization" not in controller.sources
    assert "map_tf" not in controller.sources
    controller.sources["obstacles"].valid = False
    assert drive(controller, now).state == STOPPED
    assert not controller.cancel_required and controller.new_goal_required


def test_reverse_requires_opt_in_and_is_not_allowed_during_visual_loss():
    controller, now = ready()
    assert drive(controller, now, vx=-.1).vx == 0
    controller, now = ready(supervised_reverse=True)
    assert drive(controller, now, vx=-.3).vx == -.08
    feed(controller, now + .1, lost=True)
    result = drive(controller, now + .1, vx=-.1)
    assert result.state == STOPPED and result.vx == 0


def test_reverse_time_and_distance_limits():
    controller, now = ready(supervised_reverse=True)
    for index in range(35):
        current = now + index / 10
        feed(controller, current)
        result = drive(controller, current, vx=-.1)
        if result.state == STOPPED:
            break
    assert result.state == STOPPED and result.reason == "reverse_budget_exhausted"
    controller, now = ready(supervised_reverse=True)
    for index in range(12):
        current = now + index / 10
        feed(controller, current, speed=-.3)
        result = drive(controller, current, vx=-.1)
        if result.state == STOPPED:
            break
    assert result.state == STOPPED


def test_braking_limit_accounts_for_observation_age_and_preserves_curvature():
    controller, now = ready()
    early = drive(controller, now, wz=.4, clearance_m=.12, observation_age_s=0)
    later = drive(controller, now, wz=.4, clearance_m=.12, observation_age_s=.25)
    assert 0 < later.vx < early.vx < .3
    assert later.wz / later.vx == pytest.approx(early.wz / early.vx)
    assert abs(later.wz) <= later.vx / controller.cfg.min_turning_radius


def test_unobserved_corridor_or_obstacle_at_footprint_stops():
    controller, now = ready()
    controller.note_command(.2, 0, now)
    assert controller.step(now).state == STOPPED
    controller, now = ready()
    assert drive(controller, now, clearance_m=0).state == STOPPED


def test_swept_footprint_and_missing_lateral_visibility():
    cfg = SafetyConfig()
    assert swept_clearance([(1, 0, .2)], cfg, .2, 0) < 1
    assert math.isinf(swept_clearance([(1, 1, .2)], cfg, .2, 0))
    points = [(x, y, 0) for x in (1., 1.5, 2.) for y in (-.2, 0., .2)]
    assert observed_forward_range(points, cfg) == pytest.approx(1.75)
    assert observed_forward_range([(1., 0., 0.)] * 20, cfg) == 0


def test_clock_reset_invalidates_all_cached_evidence_and_motion():
    controller, now = ready()
    drive(controller, now)
    controller.clock_reset()
    assert not controller.sources and controller.state == STOPPED
    assert controller.step(now + .1).vx == 0


@pytest.mark.parametrize("kwargs", [{"reverse_speed": math.inf}, {"sensor_timeout_s": 0},
                                    {"mode": "other"}, {"degraded_speed": .5}])
def test_invalid_protection_configuration_is_rejected(kwargs):
    with pytest.raises(ValueError):
        SafetyConfig(**kwargs)


def test_cancellation_retries_unavailable_failed_and_timed_out_services_with_bound():
    attempt = Cancellation()
    assert attempt.due(0)
    attempt.begin(0)
    assert not attempt.due(.9)
    assert attempt.due(1.0)
    attempt.complete(False)
    assert attempt.due(.5)
    for index in range(1, 5):
        attempt.begin(index)
        attempt.complete(False)
    assert not attempt.due(100) and not attempt.succeeded
    attempt = Cancellation()
    attempt.begin(0)
    attempt.complete(True)
    assert not attempt.due(100) and attempt.succeeded


def test_actual_steering_arc_is_checked_before_braking():
    from wheeltec_vision_nav.runtime_safety import normalized_command
    controller, now = ready()
    obstacle = [(.35, -.2, .2)]
    requested = (.3, 1.)
    checked = normalized_command(controller.cfg, *requested)
    assert checked == pytest.approx((.3, .5))
    assert math.isinf(swept_clearance(obstacle, controller.cfg, *requested))
    clearance = swept_clearance(obstacle, controller.cfg, *checked)
    assert clearance <= controller.cfg.collision_margin
    assert drive(controller, now, *requested, clearance_m=clearance).vx == 0


@pytest.mark.parametrize("requested", [(.3, 1.), (1., 8.), (.3, -.9)])
def test_braking_retains_the_normalized_swept_arc(requested):
    from wheeltec_vision_nav.runtime_safety import normalized_command
    controller, now = ready()
    vx, wz = normalized_command(controller.cfg, *requested)
    result = drive(controller, now, *requested, clearance_m=.12, observation_age_s=.2)
    assert 0 < result.vx < vx
    assert result.wz / result.vx == pytest.approx(wz / vx)


def test_advancing_localization_and_odom_do_not_hide_frozen_map_transform():
    controller, now = ready()
    original = controller.sources["map_tf"].stamp_ns
    for index in range(1, 53):
        current = now + index * .1
        feed(controller, current, omit=("map_tf",))
        controller.note("map_tf", original, current, ros_now_ns=int((1000 + current) * 1e9))
        result = drive(controller, current)
    assert result.state == STOPPED and result.vx == 0
    assert result.reason == "map_tf_invalid_or_stale"


@pytest.mark.parametrize("axis", ["x", "y", "z"])
@pytest.mark.parametrize("bad", [math.nan, math.inf, -math.inf])
def test_all_nonfinite_transform_translations_rejected_even_for_empty_geometry(axis, bad):
    from types import SimpleNamespace as NS
    from wheeltec_vision_nav.runtime_safety import transform_points, transform_valid
    transform = NS(translation=NS(x=0., y=0., z=0.), rotation=NS(x=0., y=0., z=0., w=1.))
    setattr(transform.translation, axis, bad)
    assert not transform_valid(transform)
    for cloud in ([], [(1., 0., 0.)]):
        with pytest.raises(ValueError):
            transform_points(cloud, transform)


def test_transform_rejects_nonfinite_inputs_and_overflowed_outputs():
    from types import SimpleNamespace as NS
    from wheeltec_vision_nav.runtime_safety import transform_points
    transform = NS(translation=NS(x=1e308, y=0., z=0.), rotation=NS(x=0., y=0., z=0., w=1.))
    for cloud in ([(math.inf, 0., 0.)], [(1e308, 0., 0.)]):
        with pytest.raises(ValueError):
            transform_points(cloud, transform)


def test_goal_acceptance_must_follow_completed_recovery_and_cancellation():
    from wheeltec_vision_nav.runtime_safety import GoalAcceptanceBoundary
    boundary = GoalAcceptanceBoundary()
    boundary.update(False, 100)
    boundary.update(False, 200)  # Recovery complete, cancellation still pending.
    boundary.update(True, 300)
    assert not boundary.accepts(90, 400)  # Previously unseen delayed old goal.
    assert not boundary.accepts(250, 400)  # Accepted while cancelling.
    assert not boundary.accepts(300, 400)
    assert not boundary.accepts(500, 400)  # Future/invalid acceptance.
    assert boundary.accepts(301, 400)
    boundary.update(False, 500)
    assert not boundary.accepts(501, 600)


def test_obstacle_transport_and_residence_age_controls_braking_with_fresh_cloud():
    controller, now = ready()
    controller.sources.pop("obstacles")
    # 200 ms segmentation/transport, then 90 ms held while raw clouds advance.
    stamp = int((1000 + now - .2) * 1e9)
    controller.note("obstacles", stamp, now, ros_now_ns=int((1000 + now) * 1e9))
    current = now + .09
    feed(controller, current, omit=("obstacles",))
    age = controller.geometry_age(current)
    assert age == pytest.approx(.29)
    assert controller.sources["obstacles"].fresh(current, .3)
    optimistic = drive(controller, current, clearance_m=.15)
    protected = drive(controller, current, clearance_m=.15, observation_age_s=age)
    assert 0 < protected.vx < optimistic.vx


@pytest.mark.parametrize("wz", [-.5, .5])
def test_straight_returns_do_not_authorize_blind_lateral_sweep(wz):
    cfg = SafetyConfig()
    narrow = [(x, y, 0.) for x in (1., 1.5, 2.) for y in (-.2, 0., .2)]
    assert observed_forward_range(narrow, cfg, .3, 0.) > 1.
    assert observed_forward_range(narrow, cfg, .3, wz) == 0.
    wide = [(x, y / 100., 0.) for x in (1., 1.5, 2.) for y in range(-200, 201, 5)]
    assert observed_forward_range(wide, cfg, .3, wz) > .3
