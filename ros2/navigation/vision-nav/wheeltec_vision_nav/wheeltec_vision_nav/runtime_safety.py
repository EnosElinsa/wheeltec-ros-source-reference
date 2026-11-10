"""Fail-closed navigation health and command limits, independent of ROS.

Receipt times are monotonic seconds; sensor stamps are integer ROS nanoseconds.
Visual quality detection is shared with the existing VINS A/B/C evaluator.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from wheeltec_vision_nav.vins_metrics import VisualHealth

INITIALIZING = "INITIALIZING"
READY = "READY"
DEGRADED = "DEGRADED"
STOPPED = "STOPPED"
RECOVERING = "RECOVERING"


def quaternion_valid(x: float, y: float, z: float, w: float) -> bool:
    values = (x, y, z, w)
    return all(math.isfinite(v) for v in values) and 0.98 <= sum(v * v for v in values) <= 1.02


def transform_valid(transform) -> bool:
    p, q = transform.translation, transform.rotation
    return (all(math.isfinite(v) for v in (p.x, p.y, p.z))
            and quaternion_valid(q.x, q.y, q.z, q.w))


def transform_points(points, transform):
    """Reject invalid geometry even for an empty obstacle cloud."""
    if not transform_valid(transform):
        raise ValueError("invalid transform")
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    if not np.isfinite(points).all():
        raise ValueError("nonfinite input points")
    q, p = transform.rotation, transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    rotation = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])
    with np.errstate(over="ignore", invalid="ignore"):
        result = points @ rotation.T + np.array([p.x, p.y, p.z])
    if not np.isfinite(result).all():
        raise ValueError("nonfinite transformed points")
    return result


def covariance_valid(values, indices=(0, 7, 35)) -> bool:
    if len(values) != 36 or not all(math.isfinite(v) for v in values):
        return False
    dimensions = [index // 6 for index in indices]
    planar = np.asarray(values).reshape(6, 6)[np.ix_(dimensions, dimensions)]
    return (all(values[i] >= 0.0 for i in indices)
            and np.allclose(planar, planar.T, atol=1e-9, rtol=1e-6)
            and float(np.linalg.eigvalsh(planar).min()) >= -1e-9)


def command_is_stale(last_cmd_time: float, now: float, timeout_s: float) -> bool:
    return last_cmd_time < 0.0 or not 0.0 <= now - last_cmd_time < timeout_s


@dataclass(frozen=True)
class SafetyConfig:
    mode: str = "navigation"
    wheel_backup: bool = False
    require_depth: bool = True
    min_inliers: int = 10
    vo_timeout_s: float = 0.5
    wheel_fresh_s: float = 0.3
    sensor_timeout_s: float = 0.3
    map_stale_s: float = 5.0
    cmd_timeout_s: float = 0.5
    future_tolerance_s: float = 0.1
    recovery_s: float = 1.0
    recovery_samples: int = 10
    degraded_s: float = 1.0
    degraded_m: float = 0.10
    degraded_speed: float = 0.10
    supervised_reverse: bool = False
    reverse_speed: float = 0.08
    reverse_m: float = 0.25
    reverse_s: float = 3.2
    max_speed: float = 0.3
    max_yaw_rate: float = 1.0
    min_turning_radius: float = 0.6
    braking_deceleration: float = 0.3
    reaction_s: float = 0.15
    footprint_front: float = 0.25
    footprint_rear: float = 0.25
    footprint_half_width: float = 0.20
    collision_margin: float = 0.05

    def __post_init__(self):
        if self.mode not in ("mapping", "navigation"):
            raise ValueError("mode must be mapping or navigation")
        for name, value in vars(self).items():
            if isinstance(value, (int, float)) and not isinstance(value, bool):
                if not math.isfinite(value) or value <= 0:
                    raise ValueError(f"{name} must be finite and positive")
        if self.degraded_speed > self.max_speed or self.reverse_speed > self.max_speed:
            raise ValueError("degraded/reverse speed must not exceed max_speed")


@dataclass
class Evidence:
    stamp_ns: int = -1
    received: float = -1.0
    valid: bool = False
    rejected: int = 0
    initial_age_s: float = 0.0

    def fresh(self, now: float, timeout: float) -> bool:
        return (self.valid and self.received >= 0.0 and now >= self.received
                and now - self.received + self.initial_age_s < timeout)


@dataclass(frozen=True)
class Decision:
    state: str
    reason: str
    vx: float = 0.0
    wz: float = 0.0
    ready: bool = False
    new_goal_required: bool = False
    cancel_required: bool = False


@dataclass
class Cancellation:
    """Bounded asynchronous retry policy; late/failed replies grant no motion."""
    attempts: int = 0
    succeeded: bool = False
    pending: bool = False
    next_attempt: float = 0.0
    retry_s: float = 0.5
    response_timeout_s: float = 1.0
    max_attempts: int = 5

    def due(self, now: float) -> bool:
        return not self.succeeded and self.attempts < self.max_attempts and now >= self.next_attempt

    def begin(self, now: float):
        self.attempts += 1
        self.pending = True
        self.next_attempt = now + self.response_timeout_s

    def complete(self, succeeded: bool):
        self.succeeded, self.pending = bool(succeeded), False
        if not succeeded:
            self.next_attempt -= self.response_timeout_s - self.retry_s


@dataclass
class GoalAcceptanceBoundary:
    """Only goals accepted after recovery AND completed cancellation may rearm."""
    after_ns: int = 0
    eligible: bool = False

    def update(self, eligible: bool, ros_now_ns: int):
        if not eligible or not self.eligible:
            self.after_ns = ros_now_ns
        self.eligible = eligible

    def accepts(self, accepted_ns: int, ros_now_ns: int) -> bool:
        return self.eligible and 0 < self.after_ns < accepted_ns <= ros_now_ns


def normalized_command(cfg: SafetyConfig, vx: float, wz: float):
    """Canonical steering arc; every later speed reduction preserves this arc."""
    speed = min(abs(vx), cfg.max_speed)
    wz = wz * speed / abs(vx) if abs(vx) > 1e-9 else 0.0
    limit = min(cfg.max_yaw_rate, speed / cfg.min_turning_radius)
    return math.copysign(speed, vx), max(-limit, min(limit, wz))


class SafetyController:
    def __init__(self, cfg: SafetyConfig):
        self.cfg = cfg
        self.sources: dict[str, Evidence] = {}
        self.visual = VisualHealth(cfg.vo_timeout_s, cfg.min_inliers)
        self.state = INITIALIZING
        self.reason = "waiting_for_sensors"
        self.ever_ready = False
        self.new_goal_required = False
        self.cancel_required = False
        self._healthy_since: float | None = None
        self._healthy_samples = 0
        self._degraded_since: float | None = None
        self._degraded_distance = 0.0
        self._reverse_since: float | None = None
        self._reverse_distance = 0.0
        self._last_step: float | None = None
        self._wheel_speed = 0.0
        self._cmd = (0.0, 0.0)
        self._last_cmd_time = -1.0
        self._distance_time: float | None = None
        self._last_visual_stamp = -1

    def timeout(self, source: str) -> float:
        if source in ("vo", "visual"):
            return self.cfg.vo_timeout_s
        if source == "wheel":
            return self.cfg.wheel_fresh_s
        if source in ("localization", "map_tf"):
            return self.cfg.map_stale_s
        return self.cfg.sensor_timeout_s

    def note(self, source: str, stamp_ns: int, now: float, *, valid=True,
             ros_now_ns: int | None = None) -> bool:
        item = self.sources.setdefault(source, Evidence())
        if not math.isfinite(now) or not isinstance(stamp_ns, int) or stamp_ns <= 0:
            valid = False
        age = 0.0
        if ros_now_ns is not None:
            age = (ros_now_ns - stamp_ns) / 1e9
            valid = valid and -self.cfg.future_tolerance_s <= age < self.timeout(source)
        if not valid:
            item.valid = False
            item.rejected += 1
            return False
        if stamp_ns <= item.stamp_ns:
            item.rejected += 1
            if stamp_ns < item.stamp_ns:
                item.valid = False
            return False
        item.stamp_ns, item.received, item.valid = stamp_ns, now, True
        item.initial_age_s = max(0.0, age)
        return True

    def note_visual(self, stamp_ns: int, now: float, lost: bool, inliers: int,
                    ros_now_ns: int | None = None):
        if not self.note("visual", stamp_ns, now, ros_now_ns=ros_now_ns):
            return
        self.visual.note_visual(stamp_ns, lost, inliers, now)
        if not lost and inliers >= self.cfg.min_inliers:
            if stamp_ns > self._last_visual_stamp:
                self._healthy_samples += 1
        else:
            self._healthy_since = None
            self._healthy_samples = 0
        self._last_visual_stamp = stamp_ns

    def note_wheel(self, stamp_ns: int, now: float, vx: float, **kwargs):
        if self.note("wheel", stamp_ns, now, valid=math.isfinite(vx), **kwargs):
            if self._distance_time is not None:
                dt = max(0.0, min(now - self._distance_time, self.cfg.wheel_fresh_s))
                distance = max(abs(vx), abs(self._wheel_speed)) * dt
                if self._degraded_since is not None:
                    self._degraded_distance += distance
                if self._reverse_since is not None:
                    self._reverse_distance += distance
            self._wheel_speed = vx
            self._distance_time = now

    def note_command(self, vx: float, wz: float, now: float):
        if not all(math.isfinite(v) for v in (vx, wz, now)):
            self.stop("invalid_command")
            return
        if self.state not in (READY, DEGRADED) or self.new_goal_required:
            self._cmd, self._last_cmd_time = (0.0, 0.0), -1.0
            return
        self._cmd, self._last_cmd_time = (vx, wz), now

    def stop(self, reason: str):
        self.state, self.reason = STOPPED, reason
        self.new_goal_required = self.ever_ready
        self.cancel_required = self.ever_ready and self.cfg.mode == "navigation"
        self._healthy_since, self._healthy_samples = None, 0
        self._degraded_since, self._degraded_distance = None, 0.0
        self._reverse_since, self._reverse_distance = None, 0.0
        self._cmd, self._last_cmd_time = (0.0, 0.0), -1.0

    def clock_reset(self):
        self.stop("ros_clock_jump")
        self.sources.clear()
        self.visual = VisualHealth(self.cfg.vo_timeout_s, self.cfg.min_inliers)
        self._last_visual_stamp = -1
        self._distance_time = None

    def rearm(self) -> bool:
        """Called for a new accepted goal, or explicit mapping operator rearm."""
        if self.state != READY or self.cancel_required:
            return False
        self.new_goal_required = False
        self._cmd, self._last_cmd_time = (0.0, 0.0), -1.0
        return True

    def _required(self):
        required = ["odom", "tf", "ownership"]
        if self.cfg.require_depth:
            required += ["depth", "cloud", "obstacles"]
        if self.cfg.wheel_backup or self.cfg.supervised_reverse:
            required += ["wheel"]
        if self.cfg.wheel_backup:
            required += ["imu"]
        if self.cfg.mode == "navigation":
            required += ["localization", "map_tf"]
        return required

    def geometry_age(self, now: float) -> float:
        """Budget transport plus residence time of every geometry input."""
        names = ("depth", "cloud", "obstacles", "tf")
        return max((max(0.0, now - self.sources[name].received)
                    + self.sources[name].initial_age_s for name in names
                    if name in self.sources), default=0.0)

    def _critical_reason(self, now: float):
        for name in self._required():
            if not self.sources.get(name, Evidence()).fresh(now, self.timeout(name)):
                return f"{name}_invalid_or_stale"
        return None

    def step(self, now: float, *, clearance_m=math.inf, observed_range_m=0.0,
             observation_age_s=0.0) -> Decision:
        if self._last_step is not None and now < self._last_step:
            self.stop("monotonic_clock_reversed")
        self._last_step = now
        critical = self._critical_reason(now)
        vo_ok = self.sources.get("vo", Evidence()).fresh(now, self.cfg.vo_timeout_s)
        visual_ok = self.sources.get("visual", Evidence()).fresh(now, self.cfg.vo_timeout_s)
        visual_ok = visual_ok and self.visual.status(now) == "tracking"
        healthy = critical is None and vo_ok and visual_ok
        if critical:
            if self.ever_ready:
                self.stop(critical)
            else:
                self.state, self.reason = INITIALIZING, critical
                self._healthy_since, self._healthy_samples = None, 0
            return self._decision()
        if not healthy:
            self._healthy_since, self._healthy_samples = None, 0
            if not self.ever_ready:
                self.state, self.reason = INITIALIZING, "waiting_for_visual_tracking"
                return self._decision()
            if self.state in (STOPPED, RECOVERING):
                self.state, self.reason = STOPPED, "visual_invalid_or_stale"
                return self._decision()
            if not self.cfg.wheel_backup:
                self.stop("visual_invalid_or_stale")
                return self._decision()
            if self._degraded_since is None:
                self._degraded_since, self._degraded_distance = now, 0.0
            if (now - self._degraded_since >= self.cfg.degraded_s or
                    self._degraded_distance >= self.cfg.degraded_m):
                self.stop("degraded_budget_exhausted")
                return self._decision()
            self.state, self.reason = DEGRADED, "visual_invalid_or_stale"
        elif self.state != READY:
            if self._healthy_since is None:
                self._healthy_since, self._healthy_samples = now, 0
            self.state, self.reason = RECOVERING, "validating_recovery"
            if now - self._healthy_since >= self.cfg.recovery_s and self._healthy_samples >= self.cfg.recovery_samples:
                self.state, self.reason = READY, "healthy"
                self.ever_ready = True
                self._degraded_since, self._degraded_distance = None, 0.0
            else:
                if self._degraded_since is not None and (
                    now - self._degraded_since >= self.cfg.degraded_s or
                    self._degraded_distance >= self.cfg.degraded_m
                ):
                    self.stop("degraded_budget_exhausted")
                return self._decision()
        else:
            self.reason = "healthy"
        if self.new_goal_required:
            return self._decision(reason="waiting_for_new_goal")
        if command_is_stale(self._last_cmd_time, now, self.cfg.cmd_timeout_s):
            self._cmd = (0.0, 0.0)
            return self._decision(reason="command_idle_or_stale")
        vx, wz = normalized_command(self.cfg, *self._cmd)
        checked_vx = vx
        if vx < 0.0:
            if not self.cfg.supervised_reverse or self.state == DEGRADED:
                self.stop("reverse_not_authorized")
                return self._decision()
            if self._reverse_since is None:
                self._reverse_since, self._reverse_distance = now, 0.0
            if now - self._reverse_since >= self.cfg.reverse_s or self._reverse_distance >= self.cfg.reverse_m:
                self.stop("reverse_budget_exhausted")
                return self._decision()
            vx = max(vx, -self.cfg.reverse_speed)
        else:
            if self._wheel_speed >= -0.001:
                self._reverse_since, self._reverse_distance = None, 0.0
            vx = min(vx, self.cfg.degraded_speed if self.state == DEGRADED else self.cfg.max_speed)
        if vx > 0.0 and self.cfg.require_depth:
            distance = min(clearance_m, observed_range_m)
            if not math.isfinite(distance) or distance <= self.cfg.collision_margin:
                self.stop("forward_corridor_unobserved_or_blocked")
                return self._decision()
            reaction = self.cfg.reaction_s + max(0.0, observation_age_s)
            available = max(0.0, distance - self.cfg.collision_margin)
            a = self.cfg.braking_deceleration
            max_safe = math.sqrt((a * reaction) ** 2 + 2 * a * available) - a * reaction
            vx = min(vx, max_safe)
        if abs(checked_vx) > 1e-9:
            wz *= abs(vx / checked_vx)
        return self._decision(vx=vx, wz=wz)

    def _decision(self, *, vx=0.0, wz=0.0, reason=None):
        return Decision(self.state, reason or self.reason, vx, wz, self.state == READY,
                        self.new_goal_required, self.cancel_required)

    def summary(self, now: float):
        return {"state": self.state, "reason": self.reason, "ready": self.state == READY,
                "new_goal_required": self.new_goal_required,
                "cancel_required": self.cancel_required,
                "degraded_distance_m": self._degraded_distance,
                "reverse_distance_m": self._reverse_distance,
                "sources": {name: {"age_s": max(0.0, now - e.received) + e.initial_age_s if e.received >= 0 else None,
                                   "stamp_ns": e.stamp_ns, "valid": e.valid, "rejected": e.rejected}
                            for name, e in self.sources.items()}}


def swept_clearance(points, cfg: SafetyConfig, vx: float, wz: float, lookahead=3.0) -> float:
    """Distance to first obstacle in a sampled planar swept footprint."""
    cloud = np.asarray(points, dtype=float).reshape(-1, 3)
    obstacles = cloud[np.isfinite(cloud).all(axis=1) & (cloud[:, 2] >= .05) & (cloud[:, 2] <= 1.8)]
    if not len(obstacles):
        return math.inf
    curvature = wz / vx if abs(vx) > 1e-4 else 0.0
    for step in range(int(lookahead / 0.05) + 1):
        travel = step * 0.05
        theta = curvature * travel
        cx = math.sin(theta) / curvature if abs(curvature) > 1e-6 else travel
        cy = (1.0 - math.cos(theta)) / curvature if abs(curvature) > 1e-6 else 0.0
        c, s = math.cos(theta), math.sin(theta)
        dx, dy = obstacles[:, 0] - cx, obstacles[:, 1] - cy
        local_x, local_y = c * dx + s * dy, -s * dx + c * dy
        if np.any((local_x >= -cfg.footprint_rear) & (local_x <= cfg.footprint_front)
                  & (np.abs(local_y) <= cfg.footprint_half_width + cfg.collision_margin)):
            return max(0.0, travel - 0.05)
    return math.inf


def observed_forward_range(points, cfg: SafetyConfig, vx=0.0, wz=0.0) -> float:
    """Range supported across the commanded sweep, not just straight ahead.

    Each lateral strip needs at least three returns. For a turn, the full
    projected footprint must remain inside the supported strips at every
    sampled pose. Sparse returns straight ahead cannot authorize a blind turn.
    """
    half = cfg.footprint_half_width + cfg.collision_margin
    cloud = np.asarray(points, dtype=float).reshape(-1, 3)
    cloud = cloud[np.isfinite(cloud).all(axis=1)]
    cloud = cloud[(cloud[:, 0] > cfg.footprint_front) & (cloud[:, 0] <= 3.0)
                  & (np.abs(cloud[:, 1]) <= 3.0) & (cloud[:, 2] >= -.15) & (cloud[:, 2] <= 1.8)]
    curvature = wz / vx if abs(vx) > 1e-4 else 0.0
    if abs(curvature) > 1e-6:
        width = 2 * half / 3
        limits = {}
        bins = np.floor((cloud[:, 1] + half) / width).astype(int)
        for index in np.unique(bins):
            strip = cloud[bins == index]
            if len(strip) >= 3:
                limits[int(index)] = float(np.max(strip[:, 0]))
        corners = np.array([(x, y) for x in (-cfg.footprint_rear, cfg.footprint_front)
                            for y in (-half, half)])
        for step in range(61):
            travel = step * .05
            theta = curvature * travel
            c, s = math.cos(theta), math.sin(theta)
            footprint = corners @ np.array([[c, s], [-s, c]])
            footprint += [math.sin(theta) / curvature, (1 - math.cos(theta)) / curvature]
            # Once the path turns behind the present camera plane, visibility
            # cannot be inferred from forward returns.
            if c <= 0 or footprint[:, 0].max() < cfg.footprint_front:
                return max(0.0, travel - .05)
            first = math.floor((footprint[:, 1].min() + half + 1e-9) / width)
            last = math.floor((footprint[:, 1].max() + half - 1e-9) / width)
            far_x = float(footprint[:, 0].max())
            if any(limits.get(index, 0.0) < far_x for index in range(first, last + 1)):
                return max(0.0, travel - .05)
        return 3.0
    limits = []
    for index in range(3):
        strip = cloud[(cloud[:, 1] >= -half + index * 2 * half / 3)
                      & (cloud[:, 1] <= -half + (index + 1) * 2 * half / 3)]
        if len(strip) < 3:
            return 0.0
        limits.append(float(np.max(strip[:, 0])))
    return max(0.0, min(limits) - cfg.footprint_front)
