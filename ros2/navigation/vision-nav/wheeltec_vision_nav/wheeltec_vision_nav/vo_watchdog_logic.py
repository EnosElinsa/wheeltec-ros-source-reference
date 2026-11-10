"""Legacy sample classifier retained for compatibility and tests.

should_zero_cmd is not runtime motion authorization; runtime_safety owns that.
No ROS imports.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

STATUS_OK = "ok"
STATUS_LOST = "lost"
STATUS_RECOVERING = "recovering"


def quaternion_valid(x: float, y: float, z: float, w: float) -> bool:
    return all(math.isfinite(v) for v in (x, y, z, w)) and 0.98 <= x*x + y*y + z*z + w*w <= 1.02


@dataclass
class WatchdogConfig:
    min_inliers: int = 20
    consecutive_bad: int = 5
    # When True, a VO loss alone does not stop the car as long as wheel
    # odometry keeps /odom alive and the map correction is still fresh.
    wheel_backup: bool = False
    wheel_fresh_s: float = 0.3
    map_stale_s: float = 5.0


@dataclass
class WatchdogState:
    consecutive_bad: int = 0
    lost: bool = False
    status: str = STATUS_OK
    last_wheel_time: float = -1.0
    last_map_time: float = -1.0


def evaluate_sample(
    state: WatchdogState,
    cfg: WatchdogConfig,
    *,
    lost_flag: bool | None = None,
    inliers: int | None = None,
    quat: tuple[float, float, float, float] | None = None,
) -> WatchdogState:
    invalid_quat = quat is not None and not quaternion_valid(*quat)
    if invalid_quat:
        state.consecutive_bad += 1
        state.lost = True
        state.status = STATUS_LOST
        return state

    has_vo = lost_flag is not None or inliers is not None
    if not has_vo:
        return state

    flagged_lost = lost_flag is True
    low_inliers = inliers is not None and inliers < cfg.min_inliers
    if flagged_lost or low_inliers:
        state.consecutive_bad += 1
        if flagged_lost or state.consecutive_bad >= cfg.consecutive_bad:
            state.lost = True
            state.status = STATUS_LOST
        return state

    state.consecutive_bad = 0
    if state.lost or state.status == STATUS_LOST:
        state.lost = False
        state.status = STATUS_RECOVERING
        return state
    state.status = STATUS_OK
    return state


def note_wheel(state: WatchdogState, now: float) -> None:
    state.last_wheel_time = now


def note_map_correction(state: WatchdogState, now: float) -> None:
    state.last_map_time = now


def wheel_fresh(state: WatchdogState, cfg: WatchdogConfig, now: float) -> bool:
    return state.last_wheel_time >= 0.0 and (now - state.last_wheel_time) < cfg.wheel_fresh_s


def map_stale(state: WatchdogState, cfg: WatchdogConfig, now: float) -> bool:
    """True when RTAB-Map has not localized for map_stale_s.

    Before the first localization arrives the map is considered stale, so the
    car does not drive on a pose that was never anchored to the map.
    """
    return state.last_map_time < 0.0 or (now - state.last_map_time) > cfg.map_stale_s


def command_is_stale(last_cmd_time: float, now: float, timeout_s: float) -> bool:
    """True when no velocity command has arrived for timeout_s (or none ever)."""
    return last_cmd_time < 0.0 or (now - last_cmd_time) > timeout_s


def vo_lost(state: WatchdogState) -> bool:
    return state.lost or state.status == STATUS_LOST


def should_zero_cmd(state: WatchdogState, cfg: WatchdogConfig | None = None, now: float | None = None) -> bool:
    """Decide whether the command to the chassis must be zeroed.

    Without wheel backup (chassis not started, or legacy call without cfg):
    stop as soon as VO is lost.
    With wheel backup: while VO is lost, keep driving on wheel odometry as
    long as the wheel stream is fresh and map->odom is not stale.
    """
    if not vo_lost(state):
        return False
    if cfg is None or now is None or not cfg.wheel_backup:
        return True
    if not wheel_fresh(state, cfg, now):
        return True
    return map_stale(state, cfg, now)
