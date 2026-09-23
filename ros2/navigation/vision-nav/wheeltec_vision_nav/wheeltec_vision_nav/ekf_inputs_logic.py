"""Prepare gyro and visual-odometry samples for the EKF. No ROS imports.

robot_localization takes each measurement's covariance from the message. Two
inputs need correcting before they reach it:

- The chassis gyro has a constant yaw-rate bias (about 8e-4 rad/s on this
  robot) and reports a variance of 1e-6, so the EKF integrates the bias into a
  steady heading drift. The bias is estimated while the wheels report
  standstill and subtracted.
- rgbd_odometry reports twist variances around 1e-7, far smaller than its real
  error, which would let a single mis-registered frame override wheel speed
  and gyro. A lower bound is applied.
"""

from __future__ import annotations

from dataclasses import dataclass

STILL_LINEAR = 1e-3
STILL_ANGULAR = 1e-3


@dataclass
class GyroBias:
    """Yaw-rate bias estimated from gyro samples taken while the wheels stand still."""

    settle_s: float = 0.5
    alpha: float = 0.02
    bias: float = 0.0
    samples: int = 0
    _still_since: float | None = None

    def note_wheel(self, stamp: float, vx: float, vyaw: float) -> None:
        if abs(vx) < STILL_LINEAR and abs(vyaw) < STILL_ANGULAR:
            if self._still_since is None:
                self._still_since = stamp
        else:
            self._still_since = None

    def is_still(self, stamp: float) -> bool:
        return self._still_since is not None and stamp - self._still_since >= self.settle_s

    def correct(self, stamp: float, gyro_z: float) -> float:
        """Return the bias-corrected yaw rate; learn the bias while standing still."""
        if self.is_still(stamp):
            if self.samples == 0:
                self.bias = gyro_z
            else:
                self.bias += self.alpha * (gyro_z - self.bias)
            self.samples += 1
        return gyro_z - self.bias


# Row-major 6x6 twist covariance diagonal indices.
VX, VY, VYAW = 0, 7, 35


def floor_twist_covariance(covariance, floors: dict[int, float]) -> list[float]:
    """Raise selected diagonal entries to at least the given variance."""
    out = list(covariance)
    for index, floor in floors.items():
        if not out[index] >= floor:
            out[index] = floor
    return out
