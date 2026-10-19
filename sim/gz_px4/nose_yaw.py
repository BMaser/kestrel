"""Yaw plan that keeps the nose of the drone (and so its camera) pointed along the direction of flight.

Pure functions, numpy only. The drone camera looks along the body x axis (the nose); see
`nose_vs_velocity_deg` and the notes in `controllers/nose_forward_controller.py`.

Plan in three steps:
  1. `path_yaw`      the heading of the horizontal velocity, continuous (no jump at +pi / -pi),
                     held where the drone is (nearly) standing still;
  2. `limit_backward` limits the rate from the end to the start, so a turn STARTS EARLY and the nose is never late;
  3. `limit_forward`  limits the rate from the start to the end, from the given start yaw, so the plan starts at the
                     start yaw and never turns faster than the allowed rate.
`plan_yaw` runs the three steps in this order.
"""

from __future__ import annotations

import math

import numpy as np

TWO_PI = 2.0 * math.pi


def _check_rate(rate_max: float, dt: float) -> float:
    if not (dt > 0.0):
        raise ValueError(f"dt must be positive, got {dt!r}")
    if not (rate_max >= 0.0):
        raise ValueError(f"rate_max must not be negative, got {rate_max!r}")
    return float(rate_max) * float(dt)


def wrap_pi(angle):
    """Angle (scalar or array) mapped to [-pi, pi)."""
    return (np.asarray(angle, dtype=float) + math.pi) % TWO_PI - math.pi


def path_yaw(vel_xy, v_min: float, start_yaw: float) -> np.ndarray:
    """Heading atan2(vy, vx) of the horizontal velocity, one value per sample, as a continuous series.

    A sample whose horizontal speed is at least `v_min` takes the angle atan2(vy, vx) plus the multiple of 2*pi that puts
    it closest to the sample before it. A sample below `v_min` repeats the sample before it. Leading samples below
    `v_min` take `start_yaw`. A velocity that is not a number counts as below `v_min`.
    """
    v = np.asarray(vel_xy, dtype=float).reshape(-1, 2)
    speed = np.hypot(v[:, 0], v[:, 1])
    out = np.empty(len(v), dtype=float)
    previous = float(start_yaw)
    for k in range(len(v)):
        if speed[k] >= v_min:
            raw = math.atan2(v[k, 1], v[k, 0])
            previous = raw + TWO_PI * round((previous - raw) / TWO_PI)
        out[k] = previous
    return out


def limit_backward(yaw, rate_max: float, dt: float) -> np.ndarray:
    """From the last sample to the first: out[k] = clip(yaw[k], out[k+1] - rate_max*dt, out[k+1] + rate_max*dt)."""
    step = _check_rate(rate_max, dt)
    yaw = np.asarray(yaw, dtype=float)
    out = yaw.copy()
    for k in range(len(yaw) - 2, -1, -1):
        out[k] = min(max(yaw[k], out[k + 1] - step), out[k + 1] + step)
    return out


def limit_forward(yaw, rate_max: float, dt: float, start: float) -> np.ndarray:
    """From the first sample to the last: out[0] = start, out[k] = clip(yaw[k], out[k-1] -+ rate_max*dt)."""
    step = _check_rate(rate_max, dt)
    yaw = np.asarray(yaw, dtype=float)
    out = yaw.copy()
    if len(out) == 0:
        return out
    out[0] = float(start)
    for k in range(1, len(yaw)):
        out[k] = min(max(yaw[k], out[k - 1] - step), out[k - 1] + step)
    return out


def plan_yaw(vel_xy, dt: float, rate_max: float, v_min: float, start_yaw: float) -> np.ndarray:
    """The yaw plan: limit_forward(limit_backward(path_yaw(...)), start=start_yaw).

    Properties (the tests check them): plan[0] == start_yaw, and |plan[k] - plan[k-1]| <= rate_max*dt + 1e-12.
    """
    path = path_yaw(vel_xy, v_min, start_yaw)
    early = limit_backward(path, rate_max, dt)
    return limit_forward(early, rate_max, dt, start_yaw)


def nose_vs_velocity_deg(yaw_actual, vel_xy) -> np.ndarray:
    """Unsigned angle in degrees (0 to 180) between the nose and the horizontal velocity.

    `yaw_actual` is the heading of the nose in rad (any multiple of 2*pi), one value per sample. Where the horizontal
    velocity is exactly zero the angle is undefined and the result is nan; the caller selects the samples by speed.
    """
    v = np.asarray(vel_xy, dtype=float).reshape(-1, 2)
    yaw = np.asarray(yaw_actual, dtype=float).reshape(-1)
    heading = np.arctan2(v[:, 1], v[:, 0])
    angle = np.degrees(np.abs(wrap_pi(heading - yaw)))
    angle = np.where(np.hypot(v[:, 0], v[:, 1]) > 0.0, angle, np.nan)
    return angle


def yaw_from_quat_xyzw(quat) -> np.ndarray:
    """Heading of the body x axis (the nose) in the horizontal plane, from quaternions in the order x, y, z, w.

    Body x axis in the world = first column of the rotation matrix = (1 - 2(y^2 + z^2), 2(xy + zw), 2(xz - yw)).
    Accepts one quaternion (4,) or many (n, 4).
    """
    q = np.asarray(quat, dtype=float)
    single = q.ndim == 1
    q = q.reshape(-1, 4)
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    heading = np.arctan2(2.0 * (x * y + z * w), 1.0 - 2.0 * (y * y + z * z))
    return heading[0] if single else heading
