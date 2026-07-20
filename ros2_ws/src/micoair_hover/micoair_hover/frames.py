"""Coordinate transforms at the PX4 NED/FRD to SDK FLU boundary.

PX4 setpoints and local position use NED.  The shared SDK bridge represents
body motion as FLU.  Keeping these transforms in one module prevents a
controller from silently mixing the two conventions.
"""

from __future__ import annotations

import math


def wrap_pi(angle_rad: float) -> float:
    """Wrap an angle to the interval [-pi, pi)."""
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def body_flu_to_world_ned(
    forward_mps: float,
    left_mps: float,
    up_mps: float,
    yaw_ned_rad: float,
) -> tuple[float, float, float]:
    """Transform a body-FLU velocity into a local NED velocity."""
    cos_yaw = math.cos(yaw_ned_rad)
    sin_yaw = math.sin(yaw_ned_rad)
    north_mps = cos_yaw * forward_mps + sin_yaw * left_mps
    east_mps = sin_yaw * forward_mps - cos_yaw * left_mps
    down_mps = -up_mps
    return north_mps, east_mps, down_mps


def world_ned_to_body_flu(
    north_mps: float,
    east_mps: float,
    down_mps: float,
    yaw_ned_rad: float,
) -> tuple[float, float, float]:
    """Transform a local NED vector into a body-FLU vector."""
    cos_yaw = math.cos(yaw_ned_rad)
    sin_yaw = math.sin(yaw_ned_rad)
    forward_mps = cos_yaw * north_mps + sin_yaw * east_mps
    right_mps = -sin_yaw * north_mps + cos_yaw * east_mps
    return forward_mps, -right_mps, -down_mps


def quaternion_frd_to_ned_to_euler(
    quaternion_wxyz: tuple[float, float, float, float],
) -> tuple[float, float, float]:
    """Return FRD/NED roll, pitch, and yaw from PX4's Hamilton quaternion."""
    w, x, y, z = quaternion_wxyz
    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll_rad = math.atan2(sin_roll, cos_roll)

    sin_pitch = 2.0 * (w * y - z * x)
    pitch_rad = math.asin(max(-1.0, min(1.0, sin_pitch)))

    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    yaw_rad = math.atan2(sin_yaw, cos_yaw)
    return roll_rad, pitch_rad, yaw_rad


def frd_rates_to_flu(
    roll_rate_rad_s: float,
    pitch_rate_rad_s: float,
    yaw_rate_rad_s: float,
) -> tuple[float, float, float]:
    """Convert body rates from FRD to FLU coordinates."""
    return roll_rate_rad_s, -pitch_rate_rad_s, -yaw_rate_rad_s
