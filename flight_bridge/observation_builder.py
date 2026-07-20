"""Build temporary Phase 0 observations from structured flight state."""

from __future__ import annotations

import math

from flight_bridge.mavlink_adapter import FlightState


class ObservationBuilder:
    """Build an 8-dimensional temporary Rover observation vector."""

    _RANGES: tuple[tuple[float, float], ...] = (
        (-math.pi, math.pi),
        (-math.pi, math.pi),
        (-math.pi, math.pi),
        (-2.0 * math.pi, 2.0 * math.pi),
        (-2.0 * math.pi, 2.0 * math.pi),
        (-2.0 * math.pi, 2.0 * math.pi),
        (-1.0, 1.0),
        (0.0, 15.0),
    )

    def build(self, state: FlightState) -> list[float]:
        heading_error_sin = math.sin(math.radians(state.heading_error_deg))
        raw_values = (
            state.roll,
            state.pitch,
            state.yaw,
            state.roll_speed,
            state.pitch_speed,
            state.yaw_speed,
            heading_error_sin,
            state.groundspeed,
        )
        return [
            self._normalize(value, lower, upper)
            for value, (lower, upper) in zip(raw_values, self._RANGES)
        ]

    @staticmethod
    def _normalize(value: float, lower: float, upper: float) -> float:
        if upper <= lower:
            raise ValueError(f"Invalid normalization range: lower={lower}, upper={upper}")

        normalized = (2.0 * (value - lower) / (upper - lower)) - 1.0
        return max(-1.0, min(1.0, normalized))


def build_feedback(state: FlightState) -> list[float]:
    """Build the temporary one-value feedback vector for Phase 0."""

    return [min(1.0, abs(state.heading_error_deg) / 180.0)]


def wrap_deg(angle_deg: float) -> float:
    """Wrap an angle in degrees to [-180, 180)."""

    return (angle_deg + 180.0) % 360.0 - 180.0

