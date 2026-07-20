"""Shared command and info types used across the flight bridge modules.

VelocityCmd is the canonical control command flowing from ActionAdapter
through SafetyShield to MavlinkAdapter.  All velocities are body-FLU,
all angles in radians unless *_deg is explicit.
"""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class VelocityCmd:
    """Body-FLU velocity + yaw-rate command.

    vx, vy, vz in m/s (body-FLU: x=forward, y=left, z=up).
    yaw_rate in rad/s.
    """

    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw_rate: float = 0.0

    def to_list(self) -> list[float]:
        return [self.vx, self.vy, self.vz, self.yaw_rate]

    @classmethod
    def hover(cls) -> "VelocityCmd":
        return cls(0.0, 0.0, 0.0, 0.0)


@dataclass
class RcCommand:
    """RC override PWM values (1000-2000, 1500=neutral).

    For throttle, neutral is hover_throttle (typically ~450 in ArduCopter 0-1000
    scale, mapped to ~1450 PWM). Actual mapping depends on config.copter.

    Use `RcCommand.neutral(hover_throttle_pwm)` to get the correct neutral
    with hover throttle; the no-arg `neutral()` default exists for emergency
    fallback only (throttle=1500 is NOT hover for most copters).
    """

    roll: int = 1500      # PWM
    pitch: int = 1500
    throttle: int = 1000   # PWM (neutral depends on hover_throttle)
    yaw: int = 1500

    def to_list(self) -> list[int]:
        return [self.roll, self.pitch, self.throttle, self.yaw]

    @classmethod
    def neutral(cls, hover_throttle_pwm: int = 1500) -> "RcCommand":
        """RC neutral — pass hover_throttle_pwm (e.g. hover_throttle + 1000)."""
        return cls(1500, 1500, hover_throttle_pwm, 1500)


@dataclass
class ShieldInfo:
    """Metadata produced by SafetyShield.filter() for logging and feedback."""

    saturated: bool = False
    safety_triggered: bool = False
    clipped_fields: list[str] = field(default_factory=list)
    filtered: bool = False
    in_deadband: bool = False
