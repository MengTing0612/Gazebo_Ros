"""ROS-independent state and setpoint helpers for PX4 hover control."""

from __future__ import annotations

import math
from dataclasses import dataclass

from flight_bridge.command_types import VelocityCmd
from flight_bridge.state_builder import CopterState

from .frames import world_ned_to_body_flu, wrap_pi


@dataclass(frozen=True)
class LocalNedState:
    """PX4 EKF local position and velocity in the NED frame."""

    north_m: float
    east_m: float
    down_m: float
    north_mps: float
    east_mps: float
    down_mps: float
    yaw_ned_rad: float
    roll_flu_rad: float = 0.0
    pitch_flu_rad: float = 0.0
    roll_rate_flu_rad_s: float = 0.0
    pitch_rate_flu_rad_s: float = 0.0
    yaw_rate_flu_rad_s: float = 0.0


@dataclass(frozen=True)
class HoverTargetNed:
    """Position target captured from the valid PX4 local estimate."""

    north_m: float
    east_m: float
    down_m: float
    yaw_ned_rad: float


@dataclass(frozen=True)
class SdkResidualSchedule:
    """Time-based attenuation for a bounded SDK velocity residual."""

    start_weight: float = 1.0
    hold_s: float = 2.0
    decay_s: float = 20.0
    final_weight: float = 0.0

    def __post_init__(self) -> None:
        values = {
            "start_weight": self.start_weight,
            "hold_s": self.hold_s,
            "decay_s": self.decay_s,
            "final_weight": self.final_weight,
        }
        if not all(math.isfinite(value) for value in values.values()):
            raise ValueError("SDK residual schedule values must be finite")
        if not 0.0 <= self.final_weight <= self.start_weight <= 1.0:
            raise ValueError("SDK residual weights must satisfy 0 <= final <= start <= 1")
        if self.hold_s < 0.0:
            raise ValueError("SDK residual hold_s must be non-negative")
        if self.decay_s <= 0.0:
            raise ValueError("SDK residual decay_s must be positive")

    def weight_at(self, elapsed_s: float) -> float:
        """Return a smooth cosine transition from start_weight to final_weight."""
        if not math.isfinite(elapsed_s) or elapsed_s < 0.0:
            raise ValueError("SDK residual elapsed_s must be finite and non-negative")
        if elapsed_s <= self.hold_s:
            return self.start_weight
        if elapsed_s >= self.hold_s + self.decay_s:
            return self.final_weight

        decay_progress = (elapsed_s - self.hold_s) / self.decay_s
        cosine_blend = 0.5 * (1.0 + math.cos(math.pi * decay_progress))
        return self.final_weight + (self.start_weight - self.final_weight) * cosine_blend


@dataclass(frozen=True)
class SitlVisualDisturbance:
    """Bounded, deterministic velocity feed-forward for a SITL-only visual demo.

    The command traces a small horizontal circle with a gentle vertical motion.
    It is an intentionally injected disturbance, not a learned SDK action.
    """

    hold_s: float = 8.0
    decay_s: float = 14.0
    horizontal_mps: float = 0.55
    vertical_mps: float = 0.12
    period_s: float = 7.0

    def __post_init__(self) -> None:
        values = {
            "hold_s": self.hold_s,
            "decay_s": self.decay_s,
            "horizontal_mps": self.horizontal_mps,
            "vertical_mps": self.vertical_mps,
            "period_s": self.period_s,
        }
        if not all(math.isfinite(value) for value in values.values()):
            raise ValueError("SITL visual demo values must be finite")
        if self.hold_s < 0.0 or self.decay_s <= 0.0 or self.period_s <= 0.0:
            raise ValueError("SITL visual demo timing values must be positive")
        if not 0.0 <= self.horizontal_mps <= 0.8:
            raise ValueError("SITL visual demo horizontal velocity must be within [0, 0.8]")
        if not 0.0 <= self.vertical_mps <= 0.25:
            raise ValueError("SITL visual demo vertical velocity must be within [0, 0.25]")

    def command_at(self, elapsed_s: float) -> VelocityCmd:
        """Return the body-FLU demo disturbance after smooth attenuation."""
        if not math.isfinite(elapsed_s) or elapsed_s < 0.0:
            raise ValueError("SITL visual demo elapsed_s must be finite and non-negative")

        attenuation = SdkResidualSchedule(
            start_weight=1.0,
            hold_s=self.hold_s,
            decay_s=self.decay_s,
            final_weight=0.0,
        ).weight_at(elapsed_s)
        phase = 2.0 * math.pi * elapsed_s / self.period_s
        return VelocityCmd(
            vx=attenuation * self.horizontal_mps * math.cos(phase),
            vy=attenuation * self.horizontal_mps * math.sin(phase),
            vz=attenuation * self.vertical_mps * math.sin(phase),
            yaw_rate=0.0,
        )

    def is_complete(self, elapsed_s: float) -> bool:
        """Return whether the injected disturbance has fully decayed."""
        return elapsed_s >= self.hold_s + self.decay_s


def capture_hover_target(state: LocalNedState, target_altitude_m: float) -> HoverTargetNed:
    """Create a takeoff-point-relative hover target at the requested altitude."""
    if target_altitude_m <= 0.0:
        raise ValueError("target_altitude_m must be positive")
    return HoverTargetNed(
        north_m=state.north_m,
        east_m=state.east_m,
        down_m=state.down_m - target_altitude_m,
        yaw_ned_rad=state.yaw_ned_rad,
    )


def build_sdk_state(state: LocalNedState, target: HoverTargetNed, armed: bool, mode: str) -> CopterState:
    """Translate PX4 NED/FRD state into the project's body-FLU SDK state."""
    target_error_ned = (
        target.north_m - state.north_m,
        target.east_m - state.east_m,
        target.down_m - state.down_m,
    )
    x_err, y_err, z_err = world_ned_to_body_flu(*target_error_ned, state.yaw_ned_rad)
    vx, vy, vz = world_ned_to_body_flu(
        state.north_mps,
        state.east_mps,
        state.down_mps,
        state.yaw_ned_rad,
    )

    # NED heading is clockwise from North.  The SDK's FLU yaw error follows
    # the ROS/ENU convention, which reverses that sign.
    yaw_err_flu_rad = wrap_pi(state.yaw_ned_rad - target.yaw_ned_rad)
    return CopterState(
        x_err=x_err,
        y_err=y_err,
        z_err=z_err,
        vx=vx,
        vy=vy,
        vz=vz,
        roll=state.roll_flu_rad,
        pitch=state.pitch_flu_rad,
        yaw_err=yaw_err_flu_rad,
        p=state.roll_rate_flu_rad_s,
        q=state.pitch_rate_flu_rad_s,
        r=state.yaw_rate_flu_rad_s,
        altitude_m=-state.down_m,
        armed=armed,
        mode=mode,
    )


def position_setpoint_velocity(
    residual_body_flu: VelocityCmd,
    max_horizontal_mps: float,
    max_vertical_mps: float,
    max_yaw_rate_rad_s: float,
) -> VelocityCmd:
    """Clamp an SDK residual before it reaches PX4 as velocity feed-forward."""
    if max_horizontal_mps <= 0.0 or max_vertical_mps <= 0.0 or max_yaw_rate_rad_s <= 0.0:
        raise ValueError("residual velocity limits must be positive")
    horizontal_scale = max(
        1.0,
        math.hypot(residual_body_flu.vx, residual_body_flu.vy) / max_horizontal_mps,
    )
    return VelocityCmd(
        vx=residual_body_flu.vx / horizontal_scale,
        vy=residual_body_flu.vy / horizontal_scale,
        vz=max(-max_vertical_mps, min(max_vertical_mps, residual_body_flu.vz)),
        yaw_rate=max(-max_yaw_rate_rad_s, min(max_yaw_rate_rad_s, residual_body_flu.yaw_rate)),
    )


def scale_velocity_command(command: VelocityCmd, weight: float) -> VelocityCmd:
    """Scale a body-FLU SDK residual without changing its coordinate frame."""
    if not math.isfinite(weight) or not 0.0 <= weight <= 1.0:
        raise ValueError("SDK residual weight must be finite and within [0, 1]")
    return VelocityCmd(
        vx=command.vx * weight,
        vy=command.vy * weight,
        vz=command.vz * weight,
        yaw_rate=command.yaw_rate * weight,
    )


def add_velocity_commands(*commands: VelocityCmd) -> VelocityCmd:
    """Combine body-FLU velocity feed-forward commands component by component."""
    return VelocityCmd(
        vx=sum(command.vx for command in commands),
        vy=sum(command.vy for command in commands),
        vz=sum(command.vz for command in commands),
        yaw_rate=sum(command.yaw_rate for command in commands),
    )
