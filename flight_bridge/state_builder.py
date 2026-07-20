"""Copter state builder — hover observation + scalar feedback.

Design:
  - obs (16-d): tanh-normalized physical quantities.
  - feedback (1-d): configurable scalar derived from hover tracking cost.

All coordinates in body-FLU (x=forward, y=left, z=up).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

from flight_bridge.command_types import ShieldInfo, VelocityCmd


# ---------------------------------------------------------------------------
# CopterState — unified body-FLU state
# ---------------------------------------------------------------------------


@dataclass
class CopterState:
    """Unified copter state in body-FLU coordinates.

    All angles in radians, positions in meters, velocities in m/s.
    """

    # Position (body-FLU relative to target/home)
    x_err: float = 0.0   # forward error  (m)
    y_err: float = 0.0   # left error     (m)
    z_err: float = 0.0   # up error       (m)

    # Body-FLU velocity
    vx: float = 0.0      # forward  (m/s)
    vy: float = 0.0      # left     (m/s)
    vz: float = 0.0      # up       (m/s)

    # Attitude (rad)
    roll: float = 0.0
    pitch: float = 0.0
    yaw_err: float = 0.0  # heading error to target (rad)

    # Body angular rates (rad/s)
    p: float = 0.0       # roll rate
    q: float = 0.0       # pitch rate
    r: float = 0.0       # yaw rate

    # Additional info (for logging)
    altitude_m: float = 0.0
    armed: bool = False
    mode: str = ""


# ---------------------------------------------------------------------------
# CopterStateBuilder
# ---------------------------------------------------------------------------


class CopterStateBuilder:
    """Builds tanh-normalized physical observation vectors.

    The SDK receives bounded physics-derived quantities:
      obs[0..2]  : position error (m)
      obs[3..5]  : velocity (m/s)
      obs[6..8]  : attitude (rad)
      obs[9..11] : angular rates (rad/s)
      obs[12..15]: previous command (physical units or RC delta [-1,1])
    """

    OBS_DIM = 16

    # tanh scale factors
    _POS_SCALE = 2.0                    # m
    _VEL_SCALE = 1.0                    # m/s
    _ANGLE_SCALE = math.radians(30.0)   # rad
    _RATE_SCALE = math.radians(180.0)   # rad/s

    def build_hover_obs(
        self,
        state: CopterState,
        last_cmd: VelocityCmd | list[float] | None = None,
        rc_mode: bool = False,
    ) -> list[float]:
        """Build 16-d tanh-normalized observation.

        tanh(value/scale) maps each dimension to [-1, 1] while preserving
        relative magnitude — a 5m error (0.99) is distinguishable from a 2m
        error (0.76), unlike hard clipping.

        obs = [
            x_err, y_err, z_err,    # tanh(pos / 2.0m)
            vx, vy, vz,              # tanh(vel / 1.0m/s)
            roll, pitch, yaw_err,    # tanh(ang / 30°)
            p, q, r,                 # tanh(rate / 180°/s)
            last_cmd[0..3],          # raw (already in [-1,1] for RC)
        ]
        """
        if last_cmd is None:
            cmd = [0.0, 0.0, 0.0, 0.0]
        elif isinstance(last_cmd, VelocityCmd):
            cmd = [last_cmd.vx, last_cmd.vy, last_cmd.vz, last_cmd.yaw_rate]
        else:
            cmd = _pad_action(last_cmd, 4)

        raw = [
            state.x_err, state.y_err, state.z_err,
            state.vx, state.vy, state.vz,
            state.roll, state.pitch, state.yaw_err,
            state.p, state.q, state.r,
        ]
        scales = [
            self._POS_SCALE, self._POS_SCALE, self._POS_SCALE,
            self._VEL_SCALE, self._VEL_SCALE, self._VEL_SCALE,
            self._ANGLE_SCALE, self._ANGLE_SCALE, self._ANGLE_SCALE,
            self._RATE_SCALE, self._RATE_SCALE, self._RATE_SCALE,
        ]
        norm = [math.tanh(v / s) for v, s in zip(raw, scales)]
        return norm + cmd


# ---------------------------------------------------------------------------
# Feedback builder
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class HoverFeedbackConfig:
    """Configuration for scalar SDK feedback.

    The SDK currently expects feedback shape (1,), so multi-term hover quality is
    compressed into one value.  The default mode maps improvement to [0, 1]:
    In reward_01 mode, 1.0 means near-target hover and 0.0 means terminal/bad
    state.  Improvement is blended in as a small shaping term.
    """

    mode: str = "reward_01"
    neutral_value: float = 0.5
    terminal_value: float = 0.0
    startup_warmup_frames: int = 10
    feedback_amplitude: float = 0.45
    improvement_scale: float = 0.04
    cost_scale: float = 0.65
    improvement_weight: float = 0.25
    cost_filter_alpha: float = 0.35

    xy_scale_m: float = 2.0
    z_scale_m: float = 1.0
    vxy_scale_mps: float = 1.0
    vz_scale_mps: float = 0.7
    tilt_scale_deg: float = 30.0
    rate_scale_degps: float = 180.0
    yaw_scale_deg: float = 45.0

    xy_weight: float = 0.26
    z_weight: float = 0.18
    vxy_weight: float = 0.18
    vz_weight: float = 0.12
    tilt_weight: float = 0.16
    rate_weight: float = 0.05
    yaw_weight: float = 0.02
    command_weight: float = 0.03
    saturation_weight: float = 0.12
    safety_weight: float = 0.25


@dataclass
class HoverFeedbackDiagnostics:
    """Detailed cost terms for JSONL logs and offline analysis."""

    mode: str = "improvement_01"
    value: float = 0.5
    cost: float = 0.0
    filtered_cost: float = 0.0
    previous_filtered_cost: float | None = None
    improvement: float = 0.0
    terms: dict[str, float] = field(default_factory=dict)

    def to_log_dict(self) -> dict[str, Any]:
        return {
            "mode": self.mode,
            "value": self.value,
            "cost": self.cost,
            "filtered_cost": self.filtered_cost,
            "previous_filtered_cost": self.previous_filtered_cost,
            "improvement": self.improvement,
            "terms": dict(self.terms),
        }


class HoverFeedbackBuilder:
    """Build one-value SDK feedback from mature hover tracking cost terms."""

    def __init__(self, config: HoverFeedbackConfig | Any | None = None) -> None:
        self._cfg = _coerce_feedback_config(config)
        self._previous_filtered_cost: float | None = None
        self._pending_value: float | None = None
        self._warmup_frames_remaining = max(0, int(self._cfg.startup_warmup_frames))
        self.last_diagnostics = HoverFeedbackDiagnostics(
            mode=self._cfg.mode,
            value=self._cfg.neutral_value,
        )

    def reset(self, pending_value: float | None = None) -> None:
        """Forget temporal cost history, for example after a Gazebo reset."""
        self._previous_filtered_cost = None
        self._pending_value = pending_value
        self._warmup_frames_remaining = max(0, int(self._cfg.startup_warmup_frames))
        self.last_diagnostics = HoverFeedbackDiagnostics(
            mode=self._cfg.mode,
            value=self._cfg.neutral_value if pending_value is None else pending_value,
        )

    @property
    def neutral_value(self) -> float:
        return self._cfg.neutral_value

    @property
    def terminal_value(self) -> float:
        return self._cfg.terminal_value

    def build(
        self,
        state: CopterState,
        shield_info: ShieldInfo | None = None,
        last_cmd: list[float] | VelocityCmd | None = None,
    ) -> list[float]:
        terms = self._build_terms(state, shield_info, last_cmd)
        cost = sum(terms.values())
        filtered_cost = self._filter_cost(cost)

        previous = self._previous_filtered_cost
        improvement = 0.0 if previous is None else previous - filtered_cost
        if self._pending_value is not None:
            value = self._pending_value
            self._pending_value = None
        elif self._warmup_frames_remaining > 0:
            value = self._cfg.neutral_value
            self._warmup_frames_remaining -= 1
        else:
            value = self._feedback_value(cost, filtered_cost, improvement, previous)

        self._previous_filtered_cost = filtered_cost
        self.last_diagnostics = HoverFeedbackDiagnostics(
            mode=self._cfg.mode,
            value=value,
            cost=cost,
            filtered_cost=filtered_cost,
            previous_filtered_cost=previous,
            improvement=improvement,
            terms=terms,
        )
        return [value]

    def _build_terms(
        self,
        state: CopterState,
        shield_info: ShieldInfo | None,
        last_cmd: list[float] | VelocityCmd | None,
    ) -> dict[str, float]:
        cfg = self._cfg
        xy_error = math.hypot(state.x_err, state.y_err)
        vxy = math.hypot(state.vx, state.vy)
        tilt = math.hypot(state.roll, state.pitch)
        angular_rate = math.sqrt(state.p ** 2 + state.q ** 2 + state.r ** 2)
        command_effort = _command_effort(last_cmd)
        saturation = 1.0 if shield_info is not None and shield_info.saturated else 0.0
        safety = 1.0 if shield_info is not None and shield_info.safety_triggered else 0.0

        return {
            "xy": cfg.xy_weight * _soft_unit(xy_error, cfg.xy_scale_m),
            "z": cfg.z_weight * _soft_unit(abs(state.z_err), cfg.z_scale_m),
            "vxy": cfg.vxy_weight * _soft_unit(vxy, cfg.vxy_scale_mps),
            "vz": cfg.vz_weight * _soft_unit(abs(state.vz), cfg.vz_scale_mps),
            "tilt": cfg.tilt_weight * _soft_unit(
                tilt, math.radians(cfg.tilt_scale_deg)
            ),
            "rate": cfg.rate_weight * _soft_unit(
                angular_rate, math.radians(cfg.rate_scale_degps)
            ),
            "yaw": cfg.yaw_weight * _soft_unit(
                abs(state.yaw_err), math.radians(cfg.yaw_scale_deg)
            ),
            "command": cfg.command_weight * _soft_unit(command_effort, 1.0),
            "saturation": cfg.saturation_weight * saturation,
            "safety": cfg.safety_weight * safety,
        }

    def _filter_cost(self, cost: float) -> float:
        previous = self._previous_filtered_cost
        if previous is None:
            return cost
        alpha = _clamp(self._cfg.cost_filter_alpha, 0.0, 1.0)
        return alpha * cost + (1.0 - alpha) * previous

    def _feedback_value(
        self,
        cost: float,
        filtered_cost: float,
        improvement: float,
        previous: float | None,
    ) -> float:
        cfg = self._cfg
        mode = cfg.mode.lower()
        if previous is None and mode.startswith("improvement"):
            return _clamp(cfg.neutral_value, 0.0, 1.0)
        if mode == "improvement_signed":
            return _clamp(
                improvement / max(cfg.improvement_scale, 1.0e-6),
                -1.0,
                1.0,
            )
        if mode == "cost_01":
            return _clamp(math.tanh(filtered_cost), 0.0, 1.0)
        if mode == "reward_01":
            cost_reward = math.exp(-filtered_cost / max(cfg.cost_scale, 1.0e-6))
            shaped_improvement = cfg.improvement_weight * math.tanh(
                improvement / max(cfg.improvement_scale, 1.0e-6)
            )
            return _clamp(cost_reward + shaped_improvement, 0.0, 1.0)

        scaled = math.tanh(improvement / max(cfg.improvement_scale, 1.0e-6))
        return _clamp(
            cfg.neutral_value + cfg.feedback_amplitude * scaled,
            0.0,
            1.0,
        )


def build_hover_feedback(state: CopterState) -> list[float]:
    """Backward-compatible distance feedback — tanh(distance / 2.0).

    Smoothly maps [0, ∞) → [0, 1):
      0m   → 0.00   (on target)
      1m   → 0.46
      2m   → 0.76
      3m   → 0.90
      5m   → 0.99
    Bounded output keeps the SDK model in-distribution.
    """
    distance = math.sqrt(
        state.x_err ** 2 + state.y_err ** 2 + state.z_err ** 2
    )
    return [math.tanh(distance / 2.0)]


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _pad_action(action: list[float], target_len: int) -> list[float]:
    if len(action) >= target_len:
        return action[:target_len]
    return action + [0.0] * (target_len - len(action))


def _coerce_feedback_config(config: HoverFeedbackConfig | Any | None) -> HoverFeedbackConfig:
    if config is None:
        return HoverFeedbackConfig()
    if isinstance(config, HoverFeedbackConfig):
        return config
    defaults = HoverFeedbackConfig()
    values = {
        field_name: getattr(config, field_name, getattr(defaults, field_name))
        for field_name in defaults.__dataclass_fields__
    }
    return HoverFeedbackConfig(**values)


def _soft_unit(value: float, scale: float) -> float:
    return math.tanh(max(0.0, value) / max(scale, 1.0e-6))


def _command_effort(last_cmd: list[float] | VelocityCmd | None) -> float:
    if last_cmd is None:
        return 0.0
    if isinstance(last_cmd, VelocityCmd):
        values = last_cmd.to_list()
    else:
        values = _pad_action([float(v) for v in last_cmd], 4)
    return math.sqrt(sum(v * v for v in values[:4]) / 4.0)


def _clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))
