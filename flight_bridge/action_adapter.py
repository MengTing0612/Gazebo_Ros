"""Action adapter — SDK action → velocity or RC residual commands.

Three mapping modes:
  - front-4 (default):     action[0..3] → roll/pitch/throttle/yaw PWM offsets
  - learned_linear (W):    action[0..9] → W @ action → 4-d RC delta PWM
  - velocity:              action[0..3] → vx/vy/vz/yaw_rate

Residual blending: final = baseline + λ × sdk_cmd
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from flight_bridge.command_types import RcCommand, VelocityCmd
from flight_bridge.config_loader import ControlConfig, CopterConfig, LimitsConfig


# ---------------------------------------------------------------------------
# Learned adapter config
# ---------------------------------------------------------------------------


@dataclass
class LearnedAdapterConfig:
    """Configuration loaded from gaia_adapter.yaml."""
    action_dim: int = 10
    residual_lambda: float = 0.03
    max_delta_pwm: dict[str, float] | None = None
    W: list[list[float]] | None = None  # [4, 10]

    @classmethod
    def from_yaml(cls, path: Path) -> "LearnedAdapterConfig":
        with open(path, encoding="utf-8") as f:
            raw = yaml.safe_load(f)
        adapter = raw.get("adapter", raw)
        return cls(
            action_dim=int(adapter.get("action_dim", 10)),
            residual_lambda=float(adapter.get("residual_lambda", 0.03)),
            max_delta_pwm=adapter.get("max_delta_pwm"),
            W=[
                adapter.get("W", {}).get("roll", [0]*10),
                adapter.get("W", {}).get("pitch", [0]*10),
                adapter.get("W", {}).get("throttle", [0]*10),
                adapter.get("W", {}).get("yaw", [0]*10),
            ],
        )

    @property
    def has_W(self) -> bool:
        return self.W is not None and len(self.W) == 4


# ---------------------------------------------------------------------------
# ActionAdapter
# ---------------------------------------------------------------------------


class ActionAdapter:
    """Maps SDK action vectors to RC PWM or velocity commands.

    Usage:
        adapter = ActionAdapter(control_cfg, limits_cfg, copter_cfg)
        rc = adapter.to_rc_command(action)       # front-4 mapping
        delta = adapter.to_rc_delta(action)       # learned W → delta PWM

        # Load learned adapter:
        adapter.load_learned("flight_bridge/gaia_adapter.yaml")
        rc = adapter.gaia_action_to_rc(baseline, action)
    """

    def __init__(
        self, control: ControlConfig, limits: LimitsConfig, copter: CopterConfig,
    ) -> None:
        self._control = control
        self._limits = limits
        self._copter = copter
        self._yaw_max_rad = math.radians(limits.yaw_rate_max_deg)
        self._learned: LearnedAdapterConfig | None = None

    def load_learned(self, path: Path) -> None:
        """Load a learned W-matrix adapter from YAML."""
        self._learned = LearnedAdapterConfig.from_yaml(path)

    @property
    def has_learned(self) -> bool:
        return self._learned is not None and self._learned.has_W

    @property
    def learned_lambda(self) -> float:
        return self._learned.residual_lambda if self._learned else 0.03

    # ------------------------------------------------------------------
    # RC residual mapping (primary — APM 2.74b native path)
    # ------------------------------------------------------------------

    def to_rc_command(self, action: list[float]) -> RcCommand:
        """Convert SDK action [-1,1] to RC PWM offsets around neutral.

        action[0] → roll PWM offset   (±max_roll_deg → ±500 PWM)
        action[1] → pitch PWM offset  (±max_pitch_deg → ±500 PWM)
        action[2] → throttle offset   (±1.0 → ±200 throttle units)
        action[3] → yaw PWM offset    (±max_yaw_rate → ±500 PWM)

        Neutral positions:
          roll_neutral    = 1500
          pitch_neutral   = 1500
          throttle_neutral = hover_throttle_mapped
          yaw_neutral     = 1500
        """
        a = _pad_action(action, 4)

        # Roll/Pitch: action ∈ [-1,1] → angle ∈ [-max, max] → PWM offset ∈ [-500, 500]
        roll_offset = int(_clamp(a[0], -1.0, 1.0) * self._limits.roll_max_deg / 45.0 * 500)
        pitch_offset = int(_clamp(a[1], -1.0, 1.0) * self._limits.pitch_max_deg / 45.0 * 500)

        # Throttle: action[2] ∈ [-1,1] → throttle offset around hover_throttle.
        # Range: hover_throttle ± (throttle_max - hover_throttle) so the SDK
        # can command anything from throttle_min to throttle_max at full action.
        _throttle_range = max(
            self._copter.hover_throttle - self._copter.throttle_min,
            self._copter.throttle_max - self._copter.hover_throttle,
        )
        throttle_raw = self._copter.hover_throttle + _clamp(a[2], -1.0, 1.0) * _throttle_range
        throttle_mapped = int(
            _clamp(throttle_raw, self._copter.throttle_min, self._copter.throttle_max)
            + 1000  # ArduCopter 0-1000 → PWM 1000-2000
        )

        # Yaw: action[3] → yaw_rate → PWM offset
        yaw_offset = int(
            _clamp(a[3], -1.0, 1.0) * self._limits.yaw_rate_max_deg / 45.0 * 500
        )

        return RcCommand(
            roll=1500 + roll_offset,
            pitch=1500 + pitch_offset,
            throttle=throttle_mapped,
            yaw=1500 + yaw_offset,
        )

    def residual_blend_rc(
        self, baseline: RcCommand, sdk_cmd: RcCommand,
    ) -> RcCommand:
        """Blend baseline RC with SDK correction.

        final_pwm = baseline_pwm + λ × (sdk_pwm - neutral_pwm)
        """
        lam = self._control.residual_lambda
        neutral = self.baseline_rc_neutral()
        return RcCommand(
            roll=_clamp_pwm(baseline.roll + int(lam * (sdk_cmd.roll - neutral.roll))),
            pitch=_clamp_pwm(baseline.pitch + int(lam * (sdk_cmd.pitch - neutral.pitch))),
            throttle=_clamp_pwm(baseline.throttle + int(lam * (sdk_cmd.throttle - neutral.throttle))),
            yaw=_clamp_pwm(baseline.yaw + int(lam * (sdk_cmd.yaw - neutral.yaw))),
        )

    def baseline_rc_neutral(self) -> RcCommand:
        """RC neutral with hover throttle."""
        hover_pwm = min(2000, max(1000, self._copter.hover_throttle + 1000))
        return RcCommand(roll=1500, pitch=1500, throttle=hover_pwm, yaw=1500)

    def baseline_rc_for_altitude(
        self,
        z_err_m: float,
        vz_up_mps: float,
        x_err_m: float = 0.0,
        y_err_m: float = 0.0,
        vx_mps: float = 0.0,
        vy_mps: float = 0.0,
        altitude_m: float = 0.0,
    ) -> RcCommand:
        """Compute a conservative RC baseline for low-altitude hover.

        This deliberately avoids inverting a calibrated RC-to-acceleration
        matrix.  The baseline is only a blunt stabilizer: altitude hold plus
        weak body-axis position bias and horizontal-speed damping.
        """
        neutral = self.baseline_rc_neutral()
        throttle = (
            neutral.throttle
            + self._copter.rc_altitude_kp_pwm_per_m * z_err_m
            - self._copter.rc_vertical_speed_kd_pwm_per_mps * vz_up_mps
        )
        min_pwm = max(1000 + self._copter.throttle_min, self._copter.rc_baseline_min_pwm)
        max_pwm = min(1000 + self._copter.throttle_max, self._copter.rc_baseline_max_pwm)
        roll_delta = 0.0
        pitch_delta = 0.0
        if altitude_m >= self._copter.rc_xy_enable_alt_m:
            max_xy = float(self._copter.rc_xy_max_delta_pwm)
            # x_err = drone_pos - anchor_pos in body frame.
            # Positive x_err → drone overshot anchor → need backward accel → nose UP.
            # desired_accel = Kp*(anchor - drone) - Kd*vel = -Kp*x_err - Kd*vx
            # pitch_delta ∝ -desired_accel = +Kp*x_err + Kd*vx
            pitch_delta = (
                self._copter.rc_xy_kp_pwm_per_m * x_err_m
                + self._copter.rc_xy_kd_pwm_per_mps * vx_mps
            )
            roll_delta = (
                self._copter.rc_xy_kp_pwm_per_m * y_err_m
                + self._copter.rc_xy_kd_pwm_per_mps * vy_mps
            )
            pitch_delta = _clamp(pitch_delta, -max_xy, max_xy)
            roll_delta = _clamp(roll_delta, -max_xy, max_xy)
        return RcCommand(
            roll=_clamp_pwm(int(round(neutral.roll + roll_delta))),
            pitch=_clamp_pwm(int(round(neutral.pitch + pitch_delta))),
            throttle=_clamp_pwm(int(round(_clamp(throttle, min_pwm, max_pwm)))),
            yaw=neutral.yaw,
        )

    # ------------------------------------------------------------------
    # Velocity control mapping (secondary — for modern ArduCopter)
    # ------------------------------------------------------------------

    def to_velocity_cmd(self, action: list[float]) -> VelocityCmd:
        """Convert SDK action to body-FLU velocity + yaw-rate command.

        action[0] → vx (forward,  m/s)
        action[1] → vy (left,     m/s)
        action[2] → vz (up,       m/s)
        action[3] → yaw_rate (rad/s)
        """
        a = _pad_action(action, 4)
        return VelocityCmd(
            vx=_clamp(a[0], -1.0, 1.0) * self._limits.vx_max,
            vy=_clamp(a[1], -1.0, 1.0) * self._limits.vy_max,
            vz=_clamp(a[2], -1.0, 1.0) * self._limits.vz_max,
            yaw_rate=_clamp(a[3], -1.0, 1.0) * self._yaw_max_rad,
        )

    def residual_blend(
        self, baseline: VelocityCmd, sdk_cmd: VelocityCmd,
    ) -> VelocityCmd:
        """final = baseline + λ × sdk_cmd"""
        lam = self._control.residual_lambda
        return VelocityCmd(
            vx=baseline.vx + lam * sdk_cmd.vx,
            vy=baseline.vy + lam * sdk_cmd.vy,
            vz=baseline.vz + lam * sdk_cmd.vz,
            yaw_rate=baseline.yaw_rate + lam * sdk_cmd.yaw_rate,
        )

    @staticmethod
    def baseline_hover_cmd() -> VelocityCmd:
        return VelocityCmd.hover()

    @property
    def residual_lambda(self) -> float:
        return self._control.residual_lambda

    # ------------------------------------------------------------------
    # Learned linear adapter (W matrix from contribution analysis)
    # ------------------------------------------------------------------

    def to_rc_delta(self, action: list[float]) -> list[float]:
        """Compute RC delta PWM via learned W matrix: delta = W @ action.

        Returns [roll_delta, pitch_delta, throttle_delta, yaw_delta] in PWM.
        Falls back to zeros if no learned adapter is loaded.
        """
        if not self.has_learned:
            return [0.0, 0.0, 0.0, 0.0]

        cfg = self._learned
        a = _pad_action(action, cfg.action_dim)
        # Clip action to valid range
        a = [_clamp(v, -1.0, 1.0) for v in a[:cfg.action_dim]]

        # W @ action
        delta = [sum(cfg.W[row][j] * a[j] for j in range(cfg.action_dim))
                 for row in range(4)]

        # Apply residual lambda online (W is saved raw)
        lam = cfg.residual_lambda
        delta = [d * lam for d in delta]

        # Axis-level clip
        limits = cfg.max_delta_pwm or {}
        names = ["roll", "pitch", "throttle", "yaw"]
        for row, name in enumerate(names):
            limit = limits.get(name, 30.0)
            delta[row] = _clamp(delta[row], -limit, limit)

        return delta

    def gaia_action_to_rc(
        self, baseline: RcCommand, action: list[float],
    ) -> RcCommand:
        """Full learned adapter: action → W@action → RC delta → final PWM.

        final = baseline + W @ action, clamped to [1000, 2000].
        """
        delta = self.to_rc_delta(action)
        return RcCommand(
            roll=_clamp_pwm(baseline.roll + int(round(delta[0]))),
            pitch=_clamp_pwm(baseline.pitch + int(round(delta[1]))),
            throttle=_clamp_pwm(baseline.throttle + int(round(delta[2]))),
            yaw=_clamp_pwm(baseline.yaw + int(round(delta[3]))),
        )


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _clamp_pwm(value: int) -> int:
    return max(1000, min(2000, value))


def _pad_action(action: list[float], target_len: int) -> list[float]:
    if len(action) >= target_len:
        return action[:target_len]
    return action + [0.0] * (target_len - len(action))
