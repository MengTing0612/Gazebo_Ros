"""Safety layer — all SDK actions must pass through here before reaching the flight controller.

Implements design doc §9: clip, low-pass filter, deadband, timeout fallback,
NaN/inf detection, rate-of-change limiting, and mode-switching protection.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field

from flight_bridge.command_types import RcCommand, ShieldInfo, VelocityCmd
from flight_bridge.config_loader import FilterConfig, LimitsConfig, SafetyConfig


@dataclass
class _CmdState:
    """Mutable internal state for the safety shield."""

    prev_cmd: VelocityCmd = field(default_factory=VelocityCmd)
    prev_time: float = 0.0
    saturation_count: int = 0
    consecutive_anomalies: int = 0


class SafetyShield:
    """Filters and validates commands before they reach the vehicle.

    Usage:
        shield = SafetyShield(safety_cfg, limits_cfg, filter_cfg)
        safe_cmd, info = shield.filter(raw_cmd)           # velocity
        safe_rc, info = shield.filter_rc(raw_rc_cmd)       # RC PWM
    """

    def __init__(
        self, safety: SafetyConfig, limits: LimitsConfig, filter_cfg: FilterConfig,
        hover_throttle_pwm: int = 1500,
    ) -> None:
        self._safety = safety
        self._limits = limits
        self._filter = filter_cfg
        self._hover_throttle_pwm = hover_throttle_pwm
        self._state = _CmdState(prev_time=time.time())
        self._rc_prev_cmd: RcCommand | None = None
        self._rc_prev_time: float = time.time()
        # RC path global stats (mirroring _state for velocity path)
        self._rc_saturation_count: int = 0
        self._rc_anomaly_count: int = 0

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def filter(self, raw_cmd: VelocityCmd) -> tuple[VelocityCmd, ShieldInfo]:
        """Apply all safety checks in sequence.

        Order: NaN/inf → range clip → low-pass → deadband → rate-of-change.
        Rate-of-change is the final gate — it protects the vehicle from large
        step changes in the smoothed command signal.
        """
        info = ShieldInfo()

        # 1. NaN / inf check — discard, use previous safe command
        if self._has_anomaly(raw_cmd):
            self._state.consecutive_anomalies += 1
            info.safety_triggered = True
            safe = self._state.prev_cmd
        else:
            self._state.consecutive_anomalies = 0
            safe = raw_cmd

        # 2. Range check — clip to limits
        safe, clipped = self._clip(safe)
        info.clipped_fields = clipped
        info.saturated = len(clipped) > 0

        # 3. Low-pass filter — smooth SDK jitter
        safe, info.filtered = self._lowpass(safe)

        # 4. Deadband — zero out micro-corrections on smoothed signal
        safe, info.in_deadband = self._deadband(safe)

        # 5. Rate-of-change limit — final gate protecting vehicle from large steps
        now = time.time()
        dt = now - self._state.prev_time
        safe, rate_clipped = self._rate_limit(safe, dt)
        if rate_clipped:
            info.clipped_fields.extend(rate_clipped)
            info.saturated = True

        # 6. Update internal state
        self._state.prev_cmd = safe
        self._state.prev_time = now
        if info.saturated:
            self._state.saturation_count += 1

        return safe, info

    def reset(self) -> None:
        """Clear internal state (call on mode switch or disarm)."""
        self._state = _CmdState(prev_time=time.time())
        self._rc_prev_cmd = None
        self._rc_prev_time = time.time()
        self._rc_saturation_count = 0
        self._rc_anomaly_count = 0

    @property
    def is_saturated(self) -> bool:
        return self._state.saturation_count > 0 or self._rc_saturation_count > 0

    @property
    def safety_flag(self) -> bool:
        return self._state.consecutive_anomalies > 0 or self._rc_anomaly_count > 0

    @property
    def prev_cmd(self) -> VelocityCmd:
        return self._state.prev_cmd

    # ------------------------------------------------------------------
    # Timeout / state-age checks (§9.4)
    # ------------------------------------------------------------------

    def should_accept_action(self, latency_ms: float, state_age_ms: float) -> bool:
        """Check whether SDK latency and state age are within limits.

        Returns False if the action should be discarded due to timeout.
        """
        if self._safety.enable_timeout_fallback:
            if latency_ms > self._safety.sdk_timeout_ms:
                return False
            if state_age_ms > self._safety.state_age_max_ms:
                return False
        return True

    # ------------------------------------------------------------------
    # RC PWM filter (primary APM 2.74b path — 5-stage pipeline)
    # ------------------------------------------------------------------

    def filter_rc(self, raw_cmd: RcCommand) -> tuple[RcCommand, ShieldInfo]:
        """Validate and smooth RC PWM command.

        Order: NaN/inf → PWM range clip → low-pass → deadband → rate-of-change.
        NaN fallback uses configured hover_throttle_pwm, not hardcoded 1500.
        """
        info = ShieldInfo()
        clipped: list[str] = []

        # 1. NaN / inf check — discard, use previous safe or hover neutral
        if self._has_anomaly_rc(raw_cmd):
            info.safety_triggered = True
            self._rc_anomaly_count += 1
            fallback = (
                self._rc_prev_cmd
                if self._rc_prev_cmd is not None
                else RcCommand.neutral(self._hover_throttle_pwm)
            )
            return fallback, info

        # 2. PWM range clip [1000, 2000]
        roll = _clamp_pwm(raw_cmd.roll, "roll", clipped)
        pitch = _clamp_pwm(raw_cmd.pitch, "pitch", clipped)
        throttle = _clamp_pwm(raw_cmd.throttle, "throttle", clipped)
        yaw = _clamp_pwm(raw_cmd.yaw, "yaw", clipped)
        safe = RcCommand(roll=roll, pitch=pitch, throttle=throttle, yaw=yaw)

        # 3. Low-pass filter on RC deltas (smooth SDK jitter)
        if self._rc_prev_cmd is not None:
            alpha = self._filter.alpha
            safe = RcCommand(
                roll=int(alpha * safe.roll + (1 - alpha) * self._rc_prev_cmd.roll),
                pitch=int(alpha * safe.pitch + (1 - alpha) * self._rc_prev_cmd.pitch),
                throttle=int(alpha * safe.throttle + (1 - alpha) * self._rc_prev_cmd.throttle),
                yaw=int(alpha * safe.yaw + (1 - alpha) * self._rc_prev_cmd.yaw),
            )
        info.filtered = (
            self._rc_prev_cmd is not None
            and (safe.roll != raw_cmd.roll or safe.pitch != raw_cmd.pitch
                 or safe.throttle != raw_cmd.throttle or safe.yaw != raw_cmd.yaw)
        )

        # 4. Deadband on stick channels (zero out small deflections)
        neutral = RcCommand.neutral(self._hover_throttle_pwm)
        deadband_pwm = 2  # ±2 PWM deadband (~0.09° at 500 PWM = 45°)
        safe = RcCommand(
            roll=safe.roll if abs(safe.roll - neutral.roll) > deadband_pwm else neutral.roll,
            pitch=safe.pitch if abs(safe.pitch - neutral.pitch) > deadband_pwm else neutral.pitch,
            throttle=safe.throttle,  # no deadband on throttle — altitude needs fast response
            yaw=safe.yaw if abs(safe.yaw - neutral.yaw) > deadband_pwm else neutral.yaw,
        )
        info.in_deadband = (
            safe.roll == neutral.roll and raw_cmd.roll != neutral.roll
            or safe.pitch == neutral.pitch and raw_cmd.pitch != neutral.pitch
            or safe.yaw == neutral.yaw and raw_cmd.yaw != neutral.yaw
        )

        # 5. Rate-of-change limit on PWM (all channels).
        now = time.time()
        dt = now - self._rc_prev_time
        if self._rc_prev_cmd is not None and dt > 0:
            max_pwm_step = 100  # max 100 PWM change per frame (~4.5° at 20Hz)
            roll_rl, roll_clipped = _rate_clamp_int_rpt(
                safe.roll, self._rc_prev_cmd.roll, max_pwm_step)
            pitch_rl, pitch_clipped = _rate_clamp_int_rpt(
                safe.pitch, self._rc_prev_cmd.pitch, max_pwm_step)
            throttle_rl, thr_clipped = _rate_clamp_int_rpt(
                safe.throttle, self._rc_prev_cmd.throttle, max_pwm_step)
            yaw_rl, yaw_clipped = _rate_clamp_int_rpt(
                safe.yaw, self._rc_prev_cmd.yaw, max_pwm_step)
            safe = RcCommand(roll=roll_rl, pitch=pitch_rl, throttle=throttle_rl, yaw=yaw_rl)
            if roll_clipped: clipped.append("rc_roll_rate")
            if pitch_clipped: clipped.append("rc_pitch_rate")
            if thr_clipped: clipped.append("rc_throttle_rate")
            if yaw_clipped: clipped.append("rc_yaw_rate")

        info.clipped_fields = clipped
        info.saturated = len(clipped) > 0

        # Update RC state + global stats
        self._rc_prev_cmd = safe
        self._rc_prev_time = now
        if info.saturated:
            self._rc_saturation_count += 1

        return safe, info

    @staticmethod
    def _has_anomaly_rc(cmd: RcCommand) -> bool:
        """True if any field is NaN or outside valid range."""
        for v in (cmd.roll, cmd.pitch, cmd.throttle, cmd.yaw):
            if math.isnan(float(v)) or math.isinf(float(v)):
                return True
        return False

    # ------------------------------------------------------------------
    # Internal checks
    # ------------------------------------------------------------------

    @staticmethod
    def _has_anomaly(cmd: VelocityCmd) -> bool:
        """True if any field is NaN or inf."""
        for v in (cmd.vx, cmd.vy, cmd.vz, cmd.yaw_rate):
            if math.isnan(v) or math.isinf(v):
                return True
        return False

    def _clip(self, cmd: VelocityCmd) -> tuple[VelocityCmd, list[str]]:
        """Clip each axis to configured limits. Returns (clipped_cmd, field_names)."""
        clipped: list[str] = []
        vx = self._clamp_field(cmd.vx, -self._limits.vx_max, self._limits.vx_max, "vx", clipped)
        vy = self._clamp_field(cmd.vy, -self._limits.vy_max, self._limits.vy_max, "vy", clipped)
        vz = self._clamp_field(cmd.vz, -self._limits.vz_max, self._limits.vz_max, "vz", clipped)
        yr_max = math.radians(self._limits.yaw_rate_max_deg)
        yr = self._clamp_field(cmd.yaw_rate, -yr_max, yr_max, "yaw_rate", clipped)
        return VelocityCmd(vx=vx, vy=vy, vz=vz, yaw_rate=yr), clipped

    @staticmethod
    def _clamp_field(
        value: float, lower: float, upper: float,
        name: str, clipped: list[str],
    ) -> float:
        if value < lower:
            clipped.append(name)
            return lower
        if value > upper:
            clipped.append(name)
            return upper
        return value

    def _rate_limit(self, cmd: VelocityCmd, dt: float) -> tuple[VelocityCmd, list[str]]:
        """Limit command change rate per axis."""
        if dt <= 0.0:
            return cmd, []
        prev = self._state.prev_cmd
        clipped: list[str] = []
        vx = self._rate_clamp(cmd.vx, prev.vx, self._safety.vx_rate_limit, dt, "vx", clipped)
        vy = self._rate_clamp(cmd.vy, prev.vy, self._safety.vy_rate_limit, dt, "vy", clipped)
        vz = self._rate_clamp(cmd.vz, prev.vz, self._safety.vz_rate_limit, dt, "vz", clipped)
        yr = self._rate_clamp(
            cmd.yaw_rate, prev.yaw_rate,
            math.radians(self._safety.yaw_rate_rate_limit), dt, "yaw_rate", clipped,
        )
        return VelocityCmd(vx=vx, vy=vy, vz=vz, yaw_rate=yr), clipped

    @staticmethod
    def _rate_clamp(
        new: float, prev: float, rate_limit: float, dt: float,
        name: str, clipped: list[str],
    ) -> float:
        max_delta = rate_limit * dt
        if new - prev > max_delta:
            clipped.append(name)
            return prev + max_delta
        if prev - new > max_delta:
            clipped.append(name)
            return prev - max_delta
        return new

    def _lowpass(self, cmd: VelocityCmd) -> tuple[VelocityCmd, bool]:
        """First-order low-pass filter: out = α·new + (1-α)·prev."""
        alpha = self._filter.alpha
        prev = self._state.prev_cmd
        filtered = VelocityCmd(
            vx=alpha * cmd.vx + (1 - alpha) * prev.vx,
            vy=alpha * cmd.vy + (1 - alpha) * prev.vy,
            vz=alpha * cmd.vz + (1 - alpha) * prev.vz,
            yaw_rate=alpha * cmd.yaw_rate + (1 - alpha) * prev.yaw_rate,
        )
        changed = (
            abs(filtered.vx - cmd.vx) > 0.001
            or abs(filtered.vy - cmd.vy) > 0.001
            or abs(filtered.vz - cmd.vz) > 0.001
            or abs(filtered.yaw_rate - cmd.yaw_rate) > 0.001
        )
        return filtered, changed

    def _deadband(self, cmd: VelocityCmd) -> tuple[VelocityCmd, bool]:
        """Zero out small commands to reduce jitter."""
        threshold_uv = self._filter.deadband_u
        threshold_v = self._filter.deadband_v
        vx = 0.0 if abs(cmd.vx) < threshold_uv else cmd.vx
        vy = 0.0 if abs(cmd.vy) < threshold_uv else cmd.vy
        vz = 0.0 if abs(cmd.vz) < threshold_v else cmd.vz
        in_db = (vx != cmd.vx or vy != cmd.vy or vz != cmd.vz)
        return VelocityCmd(vx=vx, vy=vy, vz=vz, yaw_rate=cmd.yaw_rate), in_db


def _clamp_pwm(value: int, name: str, clipped: list[str]) -> int:
    if value < 1000:
        clipped.append(name)
        return 1000
    if value > 2000:
        clipped.append(name)
        return 2000
    return value


def _rate_clamp_int_rpt(new: int, prev: int, max_step: int) -> tuple[int, bool]:
    """Clamp change and report whether clamping occurred."""
    if new - prev > max_step:
        return prev + max_step, True
    if prev - new > max_step:
        return prev - max_step, True
    return new, False
