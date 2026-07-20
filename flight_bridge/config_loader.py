"""Load typed, frozen configuration from YAML.

Single conversion boundary: YAML → frozen dataclasses.
All other modules depend on these dataclasses, never on the YAML file directly.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml


# ---------------------------------------------------------------------------
# Config dataclasses (frozen — immutable after load)
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class SdkConfig:
    """SDK TCP connection parameters."""

    host: str = "127.0.0.1"
    port: int = 9000
    timeout_ms: int = 100


@dataclass(frozen=True)
class ControlConfig:
    """Control mode and blending parameters."""

    mode: str = "velocity"  # "velocity" | "attitude" | "motor"
    residual_lambda: float = 0.03
    frequency_hz: float = 20.0


@dataclass(frozen=True)
class LimitsConfig:
    """Velocity/attitude limits for one deployment target (sim or real)."""

    vx_max: float = 0.30
    vy_max: float = 0.30
    vz_max: float = 0.20
    yaw_rate_max_deg: float = 10.0
    roll_max_deg: float = 5.0
    pitch_max_deg: float = 5.0

    @property
    def yaw_rate_max_radps(self) -> float:
        import math
        return math.radians(self.yaw_rate_max_deg)


@dataclass(frozen=True)
class FilterConfig:
    """Low-pass filter and deadband parameters."""

    alpha: float = 0.3
    deadband_u: float = 0.03
    deadband_v: float = 0.03


@dataclass(frozen=True)
class SafetyConfig:
    """Safety layer parameters."""

    enable_action_check: bool = True
    enable_timeout_fallback: bool = True
    enable_manual_override: bool = True
    fallback_mode: str = "hover"
    sdk_timeout_ms: int = 100
    state_age_max_ms: int = 100
    vx_rate_limit: float = 0.30
    vy_rate_limit: float = 0.30
    vz_rate_limit: float = 0.20
    yaw_rate_rate_limit: float = 20.0  # deg/s²


@dataclass(frozen=True)
class FeedbackConfig:
    """One-dimensional SDK feedback derived from a hover tracking cost."""

    mode: str = "reward_01"  # "reward_01" | "improvement_01" | "improvement_signed" | "cost_01"
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


@dataclass(frozen=True)
class CopterConfig:
    """Physical copter parameters (from model.sdf / ArduCopter config.h)."""

    hover_throttle: int = 700
    throttle_mid: int = 500
    throttle_min: int = 130
    throttle_max: int = 1000
    rc_altitude_kp_pwm_per_m: float = 70.0
    rc_vertical_speed_kd_pwm_per_mps: float = 50.0
    rc_baseline_min_pwm: int = 1600
    rc_baseline_max_pwm: int = 1760
    rc_xy_kp_pwm_per_m: float = 4.0
    rc_xy_kd_pwm_per_mps: float = 8.0
    rc_xy_max_delta_pwm: int = 40
    rc_xy_enable_alt_m: float = 0.8
    rc_xy_accel_kp_mps2_per_m: float = 0.35
    rc_xy_accel_kd_mps2_per_mps: float = 0.55
    rc_xy_max_accel_mps2: float = 1.2
    rc_response_ax_per_roll_pwm: float = 0.0305
    rc_response_ay_per_roll_pwm: float = 0.0030
    rc_response_ax_per_pitch_pwm: float = -0.0152
    rc_response_ay_per_pitch_pwm: float = 0.0141
    mass_kg: float = 1.5
    max_motor_speed_radps: float = 800.0
    bridge_max_motor_speed: float = 800.0


@dataclass(frozen=True)
class AppConfig:
    """Top-level application configuration."""

    sdk: SdkConfig
    control: ControlConfig
    limits_sim: LimitsConfig
    limits_real: LimitsConfig
    filter: FilterConfig
    safety: SafetyConfig
    feedback: FeedbackConfig
    copter: CopterConfig
    use_real: bool = False

    @property
    def limits(self) -> LimitsConfig:
        """Active limits based on use_real flag."""
        return self.limits_real if self.use_real else self.limits_sim


# ---------------------------------------------------------------------------
# Factory
# ---------------------------------------------------------------------------


def load_config(path: Path, use_real: bool = False) -> AppConfig:
    """Load configuration from a YAML file.

    Args:
        path: Path to config.yaml.
        use_real: If True, real-drone limits are baked into the active limits.

    Returns:
        Frozen AppConfig with all sections.
    """
    with open(path, encoding="utf-8") as fh:
        raw: dict[str, object] = yaml.safe_load(fh)

    return AppConfig(
        sdk=_load_sdk(raw.get("sdk", {})),
        control=_load_control(raw.get("control", {})),
        limits_sim=_load_limits(raw.get("limits_sim", {})),
        limits_real=_load_limits(raw.get("limits_real", {})),
        filter=_load_filter(raw.get("filter", {})),
        safety=_load_safety(raw.get("safety", {})),
        feedback=_load_feedback(raw.get("feedback", {})),
        copter=_load_copter(raw.get("copter", {})),
        use_real=use_real,
    )


# ---------------------------------------------------------------------------
# Internal section loaders
# ---------------------------------------------------------------------------


def _load_sdk(raw: object) -> SdkConfig:
    if not isinstance(raw, dict):
        return SdkConfig()
    return SdkConfig(
        host=str(raw.get("host", "127.0.0.1")),
        port=int(raw.get("port", 9000)),
        timeout_ms=int(raw.get("timeout_ms", 100)),
    )


def _load_control(raw: object) -> ControlConfig:
    if not isinstance(raw, dict):
        return ControlConfig()
    return ControlConfig(
        mode=str(raw.get("mode", "velocity")),
        residual_lambda=float(raw.get("residual_lambda", 0.03)),
        frequency_hz=float(raw.get("frequency_hz", 20.0)),
    )


def _load_limits(raw: object) -> LimitsConfig:
    if not isinstance(raw, dict):
        return LimitsConfig()
    return LimitsConfig(
        vx_max=float(raw.get("vx_max", 0.30)),
        vy_max=float(raw.get("vy_max", 0.30)),
        vz_max=float(raw.get("vz_max", 0.20)),
        yaw_rate_max_deg=float(raw.get("yaw_rate_max_deg", 10.0)),
        roll_max_deg=float(raw.get("roll_max_deg", 5.0)),
        pitch_max_deg=float(raw.get("pitch_max_deg", 5.0)),
    )


def _load_filter(raw: object) -> FilterConfig:
    if not isinstance(raw, dict):
        return FilterConfig()
    return FilterConfig(
        alpha=float(raw.get("alpha", 0.3)),
        deadband_u=float(raw.get("deadband_u", 0.03)),
        deadband_v=float(raw.get("deadband_v", 0.03)),
    )


def _load_safety(raw: object) -> SafetyConfig:
    if not isinstance(raw, dict):
        return SafetyConfig()
    return SafetyConfig(
        enable_action_check=bool(raw.get("enable_action_check", True)),
        enable_timeout_fallback=bool(raw.get("enable_timeout_fallback", True)),
        enable_manual_override=bool(raw.get("enable_manual_override", True)),
        fallback_mode=str(raw.get("fallback_mode", "hover")),
        sdk_timeout_ms=int(raw.get("sdk_timeout_ms", 100)),
        state_age_max_ms=int(raw.get("state_age_max_ms", 100)),
        vx_rate_limit=float(raw.get("vx_rate_limit", 0.30)),
        vy_rate_limit=float(raw.get("vy_rate_limit", 0.30)),
        vz_rate_limit=float(raw.get("vz_rate_limit", 0.20)),
        yaw_rate_rate_limit=float(raw.get("yaw_rate_rate_limit", 20.0)),
    )


def _load_feedback(raw: object) -> FeedbackConfig:
    if not isinstance(raw, dict):
        return FeedbackConfig()
    return FeedbackConfig(
        mode=str(raw.get("mode", "improvement_01")),
        neutral_value=float(raw.get("neutral_value", 0.5)),
        terminal_value=float(raw.get("terminal_value", 0.0)),
        startup_warmup_frames=int(raw.get("startup_warmup_frames", 10)),
        feedback_amplitude=float(raw.get("feedback_amplitude", 0.45)),
        improvement_scale=float(raw.get("improvement_scale", 0.04)),
        cost_scale=float(raw.get("cost_scale", 0.65)),
        improvement_weight=float(raw.get("improvement_weight", 0.25)),
        cost_filter_alpha=float(raw.get("cost_filter_alpha", 0.35)),
        xy_scale_m=float(raw.get("xy_scale_m", 2.0)),
        z_scale_m=float(raw.get("z_scale_m", 1.0)),
        vxy_scale_mps=float(raw.get("vxy_scale_mps", 1.0)),
        vz_scale_mps=float(raw.get("vz_scale_mps", 0.7)),
        tilt_scale_deg=float(raw.get("tilt_scale_deg", 30.0)),
        rate_scale_degps=float(raw.get("rate_scale_degps", 180.0)),
        yaw_scale_deg=float(raw.get("yaw_scale_deg", 45.0)),
        xy_weight=float(raw.get("xy_weight", 0.26)),
        z_weight=float(raw.get("z_weight", 0.18)),
        vxy_weight=float(raw.get("vxy_weight", 0.18)),
        vz_weight=float(raw.get("vz_weight", 0.12)),
        tilt_weight=float(raw.get("tilt_weight", 0.16)),
        rate_weight=float(raw.get("rate_weight", 0.05)),
        yaw_weight=float(raw.get("yaw_weight", 0.02)),
        command_weight=float(raw.get("command_weight", 0.03)),
        saturation_weight=float(raw.get("saturation_weight", 0.12)),
        safety_weight=float(raw.get("safety_weight", 0.25)),
    )


def _load_copter(raw: object) -> CopterConfig:
    if not isinstance(raw, dict):
        return CopterConfig()
    return CopterConfig(
        hover_throttle=int(raw.get("hover_throttle", 700)),
        throttle_mid=int(raw.get("throttle_mid", 500)),
        throttle_min=int(raw.get("throttle_min", 130)),
        throttle_max=int(raw.get("throttle_max", 1000)),
        rc_altitude_kp_pwm_per_m=float(raw.get("rc_altitude_kp_pwm_per_m", 70.0)),
        rc_vertical_speed_kd_pwm_per_mps=float(raw.get("rc_vertical_speed_kd_pwm_per_mps", 50.0)),
        rc_baseline_min_pwm=int(raw.get("rc_baseline_min_pwm", 1600)),
        rc_baseline_max_pwm=int(raw.get("rc_baseline_max_pwm", 1760)),
        rc_xy_kp_pwm_per_m=float(raw.get("rc_xy_kp_pwm_per_m", 4.0)),
        rc_xy_kd_pwm_per_mps=float(raw.get("rc_xy_kd_pwm_per_mps", 8.0)),
        rc_xy_max_delta_pwm=int(raw.get("rc_xy_max_delta_pwm", 40)),
        rc_xy_enable_alt_m=float(raw.get("rc_xy_enable_alt_m", 0.8)),
        rc_xy_accel_kp_mps2_per_m=float(raw.get("rc_xy_accel_kp_mps2_per_m", 0.35)),
        rc_xy_accel_kd_mps2_per_mps=float(raw.get("rc_xy_accel_kd_mps2_per_mps", 0.55)),
        rc_xy_max_accel_mps2=float(raw.get("rc_xy_max_accel_mps2", 1.2)),
        rc_response_ax_per_roll_pwm=float(raw.get("rc_response_ax_per_roll_pwm", 0.0305)),
        rc_response_ay_per_roll_pwm=float(raw.get("rc_response_ay_per_roll_pwm", 0.0030)),
        rc_response_ax_per_pitch_pwm=float(raw.get("rc_response_ax_per_pitch_pwm", -0.0152)),
        rc_response_ay_per_pitch_pwm=float(raw.get("rc_response_ay_per_pitch_pwm", 0.0141)),
        mass_kg=float(raw.get("mass_kg", 1.5)),
        max_motor_speed_radps=float(raw.get("max_motor_speed_radps", 800.0)),
        bridge_max_motor_speed=float(raw.get("bridge_max_motor_speed", 800.0)),
    )
