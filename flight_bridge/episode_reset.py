"""Gazebo episode reset and recovery state helpers."""

from __future__ import annotations

import argparse
import math
import os
import subprocess
import time
from dataclasses import dataclass

from flight_bridge.mavlink_adapter import MavlinkAdapter


@dataclass
class GazeboResetResult:
    """Result of a Gazebo-only model reset."""

    triggered: bool = False
    ok: bool = False
    reason: str = ""
    tilt_deg: float = 0.0
    xy_error_m: float = 0.0
    altitude_m: float = 0.0
    trigger_throttle_pwm: int = 0
    trigger_vz_mps: float = 0.0
    trigger_z_err_m: float = 0.0
    method: str = ""
    error: str | None = None
    reset_at: float = 0.0

    def to_log_dict(self) -> dict[str, object]:
        return {
            "triggered": self.triggered,
            "ok": self.ok,
            "reason": self.reason,
            "tilt_deg": self.tilt_deg,
            "xy_error_m": self.xy_error_m,
            "altitude_m": self.altitude_m,
            "trigger_throttle_pwm": self.trigger_throttle_pwm,
            "trigger_vz_mps": self.trigger_vz_mps,
            "trigger_z_err_m": self.trigger_z_err_m,
            "method": self.method,
            "error": self.error,
            "reset_at": self.reset_at,
        }


@dataclass
class ResetRecoveryState:
    """Episode recovery after a Gazebo crash/reset.

    This keeps the SDK socket alive, but pauses SDK control until Gazebo,
    ArduCopter state, and the bridge's local home reference are coherent again.
    """

    active: bool = False
    started_at: float = 0.0
    settle_until: float = 0.0
    baseline_until: float = 0.0
    reset_frame: int = -1
    home_rebased: bool = False
    handoff_waits: int = 0
    reset_info: dict[str, object] | None = None

    def phase(self, now: float) -> str:
        if not self.active:
            return "idle"
        if now < self.settle_until:
            return "settle"
        if now < self.baseline_until:
            return "baseline"
        return "done"


@dataclass
class PostResetGuardState:
    """Short SDK-suppression window immediately after a simulator reset."""

    active: bool = False
    until: float = 0.0
    reset_frame: int = -1
    reset_info: dict[str, object] | None = None


@dataclass
class EpisodeFailureTimers:
    """Debounce non-flip reset triggers."""

    xy_runaway_since: float | None = None
    ground_stuck_since: float | None = None

    def reset(self) -> None:
        self.xy_runaway_since = None
        self.ground_stuck_since = None


def maybe_reset_gazebo_on_episode_failure(
    args: argparse.Namespace,
    adapter: MavlinkAdapter,
    state,
    copter_state,
    shield,
    failure_timers: EpisodeFailureTimers,
    last_rc,
    last_reset_at: float,
) -> GazeboResetResult:
    """Reset Gazebo when the current rollout reaches a terminal condition."""
    tilt_deg = math.degrees(max(abs(state.roll), abs(state.pitch)))
    xy_error_m = math.hypot(copter_state.x_err, copter_state.y_err)
    altitude_m = copter_state.altitude_m
    now = time.time()

    reason = _episode_failure_reason(
        args=args,
        state=state,
        copter_state=copter_state,
        last_rc=last_rc,
        timers=failure_timers,
        now=now,
        tilt_deg=tilt_deg,
        xy_error_m=xy_error_m,
    )
    if reason is None:
        return GazeboResetResult(
            triggered=False,
            tilt_deg=tilt_deg,
            xy_error_m=xy_error_m,
            altitude_m=altitude_m,
        )
    if now - last_reset_at < args.reset_cooldown_s:
        return GazeboResetResult(
            triggered=False,
            reason=reason,
            tilt_deg=tilt_deg,
            xy_error_m=xy_error_m,
            altitude_m=altitude_m,
        )

    shield.reset()
    result = reset_gazebo_model_pose(args=args, adapter=adapter)
    if adapter.control == "rc":
        settle_rc = action_adapter_low_throttle()
        for _ in range(max(1, int(args.reset_settle_s * args.rate_hz))):
            adapter.send_rc_cmd(settle_rc)
            time.sleep(min(0.05, max(0.01, 1.0 / args.rate_hz)))

    result.triggered = True
    result.reason = reason
    result.tilt_deg = tilt_deg
    result.xy_error_m = xy_error_m
    result.altitude_m = altitude_m
    result.trigger_throttle_pwm = (
        int(getattr(last_rc, "throttle", 0)) if last_rc is not None else 0
    )
    result.trigger_vz_mps = float(copter_state.vz)
    result.trigger_z_err_m = float(copter_state.z_err)
    result.reset_at = time.time()
    return result


def action_adapter_low_throttle():
    from flight_bridge.command_types import RcCommand

    return RcCommand(roll=1500, pitch=1500, throttle=1000, yaw=1500)


def reset_gazebo_model_pose(args: argparse.Namespace, adapter: MavlinkAdapter) -> GazeboResetResult:
    """Reset the current simulator episode using the configured method."""
    method = getattr(args, "reset_method", "model-pose")
    if method == "restart-stack":
        return restart_sitl_stack(args, adapter)
    return reset_gazebo_model_pose_fast(
        world=args.gazebo_world,
        model=args.gazebo_model,
        z=args.reset_pose_z,
    )


def reset_gazebo_model_pose_fast(world: str, model: str, z: float) -> GazeboResetResult:
    """Reset a Gazebo model pose via gz service."""
    model_reset_cmd = [
        "gz", "service",
        "-s", f"/world/{world}/control",
        "--reqtype", "gz.msgs.WorldControl",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "1000",
        "--req", "reset: {model_only: true}",
    ]
    model_reset_result = _run_gz_service(model_reset_cmd)

    pose_request = (
        f'name: "{model}", '
        f'position: {{x: 0, y: 0, z: {z}}}, '
        'orientation: {w: 1, x: 0, y: 0, z: 0}'
    )
    pose_cmd = [
        "gz", "service",
        "-s", f"/world/{world}/set_pose",
        "--reqtype", "gz.msgs.Pose",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "1000",
        "--req", pose_request,
    ]
    pose_result = _run_gz_service(pose_cmd)
    if pose_result.ok:
        if model_reset_result.ok:
            pose_result.method = "model_reset_then_set_pose"
        else:
            pose_result.method = "set_pose_model_reset_failed"
            pose_result.error = f"model_reset failed: {model_reset_result.error}"
        return pose_result

    reset_cmd = [
        "gz", "service",
        "-s", f"/world/{world}/control",
        "--reqtype", "gz.msgs.WorldControl",
        "--reptype", "gz.msgs.Boolean",
        "--timeout", "1000",
        "--req", "reset: {all: true}",
    ]
    reset_result = _run_gz_service(reset_cmd)
    reset_result.method = "world_control_reset_all"
    if not reset_result.ok:
        errors = []
        if model_reset_result.error:
            errors.append(f"model_reset failed: {model_reset_result.error}")
        if pose_result.error:
            errors.append(f"set_pose failed: {pose_result.error}")
        if reset_result.error:
            errors.append(f"reset_all failed: {reset_result.error}")
        reset_result.error = "; ".join(errors) if errors else reset_result.error
    return reset_result


def restart_sitl_stack(args: argparse.Namespace, adapter: MavlinkAdapter) -> GazeboResetResult:
    """Restart Gazebo/SITL/bridge while keeping the SDK server/socket alive."""
    script = getattr(args, "sitl_start_script", None)
    if script is None:
        return GazeboResetResult(ok=False, method="restart_stack", error="missing start script")

    command = [str(script)]
    env = {
        "BUILD": "0",
        "GZ_GUI": "1",
        "GZ_BRIDGE_EXT_MOTOR_ENABLE": "0",
        "GZ_BRIDGE_SDK_ENABLE": "0",
    }
    try:
        completed = subprocess.run(
            command,
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=max(5.0, float(args.sitl_restart_timeout_s)),
            env={**os.environ, **env},
        )
    except Exception as exc:
        return GazeboResetResult(ok=False, method="restart_stack", error=str(exc))

    output = (completed.stdout + completed.stderr).strip()
    if completed.returncode != 0:
        return GazeboResetResult(ok=False, method="restart_stack", error=output)

    adapter._connected = False
    adapter._cached_state = None
    adapter._mav = None
    if not adapter.connect(getattr(args, "connect", "")):
        return GazeboResetResult(
            ok=False,
            method="restart_stack",
            error=f"stack restarted but MAVLink reconnect failed: {output[-500:]}",
        )

    return GazeboResetResult(ok=True, method="restart_stack", error=None)


def _episode_failure_reason(
    args: argparse.Namespace,
    state,
    copter_state,
    last_rc,
    timers: EpisodeFailureTimers,
    now: float,
    tilt_deg: float,
    xy_error_m: float,
) -> str | None:
    """Return the terminal reason once a reset condition has debounced."""
    if tilt_deg >= args.reset_tilt_deg:
        return "flip"

    if xy_error_m >= args.reset_xy_runaway_m:
        if timers.xy_runaway_since is None:
            timers.xy_runaway_since = now
        elif now - timers.xy_runaway_since >= args.reset_runaway_hold_s:
            return "xy_runaway"
    else:
        timers.xy_runaway_since = None

    throttle_pwm = getattr(last_rc, "throttle", 1000) if last_rc is not None else 1000
    ground_stuck = (
        throttle_pwm >= args.reset_ground_stuck_throttle_pwm
        and copter_state.altitude_m <= args.reset_ground_stuck_alt_m
        and abs(copter_state.vz) <= args.reset_ground_stuck_vz_mps
        and tilt_deg <= args.reset_recovery_tilt_deg
    )
    if ground_stuck:
        if timers.ground_stuck_since is None:
            timers.ground_stuck_since = now
        elif now - timers.ground_stuck_since >= args.reset_ground_stuck_hold_s:
            return "ground_stuck"
    else:
        timers.ground_stuck_since = None

    return None


def _run_gz_service(command: list[str]) -> GazeboResetResult:
    try:
        completed = subprocess.run(
            command,
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=2.0,
        )
    except Exception as exc:
        return GazeboResetResult(ok=False, error=str(exc))

    output = (completed.stdout + completed.stderr).strip()
    ok = completed.returncode == 0 and "true" in output.lower()
    return GazeboResetResult(ok=ok, error=None if ok else output)
