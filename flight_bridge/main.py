#!/usr/bin/env python3
"""Flight bridge entrypoint — Phase 0 (dry-run) and Phase 1 (SDK-in-the-loop).

Phase 0: Read simulated state → SDK inference → log. No control output.
Phase 1: Read vehicle state → build copter obs → SDK inference →
         action adapter → safety shield → MAVLink velocity command.
"""

from __future__ import annotations

if __package__ in {None, ""}:
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import argparse
import math
import socket
import time
from pathlib import Path

from flight_bridge.cli import default_log_path, parse_args, validate_common_args
from flight_bridge.cli_common import resolve_sdk_credentials
from flight_bridge.episode_reset import (
    EpisodeFailureTimers,
    PostResetGuardState,
    ResetRecoveryState,
    action_adapter_low_throttle,
    maybe_reset_gazebo_on_episode_failure,
    reset_gazebo_model_pose_fast,
)
from flight_bridge.gazebo_odometry import GazeboOdom, GazeboOdometryReader
from flight_bridge.jsonl_logger import JsonlLogger
from flight_bridge.mavlink_adapter import CommandLog, MavlinkAdapter
from flight_bridge.command_types import ShieldInfo
from flight_bridge.observation_builder import wrap_deg
from flight_bridge.phase0 import run_phase0
from flight_bridge.run_summary import (
    ActionSummary,
    format_frame_summary,
    should_print_summary,
    sleep_remaining,
)
from flight_bridge.sdk_client import make_inference_client


# Phase 1 imports (loaded on demand to keep Phase 0 dependency-free)
_STATE_BUILDER = None
_ACTION_ADAPTER = None
_SAFETY_SHIELD = None
_CONFIG = None


def _lazy_import_phase1():
    """Lazy-import Phase 1 modules (only when --phase 1 is used)."""
    global _STATE_BUILDER, _ACTION_ADAPTER, _SAFETY_SHIELD, _CONFIG
    if _STATE_BUILDER is None:
        from flight_bridge.action_adapter import ActionAdapter as _ACTION_ADAPTER
        from flight_bridge.config_loader import load_config as _CONFIG
        from flight_bridge.safety_shield import SafetyShield as _SAFETY_SHIELD
        from flight_bridge.state_builder import (
            CopterState,
            CopterStateBuilder,
            HoverFeedbackBuilder,
            build_hover_feedback,
        )
        _STATE_BUILDER = (
            CopterState,
            CopterStateBuilder,
            HoverFeedbackBuilder,
            build_hover_feedback,
        )


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> int:
    args = parse_args()

    if args.phase == 0:
        return run_phase0(args)
    else:
        return _main_phase1(args)


# ---------------------------------------------------------------------------
# Phase 1
# ---------------------------------------------------------------------------


def _main_phase1(args: argparse.Namespace) -> int:
    """Phase 1: full SDK-in-the-loop control.

    Pipeline:
      MavlinkAdapter.read_state() → CopterStateBuilder.build_hover_obs()
      → SDK.infer() → ActionAdapter.to_velocity_cmd()
      → SafetyShield.filter() → MavlinkAdapter.send_velocity_cmd()
      → JsonlLogger.write()
    """
    _lazy_import_phase1()
    CopterState, CopterStateBuilder, HoverFeedbackBuilder, build_hover_feedback = _STATE_BUILDER
    SafetyShield = _SAFETY_SHIELD
    ActionAdapter = _ACTION_ADAPTER
    load_config = _CONFIG

    validate_common_args(args)
    username, password = resolve_sdk_credentials(args)

    # Load config
    config = load_config(args.config, use_real=args.use_real_limits)
    limits = config.limits  # now respects use_real via AppConfig.use_real

    # Wire modules
    log_file = args.log_file or default_log_path(args.log_dir, "phase1")
    adapter = MavlinkAdapter(
        vehicle_type="copter", mode=args.mode, control=args.control,
    )
    state_builder = CopterStateBuilder()
    feedback_builder = HoverFeedbackBuilder(config.feedback)
    action_adapter = ActionAdapter(config.control, limits, config.copter)
    if getattr(args, "adapter_mode", "front-4") == "learned":
        gaia_config = getattr(args, "gaia_adapter_config", Path("flight_bridge/gaia_adapter.yaml"))
        if gaia_config.exists():
            action_adapter.load_learned(gaia_config)
            print(f"  learned adapter loaded from {gaia_config}")
        else:
            print(f"  WARNING: {gaia_config} not found, falling back to front-4")
    shield = SafetyShield(
        config.safety, limits, config.filter,
        hover_throttle_pwm=config.copter.hover_throttle + 1000,
    )
    action_summary = ActionSummary()

    # Connect to vehicle (no-op in dry-run)
    if args.mode == "sitl":
        connected = adapter.connect(args.connect)
        if not connected:
            raise SystemExit(
                f"Failed to connect to MAVLink at {args.connect or adapter.DEFAULT_CONNECTION}"
            )

    print(
        f"phase1 mode={args.mode} control={args.control} "
        f"sdk_mode={args.sdk_mode} residual_lambda={config.control.residual_lambda} "
        f"limits={'real' if config.use_real else 'sim'} log={log_file}"
    )

    try:
        inference_client = make_inference_client(
            sdk_mode=args.sdk_mode, host=args.host, port=args.port,
            timeout_s=args.timeout, username=username, password=password,
            resume=args.resume,
        )
        with inference_client as sdk, JsonlLogger(log_file) as logger:
            try:
                run_phase1_loop(
                    args, config, adapter, state_builder, action_adapter,
                    shield, feedback_builder, sdk, logger, action_summary,
                )
            finally:
                if args.mode == "sitl" and args.control == "rc":
                    for _ in range(8):
                        adapter.emergency_stop()
                        time.sleep(0.03)
    except ConnectionRefusedError as exc:
        raise SystemExit(
            f"Cannot connect to SDK server at {args.host}:{args.port}. "
            "Start ./examples/retina-server-x86_64 first, or use --sdk-mode mock."
        ) from exc
    except socket.timeout as exc:
        raise SystemExit(
            f"Timed out talking to SDK server at {args.host}:{args.port}."
        ) from exc

    print("final " + action_summary.format_line())
    return 0


def run_phase1_loop(
    args: argparse.Namespace,
    config,
    adapter: MavlinkAdapter,
    state_builder,
    action_adapter,
    shield,
    feedback_builder,
    sdk,
    logger: JsonlLogger,
    action_summary: ActionSummary,
) -> None:
    """Main Phase 1 control loop — SDK-in-the-loop RC residual or velocity control.

    RC path (--control rc, APM 2.74b primary):
      action → to_rc_command() → residual_blend_rc() → filter_rc() → send_rc_cmd()
    Velocity path (--control guided, modern ArduCopter):
      action → to_velocity_cmd() → residual_blend() → filter() → send_velocity_cmd()
    """
    _lazy_import_phase1()
    _CopterState, _CopterStateBuilder, _HoverFeedbackBuilder, build_hover_feedback = _STATE_BUILDER
    CopterState = _CopterState

    frame_index = 0
    period_s = 1.0 / args.rate_hz
    use_rc = (args.control == "rc")
    is_dry_run = (args.mode == "dry-run")
    use_gazebo_truth = (
        args.mode == "sitl" and getattr(args, "use_gazebo_truth", True)
    )
    gazebo_reader = (
        GazeboOdometryReader(args.gazebo_odom_topic)
        if use_gazebo_truth
        else None
    )
    if gazebo_reader is not None:
        gazebo_reader.start()

    # LOITER test: configure APM flight mode switch mapping via FLTMODE params.
    # APM 2.74b defaults all FLTMODE to 0 (STABILIZE), meaning ch5 position has
    # no effect. We set FLTMODE4=5 (LOITER) so ch5=1500 (range 1491-1621 →
    # readSwitch() position 3 → flight_modes[3]) maps to LOITER.
    if getattr(args, "loiter_test", False) and not is_dry_run:
        _configure_loiter_flight_modes(adapter)
        # Also configure ch7/ch8 as LOITER if available
        adapter.send_rc_override(1500, 1500, 1000, 1500, ch5=1000)
        time.sleep(0.1)

    # Pre-compute neutral/fallback commands. RC throttle baseline is updated
    # per-frame from altitude error after CopterState is available.
    baseline_rc = action_adapter.baseline_rc_neutral() if use_rc else None
    baseline_vel = action_adapter.baseline_hover_cmd() if not use_rc else None
    last_cmd_vel = baseline_vel  # velocity obs history
    last_safe_rc = baseline_rc
    last_rc_delta: list[float] = [0.0, 0.0, 0.0, 0.0]  # RC obs history
    last_shield_info = ShieldInfo()

    # Home position (set on first SITL frame)
    home_lat: float | None = None
    home_lon: float | None = None
    gazebo_home: tuple[float, float, float] | None = None
    last_gazebo_reset_at = 0.0
    last_rearm_attempt_at = 0.0
    reset_recovery = ResetRecoveryState()
    post_reset_guard = PostResetGuardState()
    failure_timers = EpisodeFailureTimers()

    while args.iterations <= 0 or frame_index < args.iterations:
        frame_started_at = time.perf_counter()

        # 1. Read vehicle state
        raw_state = adapter.read_state()
        now = time.time()
        gazebo_odom = gazebo_reader.latest(timeout=0.0) if gazebo_reader is not None else None
        if gazebo_odom is not None and gazebo_home is None:
            gazebo_home = (gazebo_odom.x, gazebo_odom.y, gazebo_odom.z)

        # 2. State freshness check (separate fast ATT/GLOBAL/VFR at 200ms,
        #    heartbeat at 1500ms per 1Hz typical rate)
        state_fresh = raw_state.is_state_fresh(
            now, fast_timeout_ms=200.0, heartbeat_timeout_ms=1500.0,
        )
        state_age_ms = raw_state.fast_state_age_ms(now)
        if state_age_ms is None:
            state_age_ms = 9999.0  # never received → stale
        gazebo_age_ms = (
            (now - gazebo_odom.updated_at) * 1000.0
            if gazebo_odom is not None else None
        )
        gazebo_fresh = (
            not use_gazebo_truth
            or (gazebo_age_ms is not None and gazebo_age_ms <= 300.0)
        )
        if use_gazebo_truth and not gazebo_fresh:
            gazebo_odom = None

        # 3. Build CopterState with real position errors
        copter_state = _flightstate_to_copterstate(
            raw_state, args.target_heading,
            target_alt_m=getattr(args, 'target_alt', 1.0),
            target_x_m=getattr(args, 'target_x', None),
            target_y_m=getattr(args, 'target_y', None),
            elapsed_s=time.monotonic() - adapter._started_at,
            is_dry_run=is_dry_run,
            home_lat=home_lat, home_lon=home_lon,
            gazebo_odom=gazebo_odom,
            gazebo_home=gazebo_home,
        )
        if use_rc:
            baseline_rc = _baseline_rc_for_state(action_adapter, copter_state)

        if reset_recovery.active:
            now_mono = time.monotonic()
            phase = reset_recovery.phase(now_mono)
            recovery_ready = (
                raw_state.armed
                and (reset_recovery.home_rebased or not args.reset_rebase_home)
                and _is_recovery_handoff_ready(copter_state, args)
            )
            if phase == "done" and recovery_ready:
                reset_recovery.active = False
                print("  reset recovery: SDK control resumed")
            else:
                if (
                    args.reset_rebase_home
                    and not reset_recovery.home_rebased
                    and phase != "settle"
                    and not is_dry_run
                    and (
                        (
                            use_gazebo_truth
                            and gazebo_home is not None
                            and _is_recovery_pose_stable(copter_state, args)
                        )
                        or (
                            not use_gazebo_truth
                            and raw_state.lat != 0
                            and raw_state.lon != 0
                            and _is_recovery_pose_stable(copter_state, args)
                        )
                    )
                ):
                    if not use_gazebo_truth:
                        home_lat = raw_state.lat
                        home_lon = raw_state.lon
                    reset_recovery.home_rebased = True

                if use_rc:
                    baseline_rc = _baseline_rc_for_state(action_adapter, copter_state)
                if (
                    use_rc
                    and phase != "settle"
                    and getattr(args, "auto_rearm", True)
                    and not raw_state.armed
                    and now_mono - last_rearm_attempt_at >= args.rearm_cooldown_s
                ):
                    last_rearm_attempt_at = now_mono
                    shield.reset()
                    adapter.send_rc_cmd(action_adapter_low_throttle())
                    rearm_ok = adapter.arm_vehicle(args.rearm_method)
                    print(
                        "  reset recovery rearm: "
                        f"ok={rearm_ok} method={args.rearm_method}"
                    )
                    raw_state = adapter.read_state()
                    copter_state = _flightstate_to_copterstate(
                        raw_state,
                        args.target_heading,
                        target_alt_m=getattr(args, 'target_alt', 1.0),
                        target_x_m=getattr(args, 'target_x', None),
                        target_y_m=getattr(args, 'target_y', None),
                        elapsed_s=time.monotonic() - adapter._started_at,
                        is_dry_run=is_dry_run,
                        home_lat=home_lat,
                        home_lon=home_lon,
                        gazebo_odom=gazebo_odom,
                        gazebo_home=gazebo_home,
                    )
                    baseline_rc = _baseline_rc_for_state(action_adapter, copter_state)

                if phase == "done":
                    reset_recovery.handoff_waits += 1

                recovery_rc = (
                    action_adapter_low_throttle()
                    if phase == "settle"
                    else baseline_rc or action_adapter.baseline_rc_neutral()
                )
                cmd_log = adapter.send_rc_cmd(recovery_rc)
                last_safe_rc = recovery_rc
                last_rc_delta = [0.0, 0.0, -1.0, 0.0] if phase == "settle" else [0.0, 0.0, 0.0, 0.0]
                last_shield_info = ShieldInfo(safety_triggered=True)
                _log_recovery_frame(
                    logger=logger,
                    frame_index=frame_index,
                    args=args,
                    raw_state=raw_state,
                    copter_state=copter_state,
                    gazebo_odom=gazebo_odom,
                    gazebo_home=gazebo_home,
                    state_age_ms=state_age_ms,
                    gazebo_age_ms=gazebo_age_ms,
                    baseline_rc=baseline_rc,
                    safe_rc=recovery_rc,
                    rc_delta=last_rc_delta,
                    feedback_builder=feedback_builder,
                    reset_recovery=reset_recovery,
                    cmd_log=cmd_log,
                    recovery_ready=recovery_ready,
                )
                frame_index += 1
                sleep_remaining(frame_started_at, period_s)
                continue

        if use_rc and getattr(args, "auto_rearm", True) and not raw_state.armed:
            now_mono = time.monotonic()
            rearm_attempted = False
            rearm_ok = False
            if now_mono - last_rearm_attempt_at >= args.rearm_cooldown_s:
                last_rearm_attempt_at = now_mono
                rearm_attempted = True
                shield.reset()
                feedback_builder.reset(pending_value=feedback_builder.terminal_value)
                adapter.send_rc_cmd(action_adapter_low_throttle())
                last_safe_rc = action_adapter_low_throttle()
                rearm_ok = adapter.arm_vehicle(args.rearm_method)

            last_rc_delta = [0.0, 0.0, -1.0, 0.0]
            last_shield_info = ShieldInfo(safety_triggered=True)
            logger.write({
                "ts": time.time(), "frame": frame_index,
                "phase": 1, "mode": args.mode, "control": args.control,
                "rc_control_mode": getattr(args, "rc_control_mode", "residual"),
                "vehicle_mode": raw_state.mode, "armed": raw_state.armed,
                "state": raw_state.to_log_dict(),
                "gazebo_odom": (
                    gazebo_odom.to_log_dict(gazebo_home)
                    if gazebo_odom is not None else None
                ),
                "copter_state": {
                    "x_err": copter_state.x_err, "y_err": copter_state.y_err,
                    "z_err": copter_state.z_err,
                    "vx": copter_state.vx, "vy": copter_state.vy, "vz": copter_state.vz,
                    "roll": copter_state.roll, "pitch": copter_state.pitch,
                    "yaw_err": copter_state.yaw_err,
                    "p": copter_state.p, "q": copter_state.q, "r": copter_state.r,
                    "altitude_m": copter_state.altitude_m,
                },
                "obs": None, "feedback": [feedback_builder.terminal_value],
                "feedback_diagnostics": feedback_builder.last_diagnostics.to_log_dict(),
                "action": [], "latency_ms": 0.0,
                "state_age_ms": state_age_ms,
                "gazebo_age_ms": gazebo_age_ms,
                "timeout_discard": True,
                "baseline_rc": baseline_rc.to_list() if baseline_rc else None,
                "candidate_command": None,
                "safe_command": [1500, 1500, 1000, 1500],
                "rc_delta": last_rc_delta,
                "gazebo_reset": None,
                "auto_rearm": {
                    "attempted": rearm_attempted,
                    "ok": rearm_ok,
                    "method": args.rearm_method,
                },
                "shield": {
                    "filtered": False,
                    "saturated": False,
                    "safety_triggered": True,
                    "clipped_fields": ["disarmed"],
                },
                "sent": False, "cmd_method": "AUTO_REARM",
                "error": None if rearm_ok or not rearm_attempted else "auto rearm failed",
            })
            if rearm_attempted:
                print(f"  auto-rearm: ok={rearm_ok} method={args.rearm_method}")
            sleep_remaining(frame_started_at, period_s)
            frame_index += 1
            continue

        # LOITER test: ch5-based mode switch.
        # Trigger is in the RC block below (LOITER_SWITCH_FRAME = 80).

        # Record home position on first valid SITL frame
        if not is_dry_run and home_lat is None and raw_state.lat != 0:
            home_lat = raw_state.lat
            home_lon = raw_state.lon

        gazebo_reset_info: dict[str, object] | None = None
        if use_rc and args.reset_on_flip:
            reset_result = maybe_reset_gazebo_on_episode_failure(
                args=args,
                adapter=adapter,
                state=raw_state,
                copter_state=copter_state,
                shield=shield,
                failure_timers=failure_timers,
                last_rc=last_safe_rc,
                last_reset_at=last_gazebo_reset_at,
            )
            if reset_result.triggered:
                last_gazebo_reset_at = reset_result.reset_at
                failure_timers.reset()
                action_summary.update_gazebo_reset()
                last_rc_delta = [0.0, 0.0, 0.0, 0.0]
                last_shield_info = ShieldInfo(safety_triggered=True)
                reset_feedback = [feedback_builder.terminal_value]
                feedback_builder.reset()
                gazebo_reset_info = reset_result.to_log_dict()
                home_lat = None
                home_lon = None
                gazebo_home = None
                if gazebo_reader is not None:
                    gazebo_reader.restart()
                    gazebo_odom = gazebo_reader.latest(timeout=2.0)
                    if gazebo_odom is not None:
                        gazebo_home = (gazebo_odom.x, gazebo_odom.y, gazebo_odom.z)
                if args.reset_recovery:
                    reset_recovery = ResetRecoveryState(
                        active=True,
                        started_at=time.monotonic(),
                        settle_until=time.monotonic() + args.reset_recovery_settle_s,
                        baseline_until=(
                            time.monotonic()
                            + args.reset_recovery_settle_s
                            + args.reset_recovery_baseline_s
                        ),
                        reset_frame=frame_index,
                        home_rebased=False,
                        reset_info=gazebo_reset_info,
                    )
                    post_reset_guard = PostResetGuardState()
                elif args.post_reset_guard_s > 0:
                    post_reset_guard = PostResetGuardState(
                        active=True,
                        until=time.monotonic() + args.post_reset_guard_s,
                        reset_frame=frame_index,
                        reset_info=gazebo_reset_info,
                    )
                reset_recovery_log = None
                if args.reset_recovery:
                    reset_recovery_log = {
                        "active": True,
                        "phase": "start",
                        "home_rebased": False,
                        "reset_frame": frame_index,
                    }
                elif post_reset_guard.active:
                    reset_recovery_log = {
                        "active": True,
                        "phase": "guard",
                        "reset_frame": frame_index,
                        "until": post_reset_guard.until,
                    }
                logger.write({
                    "ts": time.time(), "frame": frame_index,
                    "phase": 1, "mode": args.mode, "control": args.control,
                    "rc_control_mode": getattr(args, "rc_control_mode", "residual"),
                    "vehicle_mode": raw_state.mode, "armed": raw_state.armed,
                    "state": raw_state.to_log_dict(),
                    "gazebo_odom": (
                        gazebo_odom.to_log_dict(gazebo_home)
                        if gazebo_odom is not None else None
                    ),
                    "copter_state": {
                        "x_err": copter_state.x_err, "y_err": copter_state.y_err,
                        "z_err": copter_state.z_err,
                        "vx": copter_state.vx, "vy": copter_state.vy, "vz": copter_state.vz,
                        "roll": copter_state.roll, "pitch": copter_state.pitch,
                        "yaw_err": copter_state.yaw_err,
                        "p": copter_state.p, "q": copter_state.q, "r": copter_state.r,
                        "altitude_m": copter_state.altitude_m,
                    },
                    "obs": None, "feedback": reset_feedback,
                    "feedback_diagnostics": feedback_builder.last_diagnostics.to_log_dict(),
                    "action": [], "latency_ms": 0.0,
                    "state_age_ms": state_age_ms,
                    "gazebo_age_ms": gazebo_age_ms,
                    "timeout_discard": False,
                    "baseline_rc": baseline_rc.to_list() if baseline_rc else None,
                    "candidate_command": None,
                    "safe_command": [1500, 1500, 1000, 1500],
                    "rc_delta": [0.0, 0.0, -1.0, 0.0],
                    "gazebo_reset": gazebo_reset_info,
                    "reset_recovery": reset_recovery_log,
                    "shield": {
                        "filtered": False,
                        "saturated": False,
                        "safety_triggered": True,
                        "clipped_fields": ["gazebo_reset"],
                    },
                    "sent": False, "cmd_method": "gazebo_reset",
                    "error": reset_result.error,
                })
                print(
                    "  gazebo reset: "
                    f"reason={reset_result.reason} "
                    f"tilt_deg={reset_result.tilt_deg:.1f} "
                    f"xy_error_m={reset_result.xy_error_m:.1f} "
                    f"ok={reset_result.ok} method={reset_result.method}"
                )
                sleep_remaining(frame_started_at, period_s)
                frame_index += 1
                continue

        if post_reset_guard.active:
            if time.monotonic() >= post_reset_guard.until:
                post_reset_guard.active = False
                print("  post-reset guard: SDK control resumed")
            else:
                guard_rc = baseline_rc or action_adapter.baseline_rc_neutral()
                cmd_log = adapter.send_rc_cmd(guard_rc) if use_rc else adapter.send_velocity_cmd(baseline_vel)
                if use_rc:
                    last_safe_rc = guard_rc
                    last_rc_delta = [0.0, 0.0, 0.0, 0.0]
                else:
                    last_cmd_vel = baseline_vel
                last_shield_info = ShieldInfo(safety_triggered=True)
                logger.write({
                    "ts": time.time(), "frame": frame_index,
                    "phase": 1, "mode": args.mode, "control": args.control,
                    "rc_control_mode": getattr(args, "rc_control_mode", "residual"),
                    "vehicle_mode": raw_state.mode, "armed": raw_state.armed,
                    "state": raw_state.to_log_dict(),
                    "gazebo_odom": (
                        gazebo_odom.to_log_dict(gazebo_home)
                        if gazebo_odom is not None else None
                    ),
                    "copter_state": _copter_state_to_log(copter_state),
                    "obs": None,
                    "feedback": [feedback_builder.neutral_value],
                    "feedback_diagnostics": feedback_builder.last_diagnostics.to_log_dict(),
                    "action": [], "latency_ms": 0.0,
                    "state_age_ms": state_age_ms,
                    "gazebo_age_ms": gazebo_age_ms,
                    "timeout_discard": True,
                    "baseline_rc": baseline_rc.to_list() if baseline_rc else None,
                    "candidate_command": None,
                    "safe_command": guard_rc.to_list() if hasattr(guard_rc, "to_list") else None,
                    "rc_delta": [0.0, 0.0, 0.0, 0.0] if use_rc else None,
                    "gazebo_reset": post_reset_guard.reset_info,
                    "reset_recovery": {
                        "active": True,
                        "phase": "guard",
                        "reset_frame": post_reset_guard.reset_frame,
                        "until": post_reset_guard.until,
                    },
                    "shield": {
                        "filtered": False,
                        "saturated": False,
                        "safety_triggered": True,
                        "clipped_fields": ["post_reset_guard"],
                    },
                    "sent": cmd_log.sent,
                    "cmd_method": "POST_RESET_GUARD",
                    "error": cmd_log.error,
                })
                frame_index += 1
                sleep_remaining(frame_started_at, period_s)
                continue

        # 4. Build obs + feedback
        if use_rc:
            # RC path: use RC delta as last_cmd (already [-1,1]), divisor=1.0
            last_cmd_for_obs = last_rc_delta
            obs = state_builder.build_hover_obs(copter_state, last_cmd_for_obs, rc_mode=True)
        else:
            last_cmd_for_obs = last_cmd_vel.to_list() if last_cmd_vel else [0.0]*4
            obs = state_builder.build_hover_obs(copter_state, last_cmd_for_obs)
        feedback = feedback_builder.build(
            copter_state,
            shield_info=last_shield_info,
            last_cmd=last_cmd_for_obs,
        )

        # 5. SDK inference
        action: list[float] = []
        latency_ms = 0.0
        error_text: str | None = None
        cmd_log = CommandLog()
        timeout_discard = False
        shield_info = ShieldInfo()  # track shield state across all branches

        try:
            if use_rc and args.rc_control_mode == "baseline-only":
                if getattr(args, "loiter_test", False):
                    # LOITER test: STABILIZE takeoff → ch5→LOITER → neutral RC.
                    # APM 2.74b source patched: get_throttle_rate() initializes
                    # z_rate_error = target - actual (instead of 0) on mode change,
                    # so the rate controller responds immediately.
                    #   Phase 1 (frames 0-79):   STABILIZE takeoff, baseline PD, ch5=1000
                    #   Phase 2 (frames 80+):     ch5→1500, baseline PD during debounce.
                    #                             Once LOITER confirmed → neutral RC.
                    # Switch to LOITER when baseline PD reaches steady hover.
                    # Condition: near target altitude AND low vertical speed.
                    # This avoids LOITER entry during PD overshoot/correction,
                    # which would set controller_desired_alt far from target.
                    LOITER_MIN_FRAME = 60  # minimum takeoff before switching

                    loiter_active = getattr(args, "_loiter_switched", False)
                    if not loiter_active and frame_index >= LOITER_MIN_FRAME:
                        alt_ok = abs(copter_state.z_err) < 1.0   # within 1m of target
                        vz_ok = abs(copter_state.vz) < 0.5        # near-zero climb rate
                        if alt_ok and vz_ok:
                            args._loiter_switched = True
                            print(f"  loiter-test: frame={frame_index} ch5→1500, "
                                  f"alt={copter_state.altitude_m:.1f}m vz={copter_state.vz:.1f}m/s")

                    ch5 = 1500 if loiter_active else 1000
                    _mode_confirmed = getattr(args, "_loiter_mode_confirmed", False)

                    # Track when LOITER mode is confirmed
                    if loiter_active and not _mode_confirmed and raw_state.mode == "LOITER":
                        args._loiter_mode_confirmed = True
                        print(f"  loiter-test: frame={frame_index} mode=LOITER confirmed, "
                              f"alt={copter_state.altitude_m:.1f}m vz={copter_state.vz:.1f}m/s")
                        _mode_confirmed = True

                    if _mode_confirmed:
                        # Phase 3: full neutral RC → LOITER holds position+altitude
                        adapter.send_rc_override(1500, 1500, 1500, 1500, ch5=ch5)
                        shield_info = ShieldInfo()
                        safe_rc = action_adapter.baseline_rc_neutral()
                        cmd_log = CommandLog(candidate=safe_rc, safe=safe_rc, sent=True, method="LOITER_NEUTRAL")
                        last_safe_rc = safe_rc
                        action_summary.update_sent()
                        last_rc_delta = [0.0, 0.0, 0.0, 0.0]
                    elif loiter_active:
                        # Phase 2: baseline PD + ch5=1500, maintain hover during APM debounce
                        safe_rc, shield_info = shield.filter_rc(baseline_rc)
                        action_summary.update_shield(
                            clipped=len(shield_info.clipped_fields) > 0,
                            filtered=shield_info.filtered,
                            safety=shield_info.safety_triggered,
                        )
                        adapter.send_rc_override(
                            safe_rc.roll, safe_rc.pitch, safe_rc.throttle, safe_rc.yaw,
                            ch5=ch5,
                        )
                        cmd_log = CommandLog(candidate=safe_rc, safe=safe_rc, sent=True, method="LOITER_TRANSITION")
                        last_safe_rc = safe_rc
                        action_summary.update_sent()
                        neutral_rc = action_adapter.baseline_rc_neutral()
                        last_rc_delta = [
                            (safe_rc.roll - neutral_rc.roll) / 500.0,
                            (safe_rc.pitch - neutral_rc.pitch) / 500.0,
                            (safe_rc.throttle - neutral_rc.throttle) / 500.0,
                            (safe_rc.yaw - neutral_rc.yaw) / 500.0,
                        ]
                    else:
                        # Phase 1: STABILIZE takeoff, baseline PD, ch5=1000
                        safe_rc, shield_info = shield.filter_rc(baseline_rc)
                        action_summary.update_shield(
                            clipped=len(shield_info.clipped_fields) > 0,
                            filtered=shield_info.filtered,
                            safety=shield_info.safety_triggered,
                        )
                        adapter.send_rc_override(
                            safe_rc.roll, safe_rc.pitch, safe_rc.throttle, safe_rc.yaw,
                            ch5=ch5,
                        )
                        cmd_log = CommandLog(candidate=safe_rc, safe=safe_rc, sent=True, method="BASELINE_ONLY")
                        last_safe_rc = safe_rc
                        action_summary.update_sent()
                        neutral_rc = action_adapter.baseline_rc_neutral()
                        last_rc_delta = [
                            (safe_rc.roll - neutral_rc.roll) / 500.0,
                            (safe_rc.pitch - neutral_rc.pitch) / 500.0,
                            (safe_rc.throttle - neutral_rc.throttle) / 500.0,
                            (safe_rc.yaw - neutral_rc.yaw) / 500.0,
                        ]
                else:
                    safe_rc, shield_info = shield.filter_rc(baseline_rc)
                    action_summary.update_shield(
                        clipped=len(shield_info.clipped_fields) > 0,
                        filtered=shield_info.filtered,
                        safety=shield_info.safety_triggered,
                    )
                    cmd_log = adapter.send_rc_cmd(safe_rc)
                    cmd_log.method = "BASELINE_ONLY"
                    last_safe_rc = safe_rc
                    if cmd_log.sent:
                        action_summary.update_sent()
                    neutral_rc = action_adapter.baseline_rc_neutral()
                    last_rc_delta = [
                        (safe_rc.roll - neutral_rc.roll) / 500.0,
                        (safe_rc.pitch - neutral_rc.pitch) / 500.0,
                        (safe_rc.throttle - neutral_rc.throttle) / 500.0,
                        (safe_rc.yaw - neutral_rc.yaw) / 500.0,
                    ]
            else:
                infer_started_at = time.perf_counter()
                action = sdk.infer(obs, feedback)
                latency_ms = (time.perf_counter() - infer_started_at) * 1000.0
                action_summary.update(action, latency_ms)

                # Timeout / state-age enforcement (§9.4)
                if (
                    not shield.should_accept_action(latency_ms, state_age_ms)
                    or not state_fresh
                    or not gazebo_fresh
                ):
                    timeout_discard = True
                    shield_info.safety_triggered = True
                    action_summary.update_shield(clipped=False, filtered=False, safety=True)
                    # Fall back to baseline (hover) — also reset last command history
                    if use_rc:
                        cmd_log = adapter.send_rc_cmd(baseline_rc)
                        last_safe_rc = baseline_rc
                        last_rc_delta = [0.0, 0.0, 0.0, 0.0]
                    else:
                        cmd_log = adapter.send_velocity_cmd(baseline_vel)
                        last_cmd_vel = baseline_vel
                elif use_rc:
                    # RC residual path (APM 2.74b primary)
                    if args.rc_control_mode == "sdk-direct":
                        # Pure SDK path: action[0..3] become absolute RC commands.
                        # Safety still prevents the model from cutting throttle
                        # below the takeoff/hover baseline before it is airborne.
                        blended_rc = action_adapter.to_rc_command(action)
                        blended_rc = _protect_low_altitude_throttle(
                            blended_rc,
                            baseline_rc,
                            copter_state,
                            target_alt_m=getattr(args, "target_alt", 1.0),
                        )
                        blended_rc = _protect_low_altitude_attitude(
                            blended_rc,
                            copter_state,
                            level_until_alt_m=args.sdk_direct_level_until_alt_m,
                        )
                    elif action_adapter.has_learned:
                        # Learned W-matrix path: action → W@action → λ·delta → final PWM
                        sdk_rc = action_adapter.gaia_action_to_rc(baseline_rc, action)
                        blended_rc = sdk_rc  # gaia_action_to_rc already applies lambda
                    else:
                        # Front-4 hard mapping
                        sdk_rc = action_adapter.to_rc_command(action)
                        blended_rc = action_adapter.residual_blend_rc(baseline_rc, sdk_rc)
                    safe_rc, shield_info = shield.filter_rc(blended_rc)
                    action_summary.update_shield(
                        clipped=len(shield_info.clipped_fields) > 0,
                        filtered=shield_info.filtered,
                        safety=shield_info.safety_triggered,
                    )
                    cmd_log = adapter.send_rc_cmd(safe_rc)
                    last_safe_rc = safe_rc
                    if cmd_log.sent:
                        action_summary.update_sent()
                    # Save RC delta for next frame's obs
                    neutral_rc = action_adapter.baseline_rc_neutral()
                    last_rc_delta = [
                        (safe_rc.roll - neutral_rc.roll) / 500.0,       # normalized [-1,1]
                        (safe_rc.pitch - neutral_rc.pitch) / 500.0,
                        (safe_rc.throttle - neutral_rc.throttle) / 500.0,
                        (safe_rc.yaw - neutral_rc.yaw) / 500.0,
                    ]
                else:
                    # Velocity path (modern ArduCopter)
                    sdk_vel = action_adapter.to_velocity_cmd(action)
                    blended_vel = action_adapter.residual_blend(baseline_vel, sdk_vel)
                    safe_vel, shield_info = shield.filter(blended_vel)
                    action_summary.update_shield(
                        clipped=len(shield_info.clipped_fields) > 0,
                        filtered=shield_info.filtered,
                        safety=shield_info.safety_triggered,
                    )
                    cmd_log = adapter.send_velocity_cmd(safe_vel)
                    if cmd_log.sent:
                        action_summary.update_sent()
                    last_cmd_vel = safe_vel

        except RuntimeError as exc:
            error_text = str(exc)
            # On error, fall back to baseline and reset last command history
            if use_rc:
                cmd_log = adapter.send_rc_cmd(baseline_rc)
                last_safe_rc = baseline_rc
                last_rc_delta = [0.0, 0.0, 0.0, 0.0]
            else:
                cmd_log = adapter.send_velocity_cmd(baseline_vel)
                last_cmd_vel = baseline_vel
            shield_info.safety_triggered = True

        last_shield_info = shield_info

        # 6. Log
        _log_cmd_candidate = None
        _log_cmd_safe = None
        if cmd_log.candidate is not None:
            if hasattr(cmd_log.candidate, 'to_list'):
                _log_cmd_candidate = cmd_log.candidate.to_list()
            else:
                _log_cmd_candidate = str(cmd_log.candidate)
        if cmd_log.safe is not None:
            if hasattr(cmd_log.safe, 'to_list'):
                _log_cmd_safe = cmd_log.safe.to_list()
            else:
                _log_cmd_safe = str(cmd_log.safe)

        _baseline_rc_list = baseline_rc.to_list() if baseline_rc else None
        _rc_delta_list = None
        if _log_cmd_safe and _baseline_rc_list and len(_log_cmd_safe) == 4:
            _rc_delta_list = [
                (float(_log_cmd_safe[i]) - float(_baseline_rc_list[i])) / 500.0
                for i in range(4)
            ]

        # Build copter_state summary for analysis
        _copter_state_log = _copter_state_to_log(copter_state)

        _shield_log = {
            "filtered": shield_info.filtered,
            "saturated": shield_info.saturated,
            "safety_triggered": shield_info.safety_triggered,
            "clipped_fields": list(shield_info.clipped_fields),
        }

        logger.write({
            "ts": time.time(), "frame": frame_index,
            "phase": 1, "mode": args.mode, "control": args.control,
            "rc_control_mode": getattr(args, "rc_control_mode", "residual"),
            "vehicle_mode": raw_state.mode, "armed": raw_state.armed,
            "state": raw_state.to_log_dict(),
            "gazebo_odom": (
                gazebo_odom.to_log_dict(gazebo_home)
                if gazebo_odom is not None else None
            ),
            "copter_state": _copter_state_log,
            "obs": obs, "feedback": feedback,
            "feedback_diagnostics": feedback_builder.last_diagnostics.to_log_dict(),
            "action": action, "latency_ms": latency_ms,
            "state_age_ms": state_age_ms,
            "gazebo_age_ms": gazebo_age_ms,
            "timeout_discard": timeout_discard,
            "baseline_rc": _baseline_rc_list,
            "candidate_command": _log_cmd_candidate,
            "safe_command": _log_cmd_safe,
            "rc_delta": _rc_delta_list,
            "gazebo_reset": gazebo_reset_info,
            "reset_recovery": None,
            "shield": _shield_log,
            "sent": cmd_log.sent, "cmd_method": cmd_log.method,
            "error": error_text or cmd_log.error,
        })

        if error_text is not None:
            raise RuntimeError(error_text)

        if should_print_summary(frame_index, args.summary_every):
            print(format_frame_summary(
                frame_index, raw_state.mode, obs, action, latency_ms,
            ))
            s = shield
            if s.is_saturated or s.safety_flag:
                print(f"  shield: saturated={s.is_saturated} safety={s.safety_flag}")
            if timeout_discard:
                print(f"  timeout: latency_ms={latency_ms:.1f} state_age_ms={state_age_ms:.0f} → DISCARDED")

        frame_index += 1
        sleep_remaining(frame_started_at, period_s)


def _flightstate_to_copterstate(
    fs, target_heading_deg: float,
    target_alt_m: float = 1.0,
    target_x_m: float | None = None,
    target_y_m: float | None = None,
    elapsed_s: float = 0.0,
    is_dry_run: bool = True,
    home_lat: float | None = None,
    home_lon: float | None = None,
    gazebo_odom: GazeboOdom | None = None,
    gazebo_home: tuple[float, float, float] | None = None,
) -> "CopterState":
    """Convert MAVLink FlightState to body-FLU CopterState for obs building.

    Dry-run: simulates position drift so obs/feedback are non-zero.
    SITL:    computes real errors from current position vs home/target,
             rotated into body-FLU using yaw.
    """
    _lazy_import_phase1()
    CopterState = _STATE_BUILDER[0]

    heading_error = wrap_deg(target_heading_deg - fs.heading)
    import math

    if gazebo_odom is not None and gazebo_home is not None:
        target_x = gazebo_home[0] if target_x_m is None else target_x_m
        target_y = gazebo_home[1] if target_y_m is None else target_y_m
        target_z = gazebo_home[2] + target_alt_m if target_x_m is None and target_y_m is None else target_alt_m
        rel_x = gazebo_odom.x - target_x
        rel_y = gazebo_odom.y - target_y
        rel_z = gazebo_odom.z - target_z
        cos_yaw = math.cos(gazebo_odom.yaw)
        sin_yaw = math.sin(gazebo_odom.yaw)
        x_err = rel_x * cos_yaw + rel_y * sin_yaw
        y_err = rel_x * sin_yaw - rel_y * cos_yaw
        z_err = -rel_z
        vx = gazebo_odom.vx * cos_yaw + gazebo_odom.vy * sin_yaw
        vy = gazebo_odom.vx * sin_yaw - gazebo_odom.vy * cos_yaw
        return CopterState(
            x_err=x_err, y_err=y_err, z_err=z_err,
            vx=vx, vy=vy, vz=gazebo_odom.vz,
            roll=gazebo_odom.roll, pitch=gazebo_odom.pitch,
            yaw_err=math.radians(heading_error),
            p=gazebo_odom.wx, q=gazebo_odom.wy, r=gazebo_odom.wz,
            altitude_m=gazebo_odom.z,
            armed=fs.armed, mode=fs.mode,
        )

    if is_dry_run:
        # Simulated drift for testing (so SDK sees non-zero feedback)
        x_err = math.sin(elapsed_s * 0.3) * 0.5
        y_err = math.cos(elapsed_s * 0.4) * 0.4
        z_err = target_alt_m - fs.relative_alt
        vx = fs.groundspeed  # simplified — dry-run has no NED→body transform
        vy = 0.0
    elif home_lat is not None and home_lon is not None:
        # Real SITL position error in body-FLU
        # Lat/Lon → meters (approximate, for local frame)
        lat_to_m = 111320.0
        lon_to_m = 111320.0 * math.cos(math.radians(home_lat))
        dx_world = (fs.lat - home_lat) * lat_to_m
        dy_world = (fs.lon - home_lon) * lon_to_m
        # Rotate world-frame error to body-FLU using yaw
        cos_yaw = math.cos(fs.yaw)
        sin_yaw = math.sin(fs.yaw)
        x_err = dx_world * cos_yaw + dy_world * sin_yaw     # body forward
        y_err = dx_world * sin_yaw - dy_world * cos_yaw     # body left (verified: yaw=0, east→negative)
        z_err = target_alt_m - fs.relative_alt
        # NED velocity → body-FLU velocity
        v_n = fs.groundspeed * math.cos(math.radians(fs.heading))  # approx
        v_e = fs.groundspeed * math.sin(math.radians(fs.heading))
        vx = v_n * cos_yaw + v_e * sin_yaw                  # body forward
        vy = v_n * sin_yaw - v_e * cos_yaw                  # body left
    else:
        # SITL without home position yet — zeros
        x_err = 0.0
        y_err = 0.0
        z_err = target_alt_m - fs.relative_alt
        vx = 0.0
        vy = 0.0

    return CopterState(
        x_err=x_err, y_err=y_err, z_err=z_err,
        vx=vx, vy=vy, vz=fs.climb,  # VFR_HUD.climb +up, body-FLU vz +up
        roll=fs.roll, pitch=fs.pitch,
        yaw_err=math.radians(heading_error),
        p=fs.roll_speed, q=fs.pitch_speed, r=fs.yaw_speed,
        altitude_m=fs.relative_alt,
        armed=fs.armed, mode=fs.mode,
    )


def _is_recovery_pose_stable(state, args: argparse.Namespace) -> bool:
    """True when the unified copter state is calm enough to rebase home."""
    tilt_deg = math.degrees(max(abs(state.roll), abs(state.pitch)))
    return (
        tilt_deg <= args.reset_recovery_tilt_deg
        and abs(state.vz) <= 0.5
    )


def _is_recovery_handoff_ready(state, args: argparse.Namespace) -> bool:
    """True when baseline recovery has reached a sane hover handoff state."""
    return (
        _is_recovery_pose_stable(state, args)
        and abs(state.z_err) <= args.reset_recovery_alt_error_m
    )


def _log_recovery_frame(
    logger: JsonlLogger,
    frame_index: int,
    args: argparse.Namespace,
    raw_state,
    copter_state,
    gazebo_odom: GazeboOdom | None,
    gazebo_home: tuple[float, float, float] | None,
    state_age_ms: float,
    gazebo_age_ms: float | None,
    baseline_rc,
    safe_rc,
    rc_delta: list[float],
    feedback_builder,
    reset_recovery: ResetRecoveryState,
    cmd_log: CommandLog,
    recovery_ready: bool,
) -> None:
    phase = reset_recovery.phase(time.monotonic())
    logger.write({
        "ts": time.time(), "frame": frame_index,
        "phase": 1, "mode": args.mode, "control": args.control,
        "rc_control_mode": getattr(args, "rc_control_mode", "residual"),
        "vehicle_mode": raw_state.mode, "armed": raw_state.armed,
        "state": raw_state.to_log_dict(),
        "gazebo_odom": (
            gazebo_odom.to_log_dict(gazebo_home)
            if gazebo_odom is not None else None
        ),
        "copter_state": _copter_state_to_log(copter_state),
        "obs": None, "feedback": [feedback_builder.terminal_value],
        "feedback_diagnostics": None,
        "action": [], "latency_ms": 0.0,
        "state_age_ms": state_age_ms,
        "gazebo_age_ms": gazebo_age_ms,
        "timeout_discard": True,
        "baseline_rc": baseline_rc.to_list() if baseline_rc else None,
        "candidate_command": (
            cmd_log.candidate.to_list()
            if hasattr(cmd_log.candidate, "to_list")
            else None
        ),
        "safe_command": safe_rc.to_list() if hasattr(safe_rc, "to_list") else None,
        "rc_delta": rc_delta,
        "gazebo_reset": reset_recovery.reset_info,
        "reset_recovery": {
            "active": True,
            "phase": phase,
            "home_rebased": reset_recovery.home_rebased,
            "recovery_ready": recovery_ready,
            "handoff_waits": reset_recovery.handoff_waits,
            "reset_frame": reset_recovery.reset_frame,
            "started_at": reset_recovery.started_at,
        },
        "shield": {
            "filtered": False,
            "saturated": False,
            "safety_triggered": True,
            "clipped_fields": ["reset_recovery"],
        },
        "sent": cmd_log.sent,
        "cmd_method": f"RESET_RECOVERY_{phase.upper()}",
        "error": cmd_log.error,
    })


def _copter_state_to_log(copter_state) -> dict[str, float]:
    return {
        "x_err": copter_state.x_err, "y_err": copter_state.y_err,
        "z_err": copter_state.z_err,
        "vx": copter_state.vx, "vy": copter_state.vy, "vz": copter_state.vz,
        "roll": copter_state.roll, "pitch": copter_state.pitch,
        "yaw_err": copter_state.yaw_err,
        "p": copter_state.p, "q": copter_state.q, "r": copter_state.r,
        "altitude_m": copter_state.altitude_m,
    }


def _baseline_rc_for_state(action_adapter, copter_state):
    """Build the deterministic hover baseline from current state."""
    return action_adapter.baseline_rc_for_altitude(
        copter_state.z_err,
        copter_state.vz,
        x_err_m=copter_state.x_err,
        y_err_m=copter_state.y_err,
        vx_mps=copter_state.vx,
        vy_mps=copter_state.vy,
        altitude_m=copter_state.altitude_m,
    )


def _protect_low_altitude_throttle(cmd, baseline_rc, copter_state, target_alt_m: float):
    """Prevent sdk-direct from cutting throttle before the vehicle is airborne."""
    if baseline_rc is None:
        return cmd
    needs_climb = (
        copter_state.altitude_m < min(0.30, target_alt_m * 0.5)
        or copter_state.z_err > 0.35
    )
    if not needs_climb or cmd.throttle >= baseline_rc.throttle:
        return cmd

    from flight_bridge.command_types import RcCommand

    return RcCommand(
        roll=cmd.roll,
        pitch=cmd.pitch,
        throttle=baseline_rc.throttle,
        yaw=cmd.yaw,
    )


def _protect_low_altitude_attitude(cmd, copter_state, level_until_alt_m: float):
    """Hold lateral/yaw sticks neutral during the takeoff curriculum gate."""
    if copter_state.altitude_m >= level_until_alt_m:
        return cmd

    from flight_bridge.command_types import RcCommand

    return RcCommand(
        roll=1500,
        pitch=1500,
        throttle=cmd.throttle,
        yaw=1500,
    )


def _current_feedback_value(feedback_builder) -> float:
    return float(getattr(feedback_builder.last_diagnostics, "value", 0.5))


def _configure_loiter_flight_modes(adapter: MavlinkAdapter) -> None:
    """Configure APM parameters so ch5 can switch to LOITER and hold altitude.

    APM 2.74b readSwitch() maps ch5 PWM to a switch position:
      pos 0 (< 1231)   → flight_modes[0] = FLTMODE1  → STABILIZE (ch5=1000)
      pos 1 (1231-1361) → flight_modes[1] = FLTMODE2
      pos 2 (1361-1491) → flight_modes[2] = FLTMODE3
      pos 3 (1491-1621) → flight_modes[3] = FLTMODE4  → LOITER (ch5=1500) ← KEY
      pos 4 (1621-1750) → flight_modes[4] = FLTMODE5
      pos 5 (> 1750)    → flight_modes[5] = FLTMODE6

    All default to 0 (STABILIZE), so ch5 cannot change modes.  We set
    FLTMODE4=5 (LOITER) so ch5=1500 switches to LOITER after debounce.

    Also sets THR_ALT_I (default 0.0 → no integral, cannot compensate for
    hover-throttle offset) so the altitude-hold controller accumulates
    steady-state correction.
    """
    mav = getattr(adapter, "_mav", None)
    if mav is None:
        return

    import time as _time
    from pymavlink import mavutil as _mavutil

    settings = {
        # Flight mode switch mapping
        "FLTMODE1": (0, _mavutil.mavlink.MAV_PARAM_TYPE_INT8),
        "FLTMODE2": (1, _mavutil.mavlink.MAV_PARAM_TYPE_INT8),
        "FLTMODE3": (2, _mavutil.mavlink.MAV_PARAM_TYPE_INT8),
        "FLTMODE4": (5, _mavutil.mavlink.MAV_PARAM_TYPE_INT8),  # LOITER
        "FLTMODE5": (0, _mavutil.mavlink.MAV_PARAM_TYPE_INT8),
        "FLTMODE6": (0, _mavutil.mavlink.MAV_PARAM_TYPE_INT8),
        # Altitude-hold PI — gentle tuning to prevent overshoot/oscillation.
        # I term must unwind from max quickly enough to avoid overshoot.
        # With THR_ALT_I=0.04, IMAX=25: unwinds in ~6s at -100cm error.
        # P=0.3: max climb rate from 3m error = 90 cm/s, slow enough to stop.
        "THR_ALT_P": (0.25, _mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
        "THR_ALT_I": (0.03, _mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
        "THR_ALT_IMAX": (10.0, _mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
        # Inner rate loop — moderate gain with good damping
        "THR_RATE_P": (3.0, _mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
        "THR_RATE_I": (0.03, _mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
        "THR_RATE_D": (0.20, _mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
        "THR_RATE_IMAX": (80.0, _mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
        "PILOT_VELZ_MAX": (250.0, _mavutil.mavlink.MAV_PARAM_TYPE_REAL32),
    }

    configured = 0
    for name, (value, ptype) in settings.items():
        mav.mav.param_set_send(
            mav.target_system, mav.target_component,
            name.encode(), float(value), ptype,
        )
        _time.sleep(0.1)
        configured += 1

    if configured:
        print(f"  loiter-test: configured {configured} params "
              f"(FLTMODE4=5, THR_ALT_P=0.3 I=0.04 IMAX=25)")


if __name__ == "__main__":
    raise SystemExit(main())
