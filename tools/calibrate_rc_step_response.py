#!/usr/bin/env python3
"""Calibrate RC roll/pitch step response in Gazebo SITL.

This script stops the flight bridge, connects directly to ArduCopter SITL,
and measures Gazebo odometry velocity response to small RC roll/pitch steps.
It is intended for mapping RC stick axes to Gazebo x/y motion before tuning
the hover baseline.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import subprocess
import sys
import time
from pathlib import Path

if __package__ in {None, ""}:
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from flight_bridge.command_types import RcCommand
from flight_bridge.action_adapter import ActionAdapter
from flight_bridge.config_loader import load_config
from flight_bridge.episode_reset import reset_gazebo_model_pose_fast
from flight_bridge.gazebo_odometry import GazeboOdometryReader
from flight_bridge.jsonl_logger import JsonlLogger
from flight_bridge.mavlink_adapter import MavlinkAdapter


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--connect", default="tcp:127.0.0.1:5760")
    parser.add_argument("--log-file", type=Path, default=None)
    parser.add_argument("--world", default="arducopter_x3")
    parser.add_argument("--model", default="x3")
    parser.add_argument("--reset-z", type=float, default=0.35)
    parser.add_argument("--hover-pwm", type=int, default=1700)
    parser.add_argument("--takeoff-pwm", type=int, default=1760)
    parser.add_argument("--step-pwm", type=int, default=60)
    parser.add_argument("--target-alt", type=float, default=2.0)
    parser.add_argument("--takeoff-timeout-s", type=float, default=10.0)
    parser.add_argument("--takeoff-ready-alt-m", type=float, default=None,
                        help="Altitude considered ready before hover gate; default is target-alt - 0.25")
    parser.add_argument("--pre-low-s", type=float, default=0.4)
    parser.add_argument("--reset-settle-s", type=float, default=1.0,
                        help="Low-throttle settle time after Gazebo pose reset")
    parser.add_argument("--reset-still-timeout-s", type=float, default=3.0,
                        help="Max time to wait for fresh near-still odometry after reset")
    parser.add_argument("--settle-s", type=float, default=1.0)
    parser.add_argument("--step-s", type=float, default=1.0)
    parser.add_argument("--neutral-s", type=float, default=0.5)
    parser.add_argument("--rate-hz", type=float, default=20.0)
    parser.add_argument("--repeats", type=int, default=1)
    parser.add_argument("--baseline-closed-loop", action="store_true",
                        help="Apply step on top of deterministic baseline RC")
    parser.add_argument("--disable-xy-baseline", action="store_true",
                        help="Use altitude baseline only while calibrating roll/pitch response")
    parser.add_argument("--hover-gate-timeout-s", type=float, default=8.0,
                        help="Max time to wait for baseline hover stability before a step")
    parser.add_argument("--hover-gate-hold-s", type=float, default=0.5,
                        help="Required continuous stable time before applying a step")
    parser.add_argument("--hover-gate-max-vxy", type=float, default=0.8,
                        help="Max horizontal body speed for a clean baseline sample")
    parser.add_argument("--hover-gate-max-vz", type=float, default=0.25,
                        help="Max vertical speed for a clean baseline sample")
    parser.add_argument("--hover-gate-max-z-err", type=float, default=0.35,
                        help="Max altitude error for a clean baseline sample")
    parser.add_argument("--hover-gate-max-tilt-deg", type=float, default=12.0,
                        help="Max roll/pitch tilt before applying a step")
    parser.add_argument("--force-step-after-takeoff", action="store_true",
                        help="Skip hover gate and take a short response sample after takeoff/settle")
    parser.add_argument("--config", type=Path, default=Path("flight_bridge/config.yaml"))
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    _stop_existing_bridge()

    log_file = args.log_file or _default_log_path()
    adapter = MavlinkAdapter(vehicle_type="copter", mode="sitl", control="rc")
    if not adapter.connect(args.connect):
        raise SystemExit(f"failed to connect MAVLink at {args.connect}")

    odom = GazeboOdometryReader("/model/x3/odometry")
    odom.start()
    try:
        config = load_config(args.config)
        action_adapter = ActionAdapter(config.control, config.limits, config.copter)
        action_adapter._calibration_disable_xy = args.disable_xy_baseline
        adapter.arm_vehicle("both")
        tests = [
            ("roll_pos", RcCommand(1500 + args.step_pwm, 1500, args.hover_pwm, 1500)),
            ("roll_neg", RcCommand(1500 - args.step_pwm, 1500, args.hover_pwm, 1500)),
            ("pitch_pos", RcCommand(1500, 1500 + args.step_pwm, args.hover_pwm, 1500)),
            ("pitch_neg", RcCommand(1500, 1500 - args.step_pwm, args.hover_pwm, 1500)),
        ]
        with JsonlLogger(log_file) as logger:
            print(f"calibration log: {log_file}")
            results: list[dict[str, float | str]] = []
            for repeat in range(max(1, args.repeats)):
                print(f"repeat {repeat + 1}/{max(1, args.repeats)}")
                for name, step_cmd in tests:
                    result = _run_trial(args, adapter, odom, logger, name, step_cmd, action_adapter)
                    result["repeat"] = repeat
                    results.append(result)
                    print(_format_result(result))
            _print_summary(results, args.step_pwm)
    finally:
        adapter.emergency_stop()
        odom.stop()
    return 0


def _run_trial(
    args: argparse.Namespace,
    adapter: MavlinkAdapter,
    odom: GazeboOdometryReader,
    logger: JsonlLogger,
    name: str,
    step_cmd: RcCommand,
    action_adapter: ActionAdapter,
) -> dict[str, float | str]:
    reset = reset_gazebo_model_pose_fast(args.world, args.model, args.reset_z)
    odom.restart()
    if not reset.ok:
        result = _skipped_trial_result(name, step_cmd, {
            "reason": "reset_failed",
            "samples": 0,
        })
        result["reset_error"] = reset.error or ""
        logger.write(result)
        return result
    low_s = max(args.pre_low_s, args.reset_settle_s)
    if low_s > 0:
        _send_for(adapter, RcCommand(1500, 1500, 1000, 1500), low_s, args.rate_hz)
    reset_still = _wait_for_reset_still(odom, args)
    if not reset_still["ok"]:
        result = _skipped_trial_result(name, step_cmd, reset_still)
        logger.write(result)
        return result
    if not adapter.arm_vehicle("both"):
        result = _skipped_trial_result(name, step_cmd, {
            "reason": "arm_failed",
            "samples": 0,
        })
        logger.write(result)
        return result
    takeoff = _takeoff_to_altitude(adapter, odom, args)
    if not takeoff["ok"]:
        result = _skipped_trial_result(name, step_cmd, takeoff)
        logger.write(result)
        return result
    _send_for(
        adapter,
        RcCommand(1500, 1500, args.hover_pwm, 1500),
        args.settle_s,
        args.rate_hz,
        odom=odom,
        action_adapter=action_adapter if args.baseline_closed_loop else None,
        target_alt=args.target_alt,
    )
    if args.baseline_closed_loop and not args.force_step_after_takeoff:
        gate = _wait_for_hover_gate(adapter, odom, action_adapter, args)
        if not gate["stable"]:
            result = _skipped_trial_result(name, step_cmd, gate)
            logger.write(result)
            return result

    baseline = _sample_odom(odom, duration_s=0.35, rate_hz=args.rate_hz)
    step_delta = RcCommand(
        roll=step_cmd.roll - 1500,
        pitch=step_cmd.pitch - 1500,
        throttle=0,
        yaw=0,
    )
    step_samples = _send_and_sample(
        adapter,
        odom,
        step_cmd,
        args.step_s,
        args.rate_hz,
        action_adapter=action_adapter if args.baseline_closed_loop else None,
        target_alt=args.target_alt,
        step_delta=step_delta if args.baseline_closed_loop else None,
    )
    after = _send_and_sample(
        adapter,
        odom,
        RcCommand(1500, 1500, args.hover_pwm, 1500),
        args.neutral_s,
        args.rate_hz,
        action_adapter=action_adapter if args.baseline_closed_loop else None,
        target_alt=args.target_alt,
    )

    result = {
        "trial": name,
        "roll_pwm": step_cmd.roll,
        "pitch_pwm": step_cmd.pitch,
        "throttle_pwm": step_cmd.throttle,
        "baseline_vx": _mean([s["bvx"] for s in baseline]),
        "baseline_vy": _mean([s["bvy"] for s in baseline]),
        "baseline_vz": _mean([s["vz"] for s in baseline]),
        "step_vx": _mean([s["bvx"] for s in step_samples[-max(1, len(step_samples)//2):]]),
        "step_vy": _mean([s["bvy"] for s in step_samples[-max(1, len(step_samples)//2):]]),
        "step_vz": _mean([s["vz"] for s in step_samples[-max(1, len(step_samples)//2):]]),
        "after_vx": _mean([s["bvx"] for s in after]),
        "after_vy": _mean([s["bvy"] for s in after]),
        "baseline_ax": _slope(baseline, "bvx"),
        "baseline_ay": _slope(baseline, "bvy"),
        "step_ax": _slope(step_samples, "bvx"),
        "step_ay": _slope(step_samples, "bvy"),
        "baseline_n": len(baseline),
        "step_n": len(step_samples),
        "after_n": len(after),
    }
    result["dvx"] = float(result["step_vx"]) - float(result["baseline_vx"])
    result["dvy"] = float(result["step_vy"]) - float(result["baseline_vy"])
    result["dvz"] = float(result["step_vz"]) - float(result["baseline_vz"])
    result["dax"] = float(result["step_ax"]) - float(result["baseline_ax"])
    result["day"] = float(result["step_ay"]) - float(result["baseline_ay"])
    logger.write(result)
    return result


def _format_result(result: dict[str, float | str]) -> str:
    if result.get("skipped"):
        return (
            f"{result['trial']}: skipped reason={result.get('skip_reason', '')} "
            f"last_vxy={float(result.get('gate_last_vxy', 0.0)):.3f} "
            f"last_vz={float(result.get('gate_last_vz', 0.0)):.3f} "
            f"last_z_err={float(result.get('gate_last_z_err', 0.0)):.3f} "
            f"last_tilt={float(result.get('gate_last_tilt_deg', 0.0)):.1f}"
        )
    return (
        f"{result['trial']}: "
        f"dv=({float(result['dvx']):+.3f}, {float(result['dvy']):+.3f}) "
        f"da=({float(result['dax']):+.3f}, {float(result['day']):+.3f}) "
        f"baseline_v=({float(result['baseline_vx']):+.3f}, {float(result['baseline_vy']):+.3f}) "
        f"step_v=({float(result['step_vx']):+.3f}, {float(result['step_vy']):+.3f}) "
        f"n=({int(result['baseline_n'])},{int(result['step_n'])})"
    )


def _print_summary(results: list[dict[str, float | str]], step_pwm: int) -> None:
    grouped: dict[str, list[dict[str, float | str]]] = {}
    for result in results:
        if result.get("skipped"):
            continue
        grouped.setdefault(str(result["trial"]), []).append(result)

    skipped = sum(1 for result in results if result.get("skipped"))
    if skipped:
        print(f"\nskipped trials: {skipped}/{len(results)}")

    print("\nsummary median da:")
    med: dict[str, tuple[float, float]] = {}
    for name in ("roll_pos", "roll_neg", "pitch_pos", "pitch_neg"):
        items = grouped.get(name, [])
        if not items:
            continue
        dax = statistics.median(float(r["dax"]) for r in items)
        day = statistics.median(float(r["day"]) for r in items)
        med[name] = (dax, day)
        print(f"  {name}: da=({dax:+.4f}, {day:+.4f}) n={len(items)}")

    if all(name in med for name in ("roll_pos", "roll_neg", "pitch_pos", "pitch_neg")):
        denom = 2.0 * max(1, step_pwm)
        roll_ax = (med["roll_pos"][0] - med["roll_neg"][0]) / denom
        roll_ay = (med["roll_pos"][1] - med["roll_neg"][1]) / denom
        pitch_ax = (med["pitch_pos"][0] - med["pitch_neg"][0]) / denom
        pitch_ay = (med["pitch_pos"][1] - med["pitch_neg"][1]) / denom
        print("\npaired response per PWM:")
        print(f"  rc_response_ax_per_roll_pwm: {roll_ax:+.6f}")
        print(f"  rc_response_ay_per_roll_pwm: {roll_ay:+.6f}")
        print(f"  rc_response_ax_per_pitch_pwm: {pitch_ax:+.6f}")
        print(f"  rc_response_ay_per_pitch_pwm: {pitch_ay:+.6f}")


def _takeoff_to_altitude(
    adapter: MavlinkAdapter,
    odom: GazeboOdometryReader,
    args: argparse.Namespace,
) -> dict[str, float | str | bool]:
    deadline = time.time() + args.takeoff_timeout_s
    period = 1.0 / args.rate_hz
    ready_alt = (
        args.takeoff_ready_alt_m
        if args.takeoff_ready_alt_m is not None
        else max(0.0, args.target_alt - 0.25)
    )
    last: dict[str, float | str | bool] = {
        "ok": False,
        "reason": "takeoff_no_odom",
        "samples": 0,
    }
    while time.time() < deadline:
        state = odom.latest(timeout=0.2)
        if state is not None:
            sample = _odom_sample(state)
            last = {
                "ok": sample["z"] >= ready_alt,
                "reason": "takeoff_ready" if sample["z"] >= ready_alt else "takeoff_timeout",
                "samples": int(last.get("samples", 0)) + 1,
                "last_vxy": math.hypot(sample["bvx"], sample["bvy"]),
                "last_vz": sample["vz"],
                "last_z_err": args.target_alt - sample["z"],
                "last_tilt_deg": max(abs(sample["roll"]), abs(sample["pitch"])),
            }
            if sample["z"] >= ready_alt:
                return last
        adapter.send_rc_cmd(RcCommand(1500, 1500, args.takeoff_pwm, 1500))
        time.sleep(period)
    last["ok"] = False
    if last.get("reason") == "takeoff_ready":
        last["reason"] = "takeoff_timeout"
    return last


def _send_for(
    adapter: MavlinkAdapter,
    cmd: RcCommand,
    duration_s: float,
    rate_hz: float,
    odom: GazeboOdometryReader | None = None,
    action_adapter: ActionAdapter | None = None,
    target_alt: float = 0.0,
) -> None:
    deadline = time.time() + duration_s
    period = 1.0 / rate_hz
    while time.time() < deadline:
        out_cmd = _baseline_cmd(odom, action_adapter, target_alt) if action_adapter else cmd
        adapter.send_rc_cmd(out_cmd)
        time.sleep(period)


def _send_and_sample(
    adapter: MavlinkAdapter,
    odom: GazeboOdometryReader,
    cmd: RcCommand,
    duration_s: float,
    rate_hz: float,
    action_adapter: ActionAdapter | None = None,
    target_alt: float = 0.0,
    step_delta: RcCommand | None = None,
) -> list[dict[str, float]]:
    deadline = time.time() + duration_s
    period = 1.0 / rate_hz
    samples: list[dict[str, float]] = []
    while time.time() < deadline:
        out_cmd = cmd
        if action_adapter is not None:
            out_cmd = _baseline_cmd(odom, action_adapter, target_alt)
            if step_delta is not None:
                out_cmd = RcCommand(
                    roll=_clamp_pwm(out_cmd.roll + step_delta.roll),
                    pitch=_clamp_pwm(out_cmd.pitch + step_delta.pitch),
                    throttle=out_cmd.throttle,
                    yaw=out_cmd.yaw,
                )
        adapter.send_rc_cmd(out_cmd)
        state = odom.latest(timeout=0.05)
        if state is not None:
            samples.append(_odom_sample(state))
        time.sleep(period)
    return samples


def _wait_for_hover_gate(
    adapter: MavlinkAdapter,
    odom: GazeboOdometryReader,
    action_adapter: ActionAdapter,
    args: argparse.Namespace,
) -> dict[str, float | str | bool]:
    deadline = time.time() + args.hover_gate_timeout_s
    stable_since: float | None = None
    period = 1.0 / args.rate_hz
    last: dict[str, float | str | bool] = {
        "stable": False,
        "reason": "no_odom",
        "samples": 0,
    }

    while time.time() < deadline:
        cmd = _baseline_cmd(odom, action_adapter, args.target_alt)
        adapter.send_rc_cmd(cmd)
        state = odom.latest(timeout=0.05)
        if state is None:
            stable_since = None
            time.sleep(period)
            continue

        sample = _odom_sample(state)
        vxy = math.hypot(sample["bvx"], sample["bvy"])
        vz = abs(sample["vz"])
        z_err = abs(args.target_alt - sample["z"])
        tilt = max(abs(sample["roll"]), abs(sample["pitch"]))
        stable = (
            vxy <= args.hover_gate_max_vxy
            and vz <= args.hover_gate_max_vz
            and z_err <= args.hover_gate_max_z_err
            and tilt <= args.hover_gate_max_tilt_deg
        )
        last = {
            "stable": stable,
            "reason": "stable" if stable else "hover_gate_timeout",
            "samples": int(last.get("samples", 0)) + 1,
            "last_vxy": vxy,
            "last_vz": sample["vz"],
            "last_z_err": args.target_alt - sample["z"],
            "last_tilt_deg": tilt,
            "last_x": sample["x"],
            "last_y": sample["y"],
            "last_z": sample["z"],
        }
        if stable:
            stable_since = stable_since or time.time()
            if time.time() - stable_since >= args.hover_gate_hold_s:
                last["stable"] = True
                last["reason"] = "stable"
                return last
        else:
            stable_since = None
        time.sleep(period)

    last["stable"] = False
    if last.get("reason") == "stable":
        last["reason"] = "hover_gate_timeout"
    return last


def _wait_for_reset_still(
    odom: GazeboOdometryReader,
    args: argparse.Namespace,
) -> dict[str, float | str | bool]:
    deadline = time.time() + args.reset_still_timeout_s
    period = 1.0 / args.rate_hz
    last: dict[str, float | str | bool] = {
        "ok": False,
        "reason": "reset_no_odom",
        "samples": 0,
    }
    while time.time() < deadline:
        state = odom.latest(timeout=0.05)
        if state is None:
            time.sleep(period)
            continue
        sample = _odom_sample(state)
        vxy = math.hypot(sample["bvx"], sample["bvy"])
        tilt = max(abs(sample["roll"]), abs(sample["pitch"]))
        near_reset_pose = sample["z"] <= max(0.4, args.reset_z + 0.25)
        still = (
            near_reset_pose
            and vxy <= 0.25
            and abs(sample["vz"]) <= 0.25
            and tilt <= 5.0
        )
        last = {
            "ok": still,
            "reason": "reset_still" if still else "reset_still_timeout",
            "samples": int(last.get("samples", 0)) + 1,
            "last_vxy": vxy,
            "last_vz": sample["vz"],
            "last_z_err": args.target_alt - sample["z"],
            "last_tilt_deg": tilt,
        }
        if still:
            return last
        time.sleep(period)
    last["ok"] = False
    if last.get("reason") == "reset_still":
        last["reason"] = "reset_still_timeout"
    return last


def _skipped_trial_result(
    name: str,
    step_cmd: RcCommand,
    gate: dict[str, float | str | bool],
) -> dict[str, float | str | bool]:
    return {
        "trial": name,
        "roll_pwm": step_cmd.roll,
        "pitch_pwm": step_cmd.pitch,
        "throttle_pwm": step_cmd.throttle,
        "skipped": True,
        "skip_reason": str(gate.get("reason", "hover_gate_timeout")),
        "gate_samples": int(gate.get("samples", 0)),
        "gate_last_vxy": float(gate.get("last_vxy", 0.0)),
        "gate_last_vz": float(gate.get("last_vz", 0.0)),
        "gate_last_z_err": float(gate.get("last_z_err", 0.0)),
        "gate_last_tilt_deg": float(gate.get("last_tilt_deg", 0.0)),
    }


def _baseline_cmd(
    odom: GazeboOdometryReader | None,
    action_adapter: ActionAdapter | None,
    target_alt: float,
) -> RcCommand:
    if odom is None or action_adapter is None:
        return RcCommand(1500, 1500, 1700, 1500)
    state = odom.latest(timeout=0.05)
    if state is None:
        return RcCommand(1500, 1500, 1700, 1500)
    cos_yaw = math.cos(state.yaw)
    sin_yaw = math.sin(state.yaw)
    x_err = state.x * cos_yaw + state.y * sin_yaw
    y_err = state.x * sin_yaw - state.y * cos_yaw
    vx = state.vx * cos_yaw + state.vy * sin_yaw
    vy = state.vx * sin_yaw - state.vy * cos_yaw
    z_err = target_alt - state.z
    x_err = 0.0 if getattr(action_adapter, "_calibration_disable_xy", False) else x_err
    y_err = 0.0 if getattr(action_adapter, "_calibration_disable_xy", False) else y_err
    vx = 0.0 if getattr(action_adapter, "_calibration_disable_xy", False) else vx
    vy = 0.0 if getattr(action_adapter, "_calibration_disable_xy", False) else vy
    return action_adapter.baseline_rc_for_altitude(
        z_err,
        state.vz,
        x_err_m=x_err,
        y_err_m=y_err,
        vx_mps=vx,
        vy_mps=vy,
        altitude_m=state.z,
    )


def _sample_odom(
    odom: GazeboOdometryReader,
    duration_s: float,
    rate_hz: float,
) -> list[dict[str, float]]:
    deadline = time.time() + duration_s
    period = 1.0 / rate_hz
    samples: list[dict[str, float]] = []
    while time.time() < deadline:
        state = odom.latest(timeout=0.05)
        if state is not None:
            samples.append(_odom_sample(state))
        time.sleep(period)
    return samples


def _odom_sample(state) -> dict[str, float]:
    cos_yaw = math.cos(state.yaw)
    sin_yaw = math.sin(state.yaw)
    bvx = state.vx * cos_yaw + state.vy * sin_yaw
    bvy = state.vx * sin_yaw - state.vy * cos_yaw
    return {
        "t": time.time(),
        "x": state.x,
        "y": state.y,
        "z": state.z,
        "vx": state.vx,
        "vy": state.vy,
        "vz": state.vz,
        "bvx": bvx,
        "bvy": bvy,
        "roll": math.degrees(state.roll),
        "pitch": math.degrees(state.pitch),
        "yaw": math.degrees(state.yaw),
    }


def _stop_existing_bridge() -> None:
    for pattern in ("python3 -m flight_bridge.main", "tools/visualize_feedback.py"):
        subprocess.run(["pkill", "-TERM", "-f", pattern], check=False)
    time.sleep(0.5)
    for pattern in ("python3 -m flight_bridge.main", "tools/visualize_feedback.py"):
        subprocess.run(["pkill", "-KILL", "-f", pattern], check=False)


def _default_log_path() -> Path:
    log_dir = Path("logs")
    log_dir.mkdir(exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    return log_dir / f"rc_step_response_{stamp}.jsonl"


def _mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def _clamp_pwm(value: int) -> int:
    return max(1000, min(2000, int(value)))


def _slope(samples: list[dict[str, float]], key: str) -> float:
    if len(samples) < 2:
        return 0.0
    t0 = samples[0]["t"]
    xs = [s["t"] - t0 for s in samples]
    ys = [s[key] for s in samples]
    x_mean = _mean(xs)
    y_mean = _mean(ys)
    denom = sum((x - x_mean) ** 2 for x in xs)
    if denom <= 1.0e-9:
        return 0.0
    return sum((x - x_mean) * (y - y_mean) for x, y in zip(xs, ys)) / denom


if __name__ == "__main__":
    raise SystemExit(main())
