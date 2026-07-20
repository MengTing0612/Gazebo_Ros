"""Command-line interface for the flight bridge entrypoint."""

from __future__ import annotations

import argparse
from pathlib import Path

from flight_bridge.cli_common import add_sdk_arguments, timestamped_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--phase", type=int, choices=[0, 1], default=0,
                        help="Phase 0 (dry-run) or Phase 1 (SDK-in-the-loop)")
    parser.add_argument("--mode", choices=["dry-run", "sitl"], default="dry-run",
                        help="Vehicle mode (Phase 1 supports sitl)")
    parser.add_argument("--adapter-mode", choices=["front-4", "learned"], default="front-4",
                        help="Action->RC mapping: front-4 (action[0..3]) or learned (W matrix)")
    parser.add_argument("--gaia-adapter-config", type=Path, default=Path("flight_bridge/gaia_adapter.yaml"),
                        help="Path to learned adapter YAML (for --adapter-mode learned)")
    parser.add_argument("--control", choices=["off", "rc", "guided"], default="off",
                        help="Control method (Phase 1: rc or guided)")
    parser.add_argument("--rc-control-mode", choices=["residual", "sdk-direct", "baseline-only"], default="residual",
                        help="RC path: baseline+SDK residual, direct SDK action[0..3], or deterministic baseline only")
    parser.add_argument("--loiter-test", action="store_true",
                        help="Switch to LOITER mode after arm, send neutral RC for APM native hover")
    add_sdk_arguments(parser)
    parser.add_argument("--rate-hz", type=float, default=20.0, help="Loop rate")
    parser.add_argument("--iterations", type=int, default=0, help="0 means run until interrupted")
    parser.add_argument("--target-heading", type=float, default=0.0, help="Target heading (deg)")
    parser.add_argument("--summary-every", type=int, default=20, help="Print every N frames")
    parser.add_argument("--log-file", type=Path, help="Optional JSONL log path")
    parser.add_argument("--log-dir", type=Path, default=Path("logs"), help="Directory for default logs")
    parser.add_argument("--config", type=Path, default=Path("flight_bridge/config.yaml"),
                        help="Phase 1 config YAML")
    parser.add_argument("--use-real-limits", action="store_true",
                        help="Use real-drone limits instead of sim limits")
    parser.add_argument("--connect", type=str, default="",
                        help="MAVLink connection string (default: tcp:127.0.0.1:5760)")
    parser.add_argument("--target-alt", type=float, default=3.0,
                        help="Target altitude for hover (m)")
    parser.add_argument("--target-x", type=float, default=None,
                        help="Fixed hover target X in Gazebo/world meters. If omitted, use the reset/home X.")
    parser.add_argument("--target-y", type=float, default=None,
                        help="Fixed hover target Y in Gazebo/world meters. If omitted, use the reset/home Y.")
    parser.add_argument("--use-gazebo-truth", action=argparse.BooleanOptionalAction, default=True,
                        help="In SITL, use Gazebo odometry as truth for obs/feedback/reset gates")
    parser.add_argument("--gazebo-odom-topic", type=str, default="/model/x3/odometry",
                        help="Gazebo odometry topic used by --use-gazebo-truth")
    parser.add_argument("--reset-on-flip", action="store_true",
                        help="Reset only the Gazebo model pose when roll/pitch exceeds --reset-tilt-deg")
    parser.add_argument("--reset-method", choices=["model-pose", "restart-stack"], default="model-pose",
                        help="Episode reset method: fast Gazebo pose reset, or restart Gazebo/SITL stack while keeping SDK alive")
    parser.add_argument("--sitl-start-script", type=Path,
                        default=Path("APM源代码原理图/ArduPlane-2.74b/Tools/start_arducopter_gz_sitl.sh"),
                        help="Script used by --reset-method restart-stack")
    parser.add_argument("--sitl-restart-timeout-s", type=float, default=35.0,
                        help="Timeout for restarting Gazebo/SITL stack")
    parser.add_argument("--reset-tilt-deg", type=float, default=85.0,
                        help="Tilt threshold for Gazebo-only flip reset")
    parser.add_argument("--reset-xy-runaway-m", type=float, default=20.0,
                        help="Reset when horizontal XY error exceeds this for --reset-runaway-hold-s")
    parser.add_argument("--reset-runaway-hold-s", type=float, default=2.0,
                        help="Seconds that horizontal runaway must persist before reset")
    parser.add_argument("--reset-ground-stuck-alt-m", type=float, default=0.20,
                        help="Altitude below which high-throttle/no-climb is treated as ground-stuck")
    parser.add_argument("--reset-ground-stuck-throttle-pwm", type=int, default=1900,
                        help="Throttle PWM threshold for stuck-on-ground reset detection")
    parser.add_argument("--reset-ground-stuck-vz-mps", type=float, default=0.05,
                        help="Maximum vertical speed magnitude for stuck-on-ground reset detection")
    parser.add_argument("--reset-ground-stuck-hold-s", type=float, default=4.0,
                        help="Seconds that stuck-on-ground condition must persist before reset")
    parser.add_argument("--sdk-direct-level-until-alt-m", type=float, default=0.70,
                        help="In sdk-direct RC mode, hold roll/pitch neutral below this altitude")
    parser.add_argument("--reset-cooldown-s", type=float, default=2.5,
                        help="Minimum seconds between Gazebo reset attempts")
    parser.add_argument("--reset-settle-s", type=float, default=0.5,
                        help="Seconds to send low throttle after Gazebo reset")
    parser.add_argument("--gazebo-world", type=str, default="arducopter_x3",
                        help="Gazebo world name for model reset service")
    parser.add_argument("--gazebo-model", type=str, default="x3",
                        help="Gazebo model name to reset")
    parser.add_argument("--reset-pose-z", type=float, default=0.12,
                        help="Gazebo reset pose z coordinate")
    parser.add_argument("--reset-recovery-settle-s", type=float, default=2.0,
                        help="After Gazebo reset, seconds to hold low throttle and suppress SDK")
    parser.add_argument("--reset-recovery-baseline-s", type=float, default=3.0,
                        help="After reset settle, seconds to use baseline altitude control before SDK")
    parser.add_argument("--reset-recovery", action=argparse.BooleanOptionalAction, default=True,
                        help="After Gazebo reset, pause SDK control for settle/baseline recovery")
    parser.add_argument("--post-reset-guard-s", type=float, default=0.75,
                        help="Seconds to suppress SDK and send baseline RC after reset, even without full recovery")
    parser.add_argument("--reset-recovery-tilt-deg", type=float, default=15.0,
                        help="Maximum tilt considered stable during reset recovery")
    parser.add_argument("--reset-recovery-alt-error-m", type=float, default=0.35,
                        help="Altitude error threshold that can end baseline recovery early")
    parser.add_argument("--reset-rebase-home", action=argparse.BooleanOptionalAction, default=True,
                        help="Rebase local target/home to current MAVLink position after Gazebo reset")
    parser.add_argument("--auto-rearm", action=argparse.BooleanOptionalAction, default=True,
                        help="Pause SDK control and re-arm when the copter is disarmed")
    parser.add_argument("--rearm-method", choices=["command", "rudder", "both"], default="both",
                        help="Arming method used by --auto-rearm")
    parser.add_argument("--rearm-cooldown-s", type=float, default=2.0,
                        help="Minimum seconds between automatic re-arm attempts")
    return parser.parse_args()


def validate_common_args(args: argparse.Namespace) -> None:
    if args.rate_hz <= 0:
        raise SystemExit("--rate-hz must be positive")
    if args.iterations < 0:
        raise SystemExit("--iterations must be >= 0")
    if args.summary_every <= 0:
        raise SystemExit("--summary-every must be positive")


def default_log_path(log_dir: Path, phase: str) -> Path:
    return timestamped_path(log_dir, f"flight_bridge_{phase}", "jsonl")
