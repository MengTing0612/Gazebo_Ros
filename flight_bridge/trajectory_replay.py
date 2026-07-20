#!/usr/bin/env python3
"""Replay UAV trajectory samples through the flight bridge SDK interface."""

from __future__ import annotations

if __package__ in {None, ""}:
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import argparse
import csv
import json
import math
import socket
import statistics
import time
from dataclasses import dataclass
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from flight_bridge.cli_common import add_sdk_arguments, resolve_sdk_credentials, timestamped_path
from flight_bridge.jsonl_logger import JsonlLogger
from flight_bridge.mavlink_adapter import FlightState
from flight_bridge.observation_builder import ObservationBuilder, build_feedback, wrap_deg
from flight_bridge.sdk_client import make_inference_client


@dataclass(frozen=True)
class TrajectorySample:
    timestamp_s: float
    x_m: float
    y_m: float
    z_m: float


@dataclass
class ReplayFrame:
    frame: int
    sample: TrajectorySample
    state: FlightState
    obs: list[float]
    feedback: list[float]
    action: list[float]
    latency_ms: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    add_sdk_arguments(parser)
    parser.add_argument(
        "--trajectory-csv",
        type=Path,
        default=Path("outputs/uav_trajectory_adjustment/indoor_forward_6_segment_adjusted.csv"),
        help="CSV containing raw and adjusted trajectory columns",
    )
    parser.add_argument("--log-dir", type=Path, default=Path("logs"))
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/uav_sdk_replay"))
    parser.add_argument("--target-mode", choices=["next-point", "constant-heading"], default="next-point")
    parser.add_argument("--target-heading", type=float, default=0.0)
    parser.add_argument("--max-frames", type=int, default=0, help="0 replays all frames")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.max_frames < 0:
        raise SystemExit("--max-frames must be >= 0")

    raw_samples, adjusted_samples = load_comparison_csv(args.trajectory_csv)
    if args.max_frames:
        raw_samples = raw_samples[: args.max_frames]
        adjusted_samples = adjusted_samples[: args.max_frames]

    username, password = resolve_sdk_credentials(args)
    inference_client = make_inference_client(
        sdk_mode=args.sdk_mode,
        host=args.host,
        port=args.port,
        timeout_s=args.timeout,
        username=username,
        password=password,
        resume=args.resume,
    )

    args.output_dir.mkdir(parents=True, exist_ok=True)
    log_file = timestamped_path(args.log_dir, f"trajectory_replay_{args.sdk_mode}", "jsonl")

    print(
        f"trajectory replay sdk_mode={args.sdk_mode} frames={len(raw_samples)} "
        f"trajectory={args.trajectory_csv} log={log_file}"
    )

    try:
        with inference_client as sdk, JsonlLogger(log_file) as logger:
            raw_frames = replay_variant("raw", raw_samples, args, sdk, logger)
            adjusted_frames = replay_variant("adjusted", adjusted_samples, args, sdk, logger)
    except ConnectionRefusedError as exc:
        raise SystemExit(
            f"Cannot connect to SDK server at {args.host}:{args.port}. "
            "Start ./examples/retina-server-x86_64 first, or use --sdk-mode mock."
        ) from exc
    except socket.timeout as exc:
        raise SystemExit(f"Timed out while talking to SDK server at {args.host}:{args.port}.") from exc

    summary = compare_replay_frames(raw_frames, adjusted_frames)
    (args.output_dir / "sdk_replay_summary.json").write_text(
        json.dumps(summary, indent=2),
        encoding="utf-8",
    )
    write_frame_csv(args.output_dir / "sdk_replay_frames.csv", raw_frames, adjusted_frames)
    plot_replay(args.output_dir / "sdk_replay_comparison.png", raw_frames, adjusted_frames)

    print(json.dumps(summary, indent=2))
    print(f"wrote {args.output_dir / 'sdk_replay_comparison.png'}")
    return 0


def load_comparison_csv(path: Path) -> tuple[list[TrajectorySample], list[TrajectorySample]]:
    if not path.exists():
        raise SystemExit(f"Trajectory CSV does not exist: {path}")

    raw_samples: list[TrajectorySample] = []
    adjusted_samples: list[TrajectorySample] = []
    with path.open(encoding="utf-8", newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        required = {
            "timestamp",
            "tx_raw",
            "ty_raw",
            "tz_raw",
            "tx_adjusted",
            "ty_adjusted",
            "tz_adjusted",
        }
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise SystemExit(f"Trajectory CSV is missing columns: {sorted(missing)}")

        for row in reader:
            timestamp_s = float(row["timestamp"])
            raw_samples.append(
                TrajectorySample(
                    timestamp_s=timestamp_s,
                    x_m=float(row["tx_raw"]),
                    y_m=float(row["ty_raw"]),
                    z_m=float(row["tz_raw"]),
                )
            )
            adjusted_samples.append(
                TrajectorySample(
                    timestamp_s=timestamp_s,
                    x_m=float(row["tx_adjusted"]),
                    y_m=float(row["ty_adjusted"]),
                    z_m=float(row["tz_adjusted"]),
                )
            )

    if len(raw_samples) < 3:
        raise SystemExit("Need at least 3 trajectory samples to estimate speed and yaw rate")
    return raw_samples, adjusted_samples


def replay_variant(
    variant: str,
    samples: list[TrajectorySample],
    args: argparse.Namespace,
    sdk,
    logger: JsonlLogger,
) -> list[ReplayFrame]:
    obs_builder = ObservationBuilder()
    frames: list[ReplayFrame] = []

    for frame_index, sample in enumerate(samples):
        state = state_from_trajectory(samples, frame_index)
        state.heading_error_deg = compute_heading_error(samples, frame_index, state.heading, args)
        obs = obs_builder.build(state)
        feedback = build_feedback(state)

        infer_started_at = time.perf_counter()
        action = sdk.infer(obs, feedback)
        latency_ms = (time.perf_counter() - infer_started_at) * 1000.0

        replay_frame = ReplayFrame(
            frame=frame_index,
            sample=sample,
            state=state,
            obs=obs,
            feedback=feedback,
            action=action,
            latency_ms=latency_ms,
        )
        frames.append(replay_frame)

        logger.write(
            {
                "ts": time.time(),
                "source": "trajectory_replay",
                "variant": variant,
                "frame": frame_index,
                "mode": "offline-replay",
                "control": "off",
                "vehicle_mode": state.mode,
                "armed": state.armed,
                "sample": {
                    "timestamp_s": sample.timestamp_s,
                    "x_m": sample.x_m,
                    "y_m": sample.y_m,
                    "z_m": sample.z_m,
                },
                "state": state.to_log_dict(),
                "obs": obs,
                "feedback": feedback,
                "action": action,
                "latency_ms": latency_ms,
                "sent": False,
                "error": None,
            }
        )

    return frames


def state_from_trajectory(samples: list[TrajectorySample], index: int) -> FlightState:
    current = samples[index]
    previous = samples[max(0, index - 1)]
    following = samples[min(len(samples) - 1, index + 1)]

    velocity_x_mps, velocity_y_mps, vertical_speed_mps = finite_difference(previous, following)
    groundspeed_mps = math.hypot(velocity_x_mps, velocity_y_mps)
    heading_deg = heading_from_velocity(velocity_x_mps, velocity_y_mps, index, samples)
    yaw_rad = math.radians(heading_deg)
    yaw_speed_radps = yaw_rate(samples, index)

    return FlightState(
        roll=0.0,
        pitch=math.atan2(-vertical_speed_mps, max(groundspeed_mps, 0.1)),
        yaw=yaw_rad,
        roll_speed=0.0,
        pitch_speed=0.0,
        yaw_speed=yaw_speed_radps,
        heading=heading_deg,
        groundspeed=groundspeed_mps,
        lat=0.0,
        lon=0.0,
        relative_alt=current.z_m,
        mode="OFFLINE_REPLAY",
        armed=False,
        battery_voltage=0.0,
    )


def finite_difference(
    previous: TrajectorySample,
    following: TrajectorySample,
) -> tuple[float, float, float]:
    dt_s = following.timestamp_s - previous.timestamp_s
    if dt_s <= 0.0:
        return 0.0, 0.0, 0.0

    return (
        (following.x_m - previous.x_m) / dt_s,
        (following.y_m - previous.y_m) / dt_s,
        (following.z_m - previous.z_m) / dt_s,
    )


def heading_from_velocity(
    velocity_x_mps: float,
    velocity_y_mps: float,
    index: int,
    samples: list[TrajectorySample],
) -> float:
    if abs(velocity_x_mps) + abs(velocity_y_mps) < 1e-6:
        if index == 0:
            return 0.0
        previous_state = state_from_trajectory(samples, index - 1)
        return previous_state.heading
    return math.degrees(math.atan2(velocity_y_mps, velocity_x_mps)) % 360.0


def yaw_rate(samples: list[TrajectorySample], index: int) -> float:
    if index == 0 or index >= len(samples) - 1:
        return 0.0

    prev_heading = segment_heading(samples[index - 1], samples[index])
    next_heading = segment_heading(samples[index], samples[index + 1])
    dt_s = samples[index + 1].timestamp_s - samples[index - 1].timestamp_s
    if dt_s <= 0.0:
        return 0.0

    return math.radians(wrap_deg(next_heading - prev_heading)) / dt_s


def segment_heading(start: TrajectorySample, end: TrajectorySample) -> float:
    return math.degrees(math.atan2(end.y_m - start.y_m, end.x_m - start.x_m)) % 360.0


def compute_heading_error(
    samples: list[TrajectorySample],
    index: int,
    current_heading_deg: float,
    args: argparse.Namespace,
) -> float:
    if args.target_mode == "constant-heading":
        target_heading_deg = args.target_heading
    else:
        next_index = min(len(samples) - 1, index + 1)
        target_heading_deg = segment_heading(samples[index], samples[next_index])

    return wrap_deg(target_heading_deg - current_heading_deg)


def compare_replay_frames(raw_frames: list[ReplayFrame], adjusted_frames: list[ReplayFrame]) -> dict[str, object]:
    raw_actions = [frame.action for frame in raw_frames]
    adjusted_actions = [frame.action for frame in adjusted_frames]
    action_count = min(len(raw_actions), len(adjusted_actions))
    action_dim = min((len(action) for action in raw_actions + adjusted_actions), default=0)

    action_l2_deltas = [
        vector_l2_delta(raw_actions[index], adjusted_actions[index], action_dim)
        for index in range(action_count)
    ]
    feedback_deltas = [
        abs(raw_frames[index].feedback[0] - adjusted_frames[index].feedback[0])
        for index in range(action_count)
    ]

    return {
        "frames": action_count,
        "action_dim": action_dim,
        "raw": summarize_frames(raw_frames),
        "adjusted": summarize_frames(adjusted_frames),
        "adjusted_vs_raw": {
            "mean_action_l2_delta": mean(action_l2_deltas),
            "p95_action_l2_delta": percentile(action_l2_deltas, 95.0),
            "max_action_l2_delta": max(action_l2_deltas) if action_l2_deltas else 0.0,
            "mean_feedback_abs_delta": mean(feedback_deltas),
            "p95_feedback_abs_delta": percentile(feedback_deltas, 95.0),
        },
    }


def summarize_frames(frames: list[ReplayFrame]) -> dict[str, float]:
    speeds = [frame.state.groundspeed for frame in frames]
    heading_errors = [abs(frame.state.heading_error_deg) for frame in frames]
    latency_values = [frame.latency_ms for frame in frames]
    actions = [value for frame in frames for value in frame.action]
    return {
        "mean_speed_mps": mean(speeds),
        "p95_speed_mps": percentile(speeds, 95.0),
        "mean_abs_heading_error_deg": mean(heading_errors),
        "p95_abs_heading_error_deg": percentile(heading_errors, 95.0),
        "mean_latency_ms": mean(latency_values),
        "p95_latency_ms": percentile(latency_values, 95.0),
        "action_min": min(actions) if actions else 0.0,
        "action_max": max(actions) if actions else 0.0,
    }


def write_frame_csv(path: Path, raw_frames: list[ReplayFrame], adjusted_frames: list[ReplayFrame]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    action_dim = min((len(frame.action) for frame in raw_frames + adjusted_frames), default=0)

    with path.open("w", encoding="utf-8", newline="") as csv_file:
        fieldnames = [
            "frame",
            "timestamp_s",
            "raw_speed_mps",
            "adjusted_speed_mps",
            "raw_heading_error_deg",
            "adjusted_heading_error_deg",
            "raw_feedback",
            "adjusted_feedback",
            "action_l2_delta",
        ]
        fieldnames.extend(f"raw_action_{index}" for index in range(action_dim))
        fieldnames.extend(f"adjusted_action_{index}" for index in range(action_dim))
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()

        for raw_frame, adjusted_frame in zip(raw_frames, adjusted_frames):
            row = {
                "frame": raw_frame.frame,
                "timestamp_s": raw_frame.sample.timestamp_s,
                "raw_speed_mps": raw_frame.state.groundspeed,
                "adjusted_speed_mps": adjusted_frame.state.groundspeed,
                "raw_heading_error_deg": raw_frame.state.heading_error_deg,
                "adjusted_heading_error_deg": adjusted_frame.state.heading_error_deg,
                "raw_feedback": raw_frame.feedback[0],
                "adjusted_feedback": adjusted_frame.feedback[0],
                "action_l2_delta": vector_l2_delta(raw_frame.action, adjusted_frame.action, action_dim),
            }
            for index in range(action_dim):
                row[f"raw_action_{index}"] = raw_frame.action[index]
                row[f"adjusted_action_{index}"] = adjusted_frame.action[index]
            writer.writerow(row)


def plot_replay(path: Path, raw_frames: list[ReplayFrame], adjusted_frames: list[ReplayFrame]) -> None:
    t0 = raw_frames[0].sample.timestamp_s
    time_s = [frame.sample.timestamp_s - t0 for frame in raw_frames]
    raw_speed = [frame.state.groundspeed for frame in raw_frames]
    adjusted_speed = [frame.state.groundspeed for frame in adjusted_frames]
    raw_feedback = [frame.feedback[0] for frame in raw_frames]
    adjusted_feedback = [frame.feedback[0] for frame in adjusted_frames]
    action_dim = min((len(frame.action) for frame in raw_frames + adjusted_frames), default=0)
    action_delta = [
        vector_l2_delta(raw_frames[index].action, adjusted_frames[index].action, action_dim)
        for index in range(min(len(raw_frames), len(adjusted_frames)))
    ]

    fig, axes = plt.subplots(2, 2, figsize=(14, 8))
    axes[0, 0].plot(
        [frame.sample.x_m for frame in raw_frames],
        [frame.sample.y_m for frame in raw_frames],
        label="raw",
        linewidth=1.8,
    )
    axes[0, 0].plot(
        [frame.sample.x_m for frame in adjusted_frames],
        [frame.sample.y_m for frame in adjusted_frames],
        label="adjusted",
        linewidth=1.8,
    )
    axes[0, 0].set_title("XY replay path")
    axes[0, 0].set_xlabel("x [m]")
    axes[0, 0].set_ylabel("y [m]")
    axes[0, 0].axis("equal")
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].legend()

    axes[0, 1].plot(time_s, raw_speed, label="raw", linewidth=1.6)
    axes[0, 1].plot(time_s, adjusted_speed, label="adjusted", linewidth=1.6)
    axes[0, 1].set_title("SDK input speed")
    axes[0, 1].set_xlabel("time [s]")
    axes[0, 1].set_ylabel("m/s")
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].legend()

    axes[1, 0].plot(time_s, raw_feedback, label="raw", linewidth=1.6)
    axes[1, 0].plot(time_s, adjusted_feedback, label="adjusted", linewidth=1.6)
    axes[1, 0].set_title("SDK feedback")
    axes[1, 0].set_xlabel("time [s]")
    axes[1, 0].set_ylabel("normalized heading error")
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].legend()

    axes[1, 1].plot(time_s[: len(action_delta)], action_delta, linewidth=1.6)
    axes[1, 1].set_title("Action L2 delta: adjusted vs raw")
    axes[1, 1].set_xlabel("time [s]")
    axes[1, 1].set_ylabel("L2 delta")
    axes[1, 1].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(path, dpi=180)


def vector_l2_delta(left: list[float], right: list[float], dim: int) -> float:
    if dim <= 0:
        return 0.0
    return math.sqrt(sum((left[index] - right[index]) ** 2 for index in range(dim)))


def mean(values: list[float]) -> float:
    return statistics.fmean(values) if values else 0.0


def percentile(values: list[float], percentile_value: float) -> float:
    if not values:
        return 0.0

    ordered = sorted(values)
    index = (len(ordered) - 1) * (percentile_value / 100.0)
    lower_index = int(index)
    upper_index = min(lower_index + 1, len(ordered) - 1)
    fraction = index - lower_index
    return ordered[lower_index] * (1.0 - fraction) + ordered[upper_index] * fraction


if __name__ == "__main__":
    raise SystemExit(main())
