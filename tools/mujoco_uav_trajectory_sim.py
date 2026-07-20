#!/usr/bin/env python3
"""Simulate raw and adjusted UAV trajectories with a small MuJoCo model."""

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
import mujoco
import numpy as np

from flight_bridge.cli_common import add_sdk_arguments, resolve_sdk_credentials
from flight_bridge.mavlink_adapter import FlightState
from flight_bridge.observation_builder import ObservationBuilder, build_feedback, wrap_deg
from flight_bridge.sdk_client import make_inference_client


@dataclass(frozen=True)
class TrajectoryPoint:
    timestamp_s: float
    position_m: np.ndarray


@dataclass
class SimFrame:
    time_s: float
    target_m: np.ndarray
    position_m: np.ndarray
    velocity_mps: np.ndarray
    force_n: np.ndarray
    tracking_error_m: float
    sdk_action: list[float]
    sdk_latency_ms: float


MODEL_XML = """
<mujoco model="uav_point_tracker">
  <option timestep="0.005" gravity="0 0 0" integrator="RK4"/>
  <worldbody>
    <light pos="0 0 10"/>
    <geom type="plane" size="80 80 0.05" rgba="0.08 0.09 0.10 1"/>
    <body name="uav" pos="0 0 0">
      <joint name="root" type="free"/>
      <geom type="sphere" size="0.18" mass="1.0" rgba="0.1 0.45 0.95 1"/>
      <site name="center" pos="0 0 0" size="0.05" rgba="1 0.8 0.1 1"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="fx" joint="root" gear="1 0 0 0 0 0" ctrllimited="true" ctrlrange="-40 40"/>
    <motor name="fy" joint="root" gear="0 1 0 0 0 0" ctrllimited="true" ctrlrange="-40 40"/>
    <motor name="fz" joint="root" gear="0 0 1 0 0 0" ctrllimited="true" ctrlrange="-40 40"/>
  </actuator>
</mujoco>
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    add_sdk_arguments(parser)
    parser.add_argument(
        "--trajectory-csv",
        type=Path,
        default=Path("outputs/uav_trajectory_adjustment/indoor_forward_6_segment_adjusted.csv"),
    )
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/mujoco_uav_sim"))
    parser.add_argument("--max-frames", type=int, default=120)
    parser.add_argument("--kp", type=float, default=8.0, help="Position gain")
    parser.add_argument("--kd", type=float, default=4.0, help="Velocity gain")
    parser.add_argument("--max-force", type=float, default=35.0, help="Per-axis force limit in Newtons")
    parser.add_argument("--no-sdk", action="store_true", help="Skip SDK inference and only run MuJoCo tracking")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    raw_points, adjusted_points = load_trajectory_pair(args.trajectory_csv, args.max_frames)
    args.output_dir.mkdir(parents=True, exist_ok=True)

    sdk = None
    if not args.no_sdk:
        username, password = resolve_sdk_credentials(args)
        sdk = make_inference_client(
            sdk_mode=args.sdk_mode,
            host=args.host,
            port=args.port,
            timeout_s=args.timeout,
            username=username,
            password=password,
            resume=args.resume,
        )

    try:
        if sdk is None:
            raw_frames = simulate_variant(raw_points, args, sdk=None)
            adjusted_frames = simulate_variant(adjusted_points, args, sdk=None)
        else:
            with sdk as inference_client:
                raw_frames = simulate_variant(raw_points, args, sdk=inference_client)
                adjusted_frames = simulate_variant(adjusted_points, args, sdk=inference_client)
    except ConnectionRefusedError as exc:
        raise SystemExit(
            f"Cannot connect to SDK server at {args.host}:{args.port}. "
            "Run scripts/start_retina_server.py first, or pass --no-sdk."
        ) from exc
    except socket.timeout as exc:
        raise SystemExit(f"Timed out while talking to SDK server at {args.host}:{args.port}") from exc

    summary = {
        "simulator": "mujoco",
        "model": "free body point-mass UAV with 3 force actuators",
        "controller": {
            "type": "PD reference tracking",
            "kp": args.kp,
            "kd": args.kd,
            "max_force_n": args.max_force,
            "sdk_action_usage": "recorded only; not mapped to force control",
        },
        "raw": summarize_frames(raw_frames),
        "adjusted": summarize_frames(adjusted_frames),
        "adjusted_vs_raw": compare_frames(raw_frames, adjusted_frames),
    }

    (args.output_dir / "mujoco_uav_sim_summary.json").write_text(
        json.dumps(summary, indent=2),
        encoding="utf-8",
    )
    write_frames(args.output_dir / "mujoco_uav_sim_frames.csv", raw_frames, adjusted_frames)
    plot_results(args.output_dir / "mujoco_uav_sim_comparison.png", raw_frames, adjusted_frames)

    print(json.dumps(summary, indent=2))
    print(f"wrote {args.output_dir / 'mujoco_uav_sim_comparison.png'}")
    return 0


def load_trajectory_pair(path: Path, max_frames: int) -> tuple[list[TrajectoryPoint], list[TrajectoryPoint]]:
    if not path.exists():
        raise SystemExit(f"Trajectory CSV does not exist: {path}")

    raw_points: list[TrajectoryPoint] = []
    adjusted_points: list[TrajectoryPoint] = []
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
            raw_points.append(
                TrajectoryPoint(
                    timestamp_s=timestamp_s,
                    position_m=np.array(
                        [float(row["tx_raw"]), float(row["ty_raw"]), float(row["tz_raw"])],
                        dtype=float,
                    ),
                )
            )
            adjusted_points.append(
                TrajectoryPoint(
                    timestamp_s=timestamp_s,
                    position_m=np.array(
                        [
                            float(row["tx_adjusted"]),
                            float(row["ty_adjusted"]),
                            float(row["tz_adjusted"]),
                        ],
                        dtype=float,
                    ),
                )
            )

    if max_frames > 0:
        raw_points = raw_points[:max_frames]
        adjusted_points = adjusted_points[:max_frames]
    if len(raw_points) < 3:
        raise SystemExit("Need at least 3 trajectory points")

    return raw_points, adjusted_points


def simulate_variant(points: list[TrajectoryPoint], args: argparse.Namespace, sdk) -> list[SimFrame]:
    model = mujoco.MjModel.from_xml_string(MODEL_XML)
    data = mujoco.MjData(model)

    data.qpos[:3] = points[0].position_m
    data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0])
    mujoco.mj_forward(model, data)

    obs_builder = ObservationBuilder()
    frames: list[SimFrame] = []
    start_time_s = points[0].timestamp_s

    for index, point in enumerate(points):
        target_time_s = point.timestamp_s - start_time_s
        reference_velocity = estimate_reference_velocity(points, index)
        while data.time + 1e-9 < target_time_s:
            position_error = point.position_m - data.qpos[:3]
            velocity_error = reference_velocity - data.qvel[:3]
            force_n = args.kp * position_error + args.kd * velocity_error
            data.ctrl[:3] = np.clip(force_n, -args.max_force, args.max_force)
            mujoco.mj_step(model, data)

        force_n = np.array(data.ctrl[:3], dtype=float)
        state = flight_state_from_sim(point, points, index, data)
        obs = obs_builder.build(state)
        feedback = build_feedback(state)
        sdk_action: list[float] = []
        sdk_latency_ms = 0.0
        if sdk is not None:
            infer_started_at = time.perf_counter()
            sdk_action = sdk.infer(obs, feedback)
            sdk_latency_ms = (time.perf_counter() - infer_started_at) * 1000.0

        frames.append(
            SimFrame(
                time_s=target_time_s,
                target_m=point.position_m.copy(),
                position_m=np.array(data.qpos[:3], dtype=float),
                velocity_mps=np.array(data.qvel[:3], dtype=float),
                force_n=force_n,
                tracking_error_m=float(np.linalg.norm(point.position_m - data.qpos[:3])),
                sdk_action=sdk_action,
                sdk_latency_ms=sdk_latency_ms,
            )
        )

    return frames


def estimate_reference_velocity(points: list[TrajectoryPoint], index: int) -> np.ndarray:
    previous = points[max(0, index - 1)]
    following = points[min(len(points) - 1, index + 1)]
    dt_s = following.timestamp_s - previous.timestamp_s
    if dt_s <= 0.0:
        return np.zeros(3)
    return (following.position_m - previous.position_m) / dt_s


def flight_state_from_sim(
    point: TrajectoryPoint,
    points: list[TrajectoryPoint],
    index: int,
    data: mujoco.MjData,
) -> FlightState:
    velocity = np.array(data.qvel[:3], dtype=float)
    heading_deg = heading_from_velocity(velocity, points, index)
    target_heading_deg = target_heading(points, index)
    groundspeed_mps = float(np.linalg.norm(velocity[:2]))
    vertical_speed_mps = float(velocity[2])

    state = FlightState(
        roll=0.0,
        pitch=math.atan2(-vertical_speed_mps, max(groundspeed_mps, 0.1)),
        yaw=math.radians(heading_deg),
        roll_speed=0.0,
        pitch_speed=0.0,
        yaw_speed=0.0,
        heading=heading_deg,
        groundspeed=groundspeed_mps,
        lat=0.0,
        lon=0.0,
        relative_alt=float(point.position_m[2]),
        mode="MUJOCO_REPLAY",
        armed=False,
        battery_voltage=0.0,
    )
    state.heading_error_deg = wrap_deg(target_heading_deg - heading_deg)
    return state


def heading_from_velocity(velocity: np.ndarray, points: list[TrajectoryPoint], index: int) -> float:
    if abs(float(velocity[0])) + abs(float(velocity[1])) > 1e-6:
        return math.degrees(math.atan2(float(velocity[1]), float(velocity[0]))) % 360.0
    return target_heading(points, index)


def target_heading(points: list[TrajectoryPoint], index: int) -> float:
    next_index = min(len(points) - 1, index + 1)
    delta = points[next_index].position_m - points[index].position_m
    if abs(float(delta[0])) + abs(float(delta[1])) < 1e-9:
        return 0.0
    return math.degrees(math.atan2(float(delta[1]), float(delta[0]))) % 360.0


def summarize_frames(frames: list[SimFrame]) -> dict[str, float]:
    tracking_errors = [frame.tracking_error_m for frame in frames]
    speeds = [float(np.linalg.norm(frame.velocity_mps)) for frame in frames]
    force_norms = [float(np.linalg.norm(frame.force_n)) for frame in frames]
    action_norms = [float(np.linalg.norm(frame.sdk_action)) for frame in frames if frame.sdk_action]
    latencies = [frame.sdk_latency_ms for frame in frames if frame.sdk_action]
    return {
        "mean_tracking_error_m": mean(tracking_errors),
        "p95_tracking_error_m": percentile(tracking_errors, 95.0),
        "max_tracking_error_m": max(tracking_errors) if tracking_errors else 0.0,
        "mean_speed_mps": mean(speeds),
        "p95_speed_mps": percentile(speeds, 95.0),
        "mean_force_norm_n": mean(force_norms),
        "p95_force_norm_n": percentile(force_norms, 95.0),
        "mean_sdk_action_norm": mean(action_norms),
        "p95_sdk_action_norm": percentile(action_norms, 95.0),
        "mean_sdk_latency_ms": mean(latencies),
        "p95_sdk_latency_ms": percentile(latencies, 95.0),
    }


def compare_frames(raw_frames: list[SimFrame], adjusted_frames: list[SimFrame]) -> dict[str, float]:
    count = min(len(raw_frames), len(adjusted_frames))
    action_deltas = []
    for index in range(count):
        raw_action = raw_frames[index].sdk_action
        adjusted_action = adjusted_frames[index].sdk_action
        dim = min(len(raw_action), len(adjusted_action))
        if dim:
            action_deltas.append(
                math.sqrt(sum((raw_action[item] - adjusted_action[item]) ** 2 for item in range(dim)))
            )
    return {
        "mean_tracking_error_delta_m": mean(
            [adjusted_frames[index].tracking_error_m - raw_frames[index].tracking_error_m for index in range(count)]
        ),
        "mean_force_norm_delta_n": mean(
            [
                float(np.linalg.norm(adjusted_frames[index].force_n))
                - float(np.linalg.norm(raw_frames[index].force_n))
                for index in range(count)
            ]
        ),
        "mean_sdk_action_l2_delta": mean(action_deltas),
        "p95_sdk_action_l2_delta": percentile(action_deltas, 95.0),
    }


def write_frames(path: Path, raw_frames: list[SimFrame], adjusted_frames: list[SimFrame]) -> None:
    with path.open("w", encoding="utf-8", newline="") as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=[
                "time_s",
                "raw_tracking_error_m",
                "adjusted_tracking_error_m",
                "raw_force_norm_n",
                "adjusted_force_norm_n",
                "raw_speed_mps",
                "adjusted_speed_mps",
                "sdk_action_l2_delta",
            ],
        )
        writer.writeheader()
        for raw_frame, adjusted_frame in zip(raw_frames, adjusted_frames):
            dim = min(len(raw_frame.sdk_action), len(adjusted_frame.sdk_action))
            action_delta = (
                math.sqrt(
                    sum(
                        (raw_frame.sdk_action[index] - adjusted_frame.sdk_action[index]) ** 2
                        for index in range(dim)
                    )
                )
                if dim
                else 0.0
            )
            writer.writerow(
                {
                    "time_s": raw_frame.time_s,
                    "raw_tracking_error_m": raw_frame.tracking_error_m,
                    "adjusted_tracking_error_m": adjusted_frame.tracking_error_m,
                    "raw_force_norm_n": float(np.linalg.norm(raw_frame.force_n)),
                    "adjusted_force_norm_n": float(np.linalg.norm(adjusted_frame.force_n)),
                    "raw_speed_mps": float(np.linalg.norm(raw_frame.velocity_mps)),
                    "adjusted_speed_mps": float(np.linalg.norm(adjusted_frame.velocity_mps)),
                    "sdk_action_l2_delta": action_delta,
                }
            )


def plot_results(path: Path, raw_frames: list[SimFrame], adjusted_frames: list[SimFrame]) -> None:
    time_s = [frame.time_s for frame in raw_frames]
    raw_error = [frame.tracking_error_m for frame in raw_frames]
    adjusted_error = [frame.tracking_error_m for frame in adjusted_frames]
    raw_force = [float(np.linalg.norm(frame.force_n)) for frame in raw_frames]
    adjusted_force = [float(np.linalg.norm(frame.force_n)) for frame in adjusted_frames]
    raw_speed = [float(np.linalg.norm(frame.velocity_mps)) for frame in raw_frames]
    adjusted_speed = [float(np.linalg.norm(frame.velocity_mps)) for frame in adjusted_frames]

    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    axes[0, 0].plot([f.target_m[0] for f in raw_frames], [f.target_m[1] for f in raw_frames], label="raw target")
    axes[0, 0].plot([f.position_m[0] for f in raw_frames], [f.position_m[1] for f in raw_frames], label="raw sim")
    axes[0, 0].plot(
        [f.target_m[0] for f in adjusted_frames],
        [f.target_m[1] for f in adjusted_frames],
        label="adjusted target",
    )
    axes[0, 0].plot(
        [f.position_m[0] for f in adjusted_frames],
        [f.position_m[1] for f in adjusted_frames],
        label="adjusted sim",
    )
    axes[0, 0].set_title("XY tracking")
    axes[0, 0].set_xlabel("x [m]")
    axes[0, 0].set_ylabel("y [m]")
    axes[0, 0].axis("equal")
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].legend()

    axes[0, 1].plot(time_s, raw_error, label="raw")
    axes[0, 1].plot(time_s, adjusted_error, label="adjusted")
    axes[0, 1].set_title("Tracking error")
    axes[0, 1].set_xlabel("time [s]")
    axes[0, 1].set_ylabel("m")
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].legend()

    axes[1, 0].plot(time_s, raw_force, label="raw")
    axes[1, 0].plot(time_s, adjusted_force, label="adjusted")
    axes[1, 0].set_title("Control force norm")
    axes[1, 0].set_xlabel("time [s]")
    axes[1, 0].set_ylabel("N")
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].legend()

    axes[1, 1].plot(time_s, raw_speed, label="raw")
    axes[1, 1].plot(time_s, adjusted_speed, label="adjusted")
    axes[1, 1].set_title("Simulated speed")
    axes[1, 1].set_xlabel("time [s]")
    axes[1, 1].set_ylabel("m/s")
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].legend()

    fig.tight_layout()
    fig.savefig(path, dpi=180)


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
