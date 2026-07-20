#!/usr/bin/env python3
"""Track dataset UAV trajectories with a four-motor MuJoCo quadrotor model."""

from __future__ import annotations

if __package__ in {None, ""}:
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path

import imageio.v2 as imageio
import mujoco
import numpy as np

from tools.mujoco_quadrotor_hover import (
    MODEL_XML,
    HoverGains,
    QuadParams,
    apply_motor_forces,
    euler_to_quat,
    max_motor_thrust,
    mix_to_motor_speeds,
    percentile,
    quat_to_euler,
)


@dataclass(frozen=True)
class TrajectoryPoint:
    timestamp_s: float
    position_m: np.ndarray


@dataclass
class TrackFrame:
    time_s: float
    target_m: np.ndarray
    position_m: np.ndarray
    euler_rad: np.ndarray
    velocity_mps: np.ndarray
    motor_radps: np.ndarray
    tracking_error_m: float
    force_command_n: np.ndarray


@dataclass(frozen=True)
class PositionGains:
    xy_kp: float = 1.6
    xy_kd: float = 2.2
    z_kp: float = 7.0
    z_kd: float = 5.0
    max_tilt_rad: float = math.radians(30.0)


DUAL_MODEL_XML = """
<mujoco model="quadrotor_dataset_tracking">
  <option timestep="0.002" gravity="0 0 -9.81" integrator="RK4"/>
  <visual>
    <global offwidth="1280" offheight="720"/>
  </visual>
  <worldbody>
    <light pos="0 -8 12" dir="0 0 -1"/>
    <geom type="plane" size="80 80 0.05" rgba="0.08 0.09 0.10 1"/>
    <body name="raw_quad" pos="0 0 1">
      <joint name="raw_root" type="free"/>
      <geom contype="0" conaffinity="0" type="box" size="0.16 0.12 0.04" mass="0.65" rgba="0.12 0.34 1.0 1"/>
      <geom contype="0" conaffinity="0" type="box" size="0.35 0.025 0.018" mass="0.08" rgba="0.7 0.7 0.75 1"/>
      <geom contype="0" conaffinity="0" type="box" size="0.025 0.35 0.018" mass="0.08" rgba="0.7 0.7 0.75 1"/>
      <geom contype="0" conaffinity="0" type="cylinder" pos="0.35 0 0" size="0.11 0.008" mass="0.01" rgba="0.95 0.95 0.95 0.75"/>
      <geom contype="0" conaffinity="0" type="cylinder" pos="0 -0.35 0" size="0.11 0.008" mass="0.01" rgba="0.95 0.95 0.95 0.75"/>
      <geom contype="0" conaffinity="0" type="cylinder" pos="-0.35 0 0" size="0.11 0.008" mass="0.01" rgba="0.95 0.95 0.95 0.75"/>
      <geom contype="0" conaffinity="0" type="cylinder" pos="0 0.35 0" size="0.11 0.008" mass="0.01" rgba="0.95 0.95 0.95 0.75"/>
    </body>
    <body name="adjusted_quad" pos="0 0 1">
      <joint name="adjusted_root" type="free"/>
      <geom contype="0" conaffinity="0" type="box" size="0.16 0.12 0.04" mass="0.65" rgba="0.0 0.85 0.35 1"/>
      <geom contype="0" conaffinity="0" type="box" size="0.35 0.025 0.018" mass="0.08" rgba="0.7 0.7 0.75 1"/>
      <geom contype="0" conaffinity="0" type="box" size="0.025 0.35 0.018" mass="0.08" rgba="0.7 0.7 0.75 1"/>
      <geom contype="0" conaffinity="0" type="cylinder" pos="0.35 0 0" size="0.11 0.008" mass="0.01" rgba="0.95 0.95 0.95 0.75"/>
      <geom contype="0" conaffinity="0" type="cylinder" pos="0 -0.35 0" size="0.11 0.008" mass="0.01" rgba="0.95 0.95 0.95 0.75"/>
      <geom contype="0" conaffinity="0" type="cylinder" pos="-0.35 0 0" size="0.11 0.008" mass="0.01" rgba="0.95 0.95 0.95 0.75"/>
      <geom contype="0" conaffinity="0" type="cylinder" pos="0 0.35 0" size="0.11 0.008" mass="0.01" rgba="0.95 0.95 0.95 0.75"/>
    </body>
  </worldbody>
</mujoco>
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--trajectory-csv",
        type=Path,
        default=Path("outputs/uav_trajectory_adjustment/indoor_forward_6_segment_adjusted.csv"),
    )
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/mujoco_quad_dataset"))
    parser.add_argument("--max-frames", type=int, default=120)
    parser.add_argument("--render", action="store_true")
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--space-scale", type=float, default=0.25, help="Scale dataset displacement into a smaller local flight volume")
    parser.add_argument("--time-scale", type=float, default=4.0, help="Slow down dataset timestamps for trackable quadrotor motion")
    parser.add_argument("--variant", choices=["raw", "adjusted", "both"], default="both")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    raw_points, adjusted_points = load_trajectory_pair(
        args.trajectory_csv,
        args.max_frames,
        args.space_scale,
        args.time_scale,
    )

    params = QuadParams()
    hover_gains = HoverGains()
    position_gains = PositionGains()

    if args.variant in {"raw", "adjusted"}:
        selected_points = raw_points if args.variant == "raw" else adjusted_points
        frames, rendered = simulate_single(selected_points, args, params, hover_gains, position_gains, args.variant)
        summary = {
            "model": "four-motor quadrotor tracking dataset trajectory",
            "variant": args.variant,
            "coordinate_note": "dataset trajectory translated to a local origin, spatially scaled, and time-scaled for trackable quadrotor motion",
            "space_scale": args.space_scale,
            "time_scale": args.time_scale,
            args.variant: summarize(frames),
        }
        (args.output_dir / f"quad_dataset_{args.variant}_summary.json").write_text(
            json.dumps(summary, indent=2),
            encoding="utf-8",
        )
        write_single_frames(args.output_dir / f"quad_dataset_{args.variant}_frames.csv", frames)
        if rendered:
            gif_path = args.output_dir / f"quad_dataset_{args.variant}.gif"
            imageio.mimsave(gif_path, rendered, fps=args.fps)
            summary["render"] = str(gif_path)
            (args.output_dir / f"quad_dataset_{args.variant}_summary.json").write_text(
                json.dumps(summary, indent=2),
                encoding="utf-8",
            )
        print(json.dumps(summary, indent=2))
        return 0

    frames_raw, frames_adjusted, rendered = simulate_pair(raw_points, adjusted_points, args, params, hover_gains, position_gains)

    summary = {
        "model": "four-motor quadrotor tracking dataset trajectory",
        "coordinate_note": "dataset trajectory translated to a local origin, spatially scaled, and time-scaled for trackable quadrotor motion",
        "space_scale": args.space_scale,
        "time_scale": args.time_scale,
        "raw": summarize(frames_raw),
        "adjusted": summarize(frames_adjusted),
        "adjusted_vs_raw": {
            "mean_tracking_error_delta_m": mean(
                [a.tracking_error_m - r.tracking_error_m for r, a in zip(frames_raw, frames_adjusted)]
            ),
            "mean_motor_radps_delta": mean(
                [float(np.mean(a.motor_radps) - np.mean(r.motor_radps)) for r, a in zip(frames_raw, frames_adjusted)]
            ),
        },
    }
    (args.output_dir / "quad_dataset_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    write_frames(args.output_dir / "quad_dataset_frames.csv", frames_raw, frames_adjusted)

    if rendered:
        gif_path = args.output_dir / "quad_dataset_tracking.gif"
        imageio.mimsave(gif_path, rendered, fps=args.fps)
        summary["render"] = str(gif_path)
        (args.output_dir / "quad_dataset_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")

    print(json.dumps(summary, indent=2))
    return 0


def simulate_single(
    points: list[TrajectoryPoint],
    args: argparse.Namespace,
    params: QuadParams,
    hover_gains: HoverGains,
    position_gains: PositionGains,
    variant: str,
) -> tuple[list[TrackFrame], list[np.ndarray]]:
    model = mujoco.MjModel.from_xml_string(DUAL_MODEL_XML)
    data = mujoco.MjData(model)
    body_name = "raw_quad" if variant == "raw" else "adjusted_quad"
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    mass = float(model.body_subtreemass[body_id])
    qpos_offset = 0 if variant == "raw" else 7

    set_quad_state(data, qpos_offset, points[0].position_m)
    mujoco.mj_forward(model, data)

    renderer = None
    camera = None
    rendered: list[np.ndarray] = []
    if args.render:
        renderer = mujoco.Renderer(model, height=args.height, width=args.width)
        camera = make_camera(points, points)

    frames: list[TrackFrame] = []
    start_time = points[0].timestamp_s
    next_log_time = 0.0
    log_period = 0.02
    next_render_time = 0.0
    render_period = 1.0 / args.fps
    point_index = 0
    end_time = points[-1].timestamp_s - start_time

    while data.time <= end_time + 1e-9:
        while point_index + 1 < len(points) and points[point_index + 1].timestamp_s - start_time <= data.time:
            point_index += 1

        data.xfrc_applied[:, :] = 0.0
        motor_radps, force_command = trajectory_controller(
            data, qpos_offset, points, point_index, mass, params, hover_gains, position_gains
        )
        apply_motor_forces_at_offset(model, data, body_id, motor_radps, params)
        mujoco.mj_step(model, data)

        if data.time >= next_log_time:
            frames.append(make_frame(data, qpos_offset, points[point_index].position_m, motor_radps, force_command))
            next_log_time += log_period

        if renderer is not None and camera is not None and data.time >= next_render_time:
            renderer.update_scene(data, camera=camera)
            rgba = np.array([0.15, 0.45, 1.0, 0.45]) if variant == "raw" else np.array([0.0, 0.95, 0.35, 0.45])
            add_path_geoms(renderer.scene, points, rgba)
            rendered.append(renderer.render())
            next_render_time += render_period

    return frames, rendered


def load_trajectory_pair(
    path: Path,
    max_frames: int,
    space_scale: float,
    time_scale: float,
) -> tuple[list[TrajectoryPoint], list[TrajectoryPoint]]:
    raw_points: list[TrajectoryPoint] = []
    adjusted_points: list[TrajectoryPoint] = []
    with path.open(encoding="utf-8", newline="") as csv_file:
        reader = csv.DictReader(csv_file)
        for row in reader:
            timestamp_s = float(row["timestamp"])
            raw_points.append(
                TrajectoryPoint(
                    timestamp_s,
                    np.array([float(row["tx_raw"]), float(row["ty_raw"]), float(row["tz_raw"])], dtype=float),
                )
            )
            adjusted_points.append(
                TrajectoryPoint(
                    timestamp_s,
                    np.array(
                        [float(row["tx_adjusted"]), float(row["ty_adjusted"]), float(row["tz_adjusted"])],
                        dtype=float,
                    ),
                )
            )
    if max_frames > 0:
        raw_points = raw_points[:max_frames]
        adjusted_points = adjusted_points[:max_frames]

    origin_position = raw_points[0].position_m.copy()
    origin_time = raw_points[0].timestamp_s
    raw_points = normalize_points(raw_points, origin_position, origin_time, space_scale, time_scale)
    adjusted_points = normalize_points(adjusted_points, origin_position, origin_time, space_scale, time_scale)
    return raw_points, adjusted_points


def normalize_points(
    points: list[TrajectoryPoint],
    origin_position: np.ndarray,
    origin_time: float,
    space_scale: float,
    time_scale: float,
) -> list[TrajectoryPoint]:
    return [
        TrajectoryPoint(
            timestamp_s=origin_time + (point.timestamp_s - origin_time) * time_scale,
            position_m=np.array(
                [
                    (point.position_m[0] - origin_position[0]) * space_scale,
                    (point.position_m[1] - origin_position[1]) * space_scale,
                    (point.position_m[2] - origin_position[2]) * space_scale + 1.5,
                ],
                dtype=float,
            ),
        )
        for point in points
    ]


def simulate_pair(
    raw_points: list[TrajectoryPoint],
    adjusted_points: list[TrajectoryPoint],
    args: argparse.Namespace,
    params: QuadParams,
    hover_gains: HoverGains,
    position_gains: PositionGains,
) -> tuple[list[TrackFrame], list[TrackFrame], list[np.ndarray]]:
    model = mujoco.MjModel.from_xml_string(DUAL_MODEL_XML)
    data = mujoco.MjData(model)
    raw_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "raw_quad")
    adjusted_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "adjusted_quad")
    raw_mass = float(model.body_subtreemass[raw_body_id])
    adjusted_mass = float(model.body_subtreemass[adjusted_body_id])

    set_quad_state(data, 0, raw_points[0].position_m)
    set_quad_state(data, 7, adjusted_points[0].position_m)
    mujoco.mj_forward(model, data)

    renderer = None
    camera = None
    rendered: list[np.ndarray] = []
    if args.render:
        renderer = mujoco.Renderer(model, height=args.height, width=args.width)
        camera = make_camera(raw_points, adjusted_points)

    frames_raw: list[TrackFrame] = []
    frames_adjusted: list[TrackFrame] = []
    start_time = raw_points[0].timestamp_s
    next_log_time = 0.0
    log_period = 0.02
    next_render_time = 0.0
    render_period = 1.0 / args.fps
    point_index = 0
    end_time = raw_points[-1].timestamp_s - start_time

    while data.time <= end_time + 1e-9:
        while point_index + 1 < len(raw_points) and raw_points[point_index + 1].timestamp_s - start_time <= data.time:
            point_index += 1

        raw_motor, raw_force = trajectory_controller(
            data, 0, raw_points, point_index, raw_mass, params, hover_gains, position_gains
        )
        adjusted_motor, adjusted_force = trajectory_controller(
            data, 7, adjusted_points, point_index, adjusted_mass, params, hover_gains, position_gains
        )
        apply_motor_forces_at_offset(model, data, raw_body_id, raw_motor, params)
        apply_motor_forces_at_offset(model, data, adjusted_body_id, adjusted_motor, params)
        mujoco.mj_step(model, data)

        if data.time >= next_log_time:
            frames_raw.append(make_frame(data, 0, raw_points[point_index].position_m, raw_motor, raw_force))
            frames_adjusted.append(
                make_frame(data, 7, adjusted_points[point_index].position_m, adjusted_motor, adjusted_force)
            )
            next_log_time += log_period

        if renderer is not None and camera is not None and data.time >= next_render_time:
            renderer.update_scene(data, camera=camera)
            scene = renderer.scene
            add_path_geoms(scene, raw_points, np.array([0.15, 0.45, 1.0, 0.45]))
            add_path_geoms(scene, adjusted_points, np.array([0.0, 0.95, 0.35, 0.45]))
            rendered.append(renderer.render())
            next_render_time += render_period

    return frames_raw, frames_adjusted, rendered


def set_quad_state(data: mujoco.MjData, qpos_offset: int, position_m: np.ndarray) -> None:
    data.qpos[qpos_offset : qpos_offset + 3] = position_m
    data.qpos[qpos_offset + 3 : qpos_offset + 7] = euler_to_quat(np.zeros(3))


def trajectory_controller(
    data: mujoco.MjData,
    qpos_offset: int,
    points: list[TrajectoryPoint],
    index: int,
    mass_kg: float,
    params: QuadParams,
    hover_gains: HoverGains,
    position_gains: PositionGains,
) -> tuple[np.ndarray, np.ndarray]:
    qvel_offset = 0 if qpos_offset == 0 else 6
    position = np.array(data.qpos[qpos_offset : qpos_offset + 3], dtype=float)
    velocity = np.array(data.qvel[qvel_offset : qvel_offset + 3], dtype=float)
    euler = quat_to_euler(np.array(data.qpos[qpos_offset + 3 : qpos_offset + 7], dtype=float))
    angular_velocity = np.array(data.qvel[qvel_offset + 3 : qvel_offset + 6], dtype=float)

    target = points[index].position_m
    target_velocity = estimate_velocity(points, index)
    pos_error = target - position
    vel_error = target_velocity - velocity

    desired_accel = np.array(
        [
            position_gains.xy_kp * pos_error[0] + position_gains.xy_kd * vel_error[0],
            position_gains.xy_kp * pos_error[1] + position_gains.xy_kd * vel_error[1],
            position_gains.z_kp * pos_error[2] + position_gains.z_kd * vel_error[2],
        ],
        dtype=float,
    )
    desired_roll = np.clip(-desired_accel[1] / 9.81, -position_gains.max_tilt_rad, position_gains.max_tilt_rad)
    desired_pitch = np.clip(desired_accel[0] / 9.81, -position_gains.max_tilt_rad, position_gains.max_tilt_rad)
    desired_yaw = target_heading(points, index)
    yaw_error = wrap_angle(desired_yaw - euler[2])

    collective_thrust = mass_kg * (9.81 + desired_accel[2])
    collective_thrust = float(np.clip(collective_thrust, 0.0, 4.0 * max_motor_thrust(params)))
    torque = np.array(
        [
            hover_gains.angle_kp * (desired_roll - euler[0]) - hover_gains.rate_kd * angular_velocity[0],
            hover_gains.angle_kp * (desired_pitch - euler[1]) - hover_gains.rate_kd * angular_velocity[1],
            hover_gains.yaw_kp * yaw_error - hover_gains.yaw_rate_kd * angular_velocity[2],
        ],
        dtype=float,
    )
    return mix_to_motor_speeds(collective_thrust, torque, params), mass_kg * desired_accel


def apply_motor_forces_at_offset(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    body_id: int,
    motor_radps: np.ndarray,
    params: QuadParams,
) -> None:
    thrusts = params.thrust_coeff_n_per_rad2 * motor_radps**2
    l = params.arm_length_m
    body_force = np.array([0.0, 0.0, float(np.sum(thrusts))])
    body_torque = np.array(
        [
            l * (thrusts[3] - thrusts[1]),
            l * (thrusts[2] - thrusts[0]),
            params.yaw_moment_coeff_nm_per_rad2
            * (motor_radps[0] ** 2 - motor_radps[1] ** 2 + motor_radps[2] ** 2 - motor_radps[3] ** 2),
        ],
        dtype=float,
    )
    rotation = np.array(data.xmat[body_id], dtype=float).reshape(3, 3)
    data.xfrc_applied[body_id, :3] = rotation @ body_force
    data.xfrc_applied[body_id, 3:] = rotation @ body_torque


def make_frame(
    data: mujoco.MjData,
    qpos_offset: int,
    target_m: np.ndarray,
    motor_radps: np.ndarray,
    force_command_n: np.ndarray,
) -> TrackFrame:
    qvel_offset = 0 if qpos_offset == 0 else 6
    position = np.array(data.qpos[qpos_offset : qpos_offset + 3], dtype=float)
    return TrackFrame(
        time_s=float(data.time),
        target_m=target_m.copy(),
        position_m=position,
        euler_rad=quat_to_euler(np.array(data.qpos[qpos_offset + 3 : qpos_offset + 7], dtype=float)),
        velocity_mps=np.array(data.qvel[qvel_offset : qvel_offset + 3], dtype=float),
        motor_radps=motor_radps.copy(),
        tracking_error_m=float(np.linalg.norm(target_m - position)),
        force_command_n=force_command_n.copy(),
    )


def estimate_velocity(points: list[TrajectoryPoint], index: int) -> np.ndarray:
    previous = points[max(0, index - 1)]
    following = points[min(len(points) - 1, index + 1)]
    dt = following.timestamp_s - previous.timestamp_s
    if dt <= 0.0:
        return np.zeros(3)
    return (following.position_m - previous.position_m) / dt


def target_heading(points: list[TrajectoryPoint], index: int) -> float:
    velocity = estimate_velocity(points, index)
    if abs(float(velocity[0])) + abs(float(velocity[1])) < 1e-9:
        return 0.0
    return math.atan2(float(velocity[1]), float(velocity[0]))


def summarize(frames: list[TrackFrame]) -> dict[str, float]:
    errors = [frame.tracking_error_m for frame in frames]
    motor_values = [float(value) for frame in frames for value in frame.motor_radps]
    force_norms = [float(np.linalg.norm(frame.force_command_n)) for frame in frames]
    attitude_errors = [float(np.linalg.norm(frame.euler_rad[:2])) for frame in frames]
    return {
        "mean_tracking_error_m": mean(errors),
        "p95_tracking_error_m": percentile(errors, 95.0),
        "max_tracking_error_m": max(errors),
        "mean_motor_radps": mean(motor_values),
        "max_motor_radps": max(motor_values),
        "mean_force_command_n": mean(force_norms),
        "p95_force_command_n": percentile(force_norms, 95.0),
        "mean_roll_pitch_error_rad": mean(attitude_errors),
        "p95_roll_pitch_error_rad": percentile(attitude_errors, 95.0),
    }


def write_frames(path: Path, raw_frames: list[TrackFrame], adjusted_frames: list[TrackFrame]) -> None:
    with path.open("w", encoding="utf-8", newline="") as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=[
                "time_s",
                "raw_tracking_error_m",
                "adjusted_tracking_error_m",
                "raw_motor_mean_radps",
                "adjusted_motor_mean_radps",
                "raw_x_m",
                "raw_y_m",
                "raw_z_m",
                "adjusted_x_m",
                "adjusted_y_m",
                "adjusted_z_m",
            ],
        )
        writer.writeheader()
        for raw, adjusted in zip(raw_frames, adjusted_frames):
            writer.writerow(
                {
                    "time_s": raw.time_s,
                    "raw_tracking_error_m": raw.tracking_error_m,
                    "adjusted_tracking_error_m": adjusted.tracking_error_m,
                    "raw_motor_mean_radps": float(np.mean(raw.motor_radps)),
                    "adjusted_motor_mean_radps": float(np.mean(adjusted.motor_radps)),
                    "raw_x_m": raw.position_m[0],
                    "raw_y_m": raw.position_m[1],
                    "raw_z_m": raw.position_m[2],
                    "adjusted_x_m": adjusted.position_m[0],
                    "adjusted_y_m": adjusted.position_m[1],
                    "adjusted_z_m": adjusted.position_m[2],
                }
            )


def write_single_frames(path: Path, frames: list[TrackFrame]) -> None:
    with path.open("w", encoding="utf-8", newline="") as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=[
                "time_s",
                "tracking_error_m",
                "motor_front_radps",
                "motor_right_radps",
                "motor_back_radps",
                "motor_left_radps",
                "x_m",
                "y_m",
                "z_m",
                "target_x_m",
                "target_y_m",
                "target_z_m",
                "roll_rad",
                "pitch_rad",
                "yaw_rad",
            ],
        )
        writer.writeheader()
        for frame in frames:
            writer.writerow(
                {
                    "time_s": frame.time_s,
                    "tracking_error_m": frame.tracking_error_m,
                    "motor_front_radps": frame.motor_radps[0],
                    "motor_right_radps": frame.motor_radps[1],
                    "motor_back_radps": frame.motor_radps[2],
                    "motor_left_radps": frame.motor_radps[3],
                    "x_m": frame.position_m[0],
                    "y_m": frame.position_m[1],
                    "z_m": frame.position_m[2],
                    "target_x_m": frame.target_m[0],
                    "target_y_m": frame.target_m[1],
                    "target_z_m": frame.target_m[2],
                    "roll_rad": frame.euler_rad[0],
                    "pitch_rad": frame.euler_rad[1],
                    "yaw_rad": frame.euler_rad[2],
                }
            )


def make_camera(raw_points: list[TrajectoryPoint], adjusted_points: list[TrajectoryPoint]) -> mujoco.MjvCamera:
    all_positions = np.array([point.position_m for point in raw_points + adjusted_points])
    center = all_positions.mean(axis=0)
    span = np.ptp(all_positions, axis=0)
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = center
    camera.distance = max(float(np.linalg.norm(span[:2])) * 0.85, 12.0)
    camera.azimuth = 135.0
    camera.elevation = -32.0
    return camera


def add_path_geoms(scene: mujoco.MjvScene, points: list[TrajectoryPoint], rgba: np.ndarray) -> None:
    for start, end in zip(points[:-1], points[1:]):
        if scene.ngeom >= scene.maxgeom:
            return
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(
            geom,
            mujoco.mjtGeom.mjGEOM_CAPSULE,
            np.zeros(3),
            np.zeros(3),
            np.eye(3).reshape(-1),
            rgba,
        )
        mujoco.mjv_connector(
            geom,
            mujoco.mjtGeom.mjGEOM_CAPSULE,
            0.035,
            start.position_m.astype(float),
            end.position_m.astype(float),
        )
        scene.ngeom += 1


def wrap_angle(angle_rad: float) -> float:
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else 0.0


if __name__ == "__main__":
    raise SystemExit(main())
