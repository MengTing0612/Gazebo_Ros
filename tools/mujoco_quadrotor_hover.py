#!/usr/bin/env python3
"""MuJoCo quadrotor hover simulation driven by four motor speeds."""

from __future__ import annotations

import argparse
import csv
import json
import math
from dataclasses import dataclass
from pathlib import Path

import imageio.v2 as imageio
import mujoco
import numpy as np


MODEL_XML = """
<mujoco model="quadrotor_hover">
  <option timestep="0.002" gravity="0 0 -9.81" integrator="RK4"/>
  <visual>
    <global offwidth="1280" offheight="720"/>
  </visual>
  <worldbody>
    <light pos="0 -4 8" dir="0 0 -1"/>
    <geom type="plane" size="8 8 0.05" rgba="0.08 0.09 0.10 1"/>
    <body name="quad" pos="0 0 1">
      <joint name="root" type="free"/>
      <geom name="body" type="box" size="0.16 0.12 0.04" mass="0.65" rgba="0.15 0.35 0.95 1"/>
      <geom name="arm_x" type="box" size="0.35 0.025 0.018" mass="0.08" rgba="0.7 0.7 0.75 1"/>
      <geom name="arm_y" type="box" size="0.025 0.35 0.018" mass="0.08" rgba="0.7 0.7 0.75 1"/>
      <site name="m1_front" pos="0.35 0 0" size="0.045" rgba="1 0.25 0.2 1"/>
      <site name="m2_right" pos="0 -0.35 0" size="0.045" rgba="0.1 1 0.35 1"/>
      <site name="m3_back" pos="-0.35 0 0" size="0.045" rgba="1 0.25 0.2 1"/>
      <site name="m4_left" pos="0 0.35 0" size="0.045" rgba="0.1 1 0.35 1"/>
      <geom type="cylinder" pos="0.35 0 0" size="0.11 0.008" mass="0.01" rgba="0.95 0.95 0.95 0.75"/>
      <geom type="cylinder" pos="0 -0.35 0" size="0.11 0.008" mass="0.01" rgba="0.95 0.95 0.95 0.75"/>
      <geom type="cylinder" pos="-0.35 0 0" size="0.11 0.008" mass="0.01" rgba="0.95 0.95 0.95 0.75"/>
      <geom type="cylinder" pos="0 0.35 0" size="0.11 0.008" mass="0.01" rgba="0.95 0.95 0.95 0.75"/>
    </body>
  </worldbody>
</mujoco>
"""


@dataclass(frozen=True)
class QuadParams:
    arm_length_m: float = 0.35
    thrust_coeff_n_per_rad2: float = 1.8e-5
    yaw_moment_coeff_nm_per_rad2: float = 2.5e-7
    min_motor_radps: float = 0.0
    max_motor_radps: float = 900.0


@dataclass(frozen=True)
class HoverGains:
    z_kp: float = 6.0
    z_kd: float = 4.5
    angle_kp: float = 2.8
    rate_kd: float = 0.45
    yaw_kp: float = 1.5
    yaw_rate_kd: float = 0.18


@dataclass
class HoverFrame:
    time_s: float
    position_m: np.ndarray
    euler_rad: np.ndarray
    velocity_mps: np.ndarray
    angular_velocity_radps: np.ndarray
    motor_radps: np.ndarray
    altitude_error_m: float
    attitude_error_rad: float


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--target-z", type=float, default=1.5)
    parser.add_argument("--output-dir", type=Path, default=Path("outputs/mujoco_quad_hover"))
    parser.add_argument("--render", action="store_true")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=540)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    params = QuadParams()
    gains = HoverGains()
    frames, rendered_frames = simulate_hover(args, params, gains)

    summary = summarize(frames)
    summary["target_z_m"] = args.target_z
    summary["motor_model"] = {
        "thrust": "f_i = k_f * omega_i^2",
        "yaw_moment": "m_i = k_m * omega_i^2 with alternating spin directions",
        "motor_order": ["front", "right", "back", "left"],
        "units": "omega in rad/s",
    }
    (args.output_dir / "quad_hover_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    write_frames(args.output_dir / "quad_hover_frames.csv", frames)

    if rendered_frames:
        gif_path = args.output_dir / "quad_hover.gif"
        imageio.mimsave(gif_path, rendered_frames, fps=args.fps)
        summary["render"] = str(gif_path)
        (args.output_dir / "quad_hover_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")

    print(json.dumps(summary, indent=2))
    return 0


def simulate_hover(
    args: argparse.Namespace,
    params: QuadParams,
    gains: HoverGains,
) -> tuple[list[HoverFrame], list[np.ndarray]]:
    model = mujoco.MjModel.from_xml_string(MODEL_XML)
    data = mujoco.MjData(model)
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "quad")
    mass_kg = float(model.body_subtreemass[body_id])

    data.qpos[:3] = np.array([0.18, -0.12, 0.75])
    data.qpos[3:7] = euler_to_quat(np.array([math.radians(6.0), math.radians(-4.0), math.radians(12.0)]))
    data.qvel[:3] = np.array([0.0, 0.0, 0.0])
    data.qvel[3:6] = np.array([0.0, 0.0, 0.0])
    mujoco.mj_forward(model, data)

    renderer = None
    camera = None
    if args.render:
        renderer = mujoco.Renderer(model, height=args.height, width=args.width)
        camera = mujoco.MjvCamera()
        camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        camera.lookat[:] = np.array([0.0, 0.0, 1.0])
        camera.distance = 5.0
        camera.azimuth = 135.0
        camera.elevation = -25.0

    frames: list[HoverFrame] = []
    rendered_frames: list[np.ndarray] = []
    next_log_time = 0.0
    log_period = 0.02
    next_render_time = 0.0
    render_period = 1.0 / args.fps

    while data.time < args.duration:
        motor_radps = hover_controller(data, mass_kg, args.target_z, params, gains)
        apply_motor_forces(model, data, body_id, motor_radps, params)
        mujoco.mj_step(model, data)

        if data.time >= next_log_time:
            frames.append(make_frame(data, motor_radps, args.target_z))
            next_log_time += log_period

        if renderer is not None and camera is not None and data.time >= next_render_time:
            renderer.update_scene(data, camera=camera)
            rendered_frames.append(renderer.render())
            next_render_time += render_period

    return frames, rendered_frames


def hover_controller(
    data: mujoco.MjData,
    mass_kg: float,
    target_z_m: float,
    params: QuadParams,
    gains: HoverGains,
) -> np.ndarray:
    position = np.array(data.qpos[:3], dtype=float)
    velocity = np.array(data.qvel[:3], dtype=float)
    euler = quat_to_euler(np.array(data.qpos[3:7], dtype=float))
    angular_velocity = np.array(data.qvel[3:6], dtype=float)

    z_error = target_z_m - position[2]
    collective_thrust_n = mass_kg * 9.81 + gains.z_kp * z_error - gains.z_kd * velocity[2]
    collective_thrust_n = float(np.clip(collective_thrust_n, 0.0, 4.0 * max_motor_thrust(params)))

    target_torque = np.array(
        [
            -gains.angle_kp * euler[0] - gains.rate_kd * angular_velocity[0],
            -gains.angle_kp * euler[1] - gains.rate_kd * angular_velocity[1],
            -gains.yaw_kp * euler[2] - gains.yaw_rate_kd * angular_velocity[2],
        ],
        dtype=float,
    )
    return mix_to_motor_speeds(collective_thrust_n, target_torque, params)


def mix_to_motor_speeds(total_thrust_n: float, torque_body_nm: np.ndarray, params: QuadParams) -> np.ndarray:
    l = params.arm_length_m
    kf = params.thrust_coeff_n_per_rad2
    km = params.yaw_moment_coeff_nm_per_rad2
    mixer = np.array(
        [
            [1.0, 1.0, 1.0, 1.0],
            [0.0, -l, 0.0, l],
            [-l, 0.0, l, 0.0],
            [km / kf, -km / kf, km / kf, -km / kf],
        ],
        dtype=float,
    )
    desired = np.array([total_thrust_n, *torque_body_nm], dtype=float)
    thrusts_n = np.linalg.solve(mixer, desired)
    thrusts_n = np.clip(thrusts_n, 0.0, max_motor_thrust(params))
    return np.sqrt(thrusts_n / params.thrust_coeff_n_per_rad2)


def apply_motor_forces(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    body_id: int,
    motor_radps: np.ndarray,
    params: QuadParams,
) -> None:
    data.xfrc_applied[:, :] = 0.0
    thrusts_n = params.thrust_coeff_n_per_rad2 * motor_radps**2
    total_body_force = np.array([0.0, 0.0, float(np.sum(thrusts_n))])
    l = params.arm_length_m
    body_torque = np.array(
        [
            l * (thrusts_n[3] - thrusts_n[1]),
            l * (thrusts_n[2] - thrusts_n[0]),
            params.yaw_moment_coeff_nm_per_rad2
            * (motor_radps[0] ** 2 - motor_radps[1] ** 2 + motor_radps[2] ** 2 - motor_radps[3] ** 2),
        ],
        dtype=float,
    )
    rotation = np.array(data.xmat[body_id], dtype=float).reshape(3, 3)
    world_force = rotation @ total_body_force
    world_torque = rotation @ body_torque
    data.xfrc_applied[body_id, :3] = world_force
    data.xfrc_applied[body_id, 3:] = world_torque


def make_frame(data: mujoco.MjData, motor_radps: np.ndarray, target_z_m: float) -> HoverFrame:
    euler = quat_to_euler(np.array(data.qpos[3:7], dtype=float))
    return HoverFrame(
        time_s=float(data.time),
        position_m=np.array(data.qpos[:3], dtype=float),
        euler_rad=euler,
        velocity_mps=np.array(data.qvel[:3], dtype=float),
        angular_velocity_radps=np.array(data.qvel[3:6], dtype=float),
        motor_radps=np.array(motor_radps, dtype=float),
        altitude_error_m=float(target_z_m - data.qpos[2]),
        attitude_error_rad=float(np.linalg.norm(euler)),
    )


def write_frames(path: Path, frames: list[HoverFrame]) -> None:
    with path.open("w", encoding="utf-8", newline="") as csv_file:
        writer = csv.DictWriter(
            csv_file,
            fieldnames=[
                "time_s",
                "x_m",
                "y_m",
                "z_m",
                "roll_rad",
                "pitch_rad",
                "yaw_rad",
                "altitude_error_m",
                "attitude_error_rad",
                "motor_front_radps",
                "motor_right_radps",
                "motor_back_radps",
                "motor_left_radps",
            ],
        )
        writer.writeheader()
        for frame in frames:
            writer.writerow(
                {
                    "time_s": frame.time_s,
                    "x_m": frame.position_m[0],
                    "y_m": frame.position_m[1],
                    "z_m": frame.position_m[2],
                    "roll_rad": frame.euler_rad[0],
                    "pitch_rad": frame.euler_rad[1],
                    "yaw_rad": frame.euler_rad[2],
                    "altitude_error_m": frame.altitude_error_m,
                    "attitude_error_rad": frame.attitude_error_rad,
                    "motor_front_radps": frame.motor_radps[0],
                    "motor_right_radps": frame.motor_radps[1],
                    "motor_back_radps": frame.motor_radps[2],
                    "motor_left_radps": frame.motor_radps[3],
                }
            )


def summarize(frames: list[HoverFrame]) -> dict[str, float]:
    altitude_errors = [abs(frame.altitude_error_m) for frame in frames]
    attitude_errors = [frame.attitude_error_rad for frame in frames]
    motor_values = [float(value) for frame in frames for value in frame.motor_radps]
    final = frames[-1]
    return {
        "duration_s": frames[-1].time_s if frames else 0.0,
        "mean_abs_altitude_error_m": mean(altitude_errors),
        "p95_abs_altitude_error_m": percentile(altitude_errors, 95.0),
        "final_abs_altitude_error_m": abs(final.altitude_error_m),
        "mean_attitude_error_rad": mean(attitude_errors),
        "p95_attitude_error_rad": percentile(attitude_errors, 95.0),
        "final_attitude_error_rad": final.attitude_error_rad,
        "mean_motor_radps": mean(motor_values),
        "min_motor_radps": min(motor_values),
        "max_motor_radps": max(motor_values),
    }


def max_motor_thrust(params: QuadParams) -> float:
    return params.thrust_coeff_n_per_rad2 * params.max_motor_radps**2


def euler_to_quat(euler_rad: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = euler_rad
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return np.array(
        [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ],
        dtype=float,
    )


def quat_to_euler(quat_wxyz: np.ndarray) -> np.ndarray:
    w, x, y, z = quat_wxyz / np.linalg.norm(quat_wxyz)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.array([roll, pitch, yaw], dtype=float)


def mean(values: list[float]) -> float:
    return sum(values) / len(values) if values else 0.0


def percentile(values: list[float], percentile_value: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = (len(ordered) - 1) * percentile_value / 100.0
    lower = int(index)
    upper = min(lower + 1, len(ordered) - 1)
    fraction = index - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


if __name__ == "__main__":
    raise SystemExit(main())
