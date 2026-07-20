#!/usr/bin/env python3
"""Render raw vs adjusted MuJoCo UAV trajectory tracking to an animation."""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path

import imageio.v2 as imageio
import mujoco
import numpy as np


@dataclass(frozen=True)
class TrajectoryPoint:
    timestamp_s: float
    position_m: np.ndarray


MODEL_XML = """
<mujoco model="uav_trajectory_render">
  <option timestep="0.005" gravity="0 0 0" integrator="RK4"/>
  <visual>
    <global offwidth="1280" offheight="720"/>
    <quality shadowsize="2048"/>
    <map force="0.1" znear="0.01" zfar="200"/>
  </visual>
  <worldbody>
    <light pos="0 -8 12" dir="0 1 -1"/>
    <geom name="ground" type="plane" size="90 90 0.05" rgba="0.08 0.09 0.10 1"/>
    <body name="raw_uav" pos="0 0 0">
      <joint name="raw_root" type="free"/>
      <geom type="sphere" size="0.22" mass="1.0" rgba="0.1 0.35 1.0 1"/>
      <geom type="box" pos="0.32 0 0" size="0.20 0.035 0.035" rgba="0.2 0.55 1.0 1"/>
    </body>
    <body name="adjusted_uav" pos="0 0 0">
      <joint name="adjusted_root" type="free"/>
      <geom type="sphere" size="0.22" mass="1.0" rgba="0.0 0.85 0.35 1"/>
      <geom type="box" pos="0.32 0 0" size="0.20 0.035 0.035" rgba="0.2 1.0 0.45 1"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="raw_fx" joint="raw_root" gear="1 0 0 0 0 0" ctrllimited="true" ctrlrange="-40 40"/>
    <motor name="raw_fy" joint="raw_root" gear="0 1 0 0 0 0" ctrllimited="true" ctrlrange="-40 40"/>
    <motor name="raw_fz" joint="raw_root" gear="0 0 1 0 0 0" ctrllimited="true" ctrlrange="-40 40"/>
    <motor name="adjusted_fx" joint="adjusted_root" gear="1 0 0 0 0 0" ctrllimited="true" ctrlrange="-40 40"/>
    <motor name="adjusted_fy" joint="adjusted_root" gear="0 1 0 0 0 0" ctrllimited="true" ctrlrange="-40 40"/>
    <motor name="adjusted_fz" joint="adjusted_root" gear="0 0 1 0 0 0" ctrllimited="true" ctrlrange="-40 40"/>
  </actuator>
</mujoco>
"""


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--trajectory-csv",
        type=Path,
        default=Path("outputs/uav_trajectory_adjustment/indoor_forward_6_segment_adjusted.csv"),
    )
    parser.add_argument("--output", type=Path, default=Path("outputs/mujoco_uav_sim/uav_trajectory_render.gif"))
    parser.add_argument("--max-frames", type=int, default=120)
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--kp", type=float, default=8.0)
    parser.add_argument("--kd", type=float, default=4.0)
    parser.add_argument("--max-force", type=float, default=35.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    raw_points, adjusted_points = load_trajectory_pair(args.trajectory_csv, args.max_frames)
    args.output.parent.mkdir(parents=True, exist_ok=True)

    model = mujoco.MjModel.from_xml_string(MODEL_XML)
    data = mujoco.MjData(model)
    data.qpos[0:3] = raw_points[0].position_m
    data.qpos[3:7] = np.array([1.0, 0.0, 0.0, 0.0])
    data.qpos[7:10] = adjusted_points[0].position_m
    data.qpos[10:14] = np.array([1.0, 0.0, 0.0, 0.0])
    mujoco.mj_forward(model, data)

    camera = make_camera(raw_points, adjusted_points)
    renderer = mujoco.Renderer(model, height=args.height, width=args.width)

    frames = []
    start_time_s = raw_points[0].timestamp_s
    render_period_s = 1.0 / args.fps
    next_render_time_s = 0.0
    point_index = 0
    last_time_s = raw_points[-1].timestamp_s - start_time_s

    while data.time <= last_time_s + 1e-9:
        while (
            point_index + 1 < len(raw_points)
            and raw_points[point_index + 1].timestamp_s - start_time_s <= data.time
        ):
            point_index += 1

        raw_target = raw_points[point_index]
        adjusted_target = adjusted_points[point_index]
        apply_pd_control(data, raw_target.position_m, 0, raw_points, point_index, args)
        apply_pd_control(data, adjusted_target.position_m, 3, adjusted_points, point_index, args)
        mujoco.mj_step(model, data)

        if data.time + 1e-9 >= next_render_time_s:
            renderer.update_scene(data, camera=camera)
            scene = renderer.scene
            add_path_geoms(scene, raw_points, rgba=np.array([0.15, 0.45, 1.0, 0.45]))
            add_path_geoms(scene, adjusted_points, rgba=np.array([0.0, 0.95, 0.35, 0.45]))
            add_marker(scene, raw_target.position_m, np.array([0.2, 0.55, 1.0, 0.9]))
            add_marker(scene, adjusted_target.position_m, np.array([0.0, 1.0, 0.35, 0.9]))
            frames.append(renderer.render())
            next_render_time_s += render_period_s

    if args.output.suffix.lower() == ".gif":
        imageio.mimsave(args.output, frames, fps=args.fps)
    else:
        imageio.mimsave(args.output, frames, fps=args.fps, quality=8)

    print(f"rendered {len(frames)} frames to {args.output}")
    return 0


def load_trajectory_pair(path: Path, max_frames: int) -> tuple[list[TrajectoryPoint], list[TrajectoryPoint]]:
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
    return raw_points, adjusted_points


def apply_pd_control(
    data: mujoco.MjData,
    target_m: np.ndarray,
    body_offset: int,
    points: list[TrajectoryPoint],
    point_index: int,
    args: argparse.Namespace,
) -> None:
    qpos_start = 0 if body_offset == 0 else 7
    qvel_start = body_offset
    ctrl_start = body_offset
    reference_velocity = estimate_reference_velocity(points, point_index)
    position_error = target_m - data.qpos[qpos_start : qpos_start + 3]
    velocity_error = reference_velocity - data.qvel[qvel_start : qvel_start + 3]
    force_n = args.kp * position_error + args.kd * velocity_error
    data.ctrl[ctrl_start : ctrl_start + 3] = np.clip(force_n, -args.max_force, args.max_force)


def estimate_reference_velocity(points: list[TrajectoryPoint], index: int) -> np.ndarray:
    previous = points[max(0, index - 1)]
    following = points[min(len(points) - 1, index + 1)]
    dt_s = following.timestamp_s - previous.timestamp_s
    if dt_s <= 0.0:
        return np.zeros(3)
    return (following.position_m - previous.position_m) / dt_s


def make_camera(raw_points: list[TrajectoryPoint], adjusted_points: list[TrajectoryPoint]) -> mujoco.MjvCamera:
    all_positions = np.array([point.position_m for point in raw_points + adjusted_points])
    center = all_positions.mean(axis=0)
    span = np.ptp(all_positions, axis=0)
    distance = max(float(np.linalg.norm(span[:2])) * 0.85, 12.0)

    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = center
    camera.distance = distance
    camera.azimuth = 135.0
    camera.elevation = -35.0
    return camera


def add_path_geoms(scene: mujoco.MjvScene, points: list[TrajectoryPoint], rgba: np.ndarray) -> None:
    for start, end in zip(points[:-1], points[1:]):
        add_capsule(scene, start.position_m, end.position_m, radius=0.035, rgba=rgba)


def add_marker(scene: mujoco.MjvScene, position_m: np.ndarray, rgba: np.ndarray) -> None:
    if scene.ngeom >= scene.maxgeom:
        return
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_SPHERE,
        np.array([0.12, 0.12, 0.12]),
        position_m,
        np.eye(3).reshape(-1),
        rgba,
    )
    scene.ngeom += 1


def add_capsule(scene: mujoco.MjvScene, start_m: np.ndarray, end_m: np.ndarray, radius: float, rgba: np.ndarray) -> None:
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
        radius,
        start_m.astype(float),
        end_m.astype(float),
    )
    scene.ngeom += 1


if __name__ == "__main__":
    raise SystemExit(main())
