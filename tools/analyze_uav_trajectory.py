#!/usr/bin/env python3
import json
from pathlib import Path

import numpy as np
import pandas as pd

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def centered_moving_average(values, window):
    if window % 2 == 0 or window < 3:
        raise ValueError("window must be an odd integer >= 3")
    pad = window // 2
    padded = np.pad(values, ((pad, pad), (0, 0)), mode="edge")
    kernel = np.ones(window, dtype=float) / window
    cols = [np.convolve(padded[:, i], kernel, mode="valid") for i in range(values.shape[1])]
    return np.column_stack(cols)


def metrics(t, xyz):
    dt = np.diff(t)
    dxyz = np.diff(xyz, axis=0)
    step = np.linalg.norm(dxyz, axis=1)
    velocity_vec = dxyz / dt[:, None]
    speed = np.linalg.norm(velocity_vec, axis=1)
    accel_vec = np.diff(velocity_vec, axis=0) / dt[1:, None]
    accel = np.linalg.norm(accel_vec, axis=1)
    jerk_vec = np.diff(accel_vec, axis=0) / dt[2:, None]
    jerk = np.linalg.norm(jerk_vec, axis=1)
    return {
        "points": int(len(t)),
        "duration_s": float(t[-1] - t[0]),
        "path_length_m": float(step.sum()),
        "mean_speed_mps": float(speed.mean()),
        "p95_speed_mps": float(np.percentile(speed, 95)),
        "max_speed_mps": float(speed.max()),
        "p95_accel_mps2": float(np.percentile(accel, 95)),
        "max_accel_mps2": float(accel.max()),
        "p95_jerk_mps3": float(np.percentile(jerk, 95)),
        "max_jerk_mps3": float(jerk.max()),
    }


def preserve_endpoints_smoothly(raw, smoothed):
    start_error = raw[0] - smoothed[0]
    end_error = raw[-1] - smoothed[-1]
    alpha = np.linspace(0.0, 1.0, len(raw))[:, None]
    correction = (1.0 - alpha) * start_error + alpha * end_error
    return smoothed + correction


def main():
    source = Path("data/drone_trajectories-main/trajectories/fpv_uzh/indoor_forward_6_snapdragon_with_gt.txt")
    out_dir = Path("outputs/uav_trajectory_adjustment")
    out_dir.mkdir(parents=True, exist_ok=True)

    df = pd.read_csv(source)
    df["t_rel"] = df["timestamp"] - df["timestamp"].iloc[0]
    segment = df[(df["t_rel"] >= 8.0) & (df["t_rel"] <= 20.0)].copy().reset_index(drop=True)

    t = segment["t_rel"].to_numpy()
    xyz = segment[["tx", "ty", "tz"]].to_numpy()

    smoothed = centered_moving_average(xyz, window=5)
    # Keep endpoints fixed so the adjusted segment can be stitched back into the original trajectory.
    smoothed = preserve_endpoints_smoothly(xyz, smoothed)

    adjusted = segment.copy()
    adjusted[["tx_adjusted", "ty_adjusted", "tz_adjusted"]] = smoothed
    adjusted[["tx_raw", "ty_raw", "tz_raw"]] = xyz
    adjusted[["timestamp", "t_rel", "tx_raw", "ty_raw", "tz_raw", "tx_adjusted", "ty_adjusted", "tz_adjusted"]].to_csv(
        out_dir / "indoor_forward_6_segment_adjusted.csv", index=False
    )

    raw_metrics = metrics(t, xyz)
    adjusted_metrics = metrics(t, smoothed)
    summary = {
        "source": str(source),
        "segment_window_s": [8.0, 20.0],
        "adjustment": "5-point centered moving average on tx/ty/tz, with linear endpoint correction",
        "raw": raw_metrics,
        "adjusted": adjusted_metrics,
        "relative_change_percent": {
            k: float((adjusted_metrics[k] - raw_metrics[k]) / raw_metrics[k] * 100.0)
            for k in raw_metrics
            if isinstance(raw_metrics[k], (float, int)) and raw_metrics[k] != 0 and k != "points"
        },
    }
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")

    speed_raw = np.linalg.norm(np.diff(xyz, axis=0) / np.diff(t)[:, None], axis=1)
    speed_adj = np.linalg.norm(np.diff(smoothed, axis=0) / np.diff(t)[:, None], axis=1)
    accel_raw = np.linalg.norm(np.diff(np.diff(xyz, axis=0) / np.diff(t)[:, None], axis=0) / np.diff(t)[1:, None], axis=1)
    accel_adj = np.linalg.norm(np.diff(np.diff(smoothed, axis=0) / np.diff(t)[:, None], axis=0) / np.diff(t)[1:, None], axis=1)

    fig = plt.figure(figsize=(14, 9))
    ax3d = fig.add_subplot(2, 2, 1, projection="3d")
    ax3d.plot(xyz[:, 0], xyz[:, 1], xyz[:, 2], label="raw", linewidth=1.8)
    ax3d.plot(smoothed[:, 0], smoothed[:, 1], smoothed[:, 2], label="adjusted", linewidth=1.8)
    ax3d.set_title("3D trajectory segment")
    ax3d.set_xlabel("tx [m]")
    ax3d.set_ylabel("ty [m]")
    ax3d.set_zlabel("tz [m]")
    ax3d.legend()

    ax_xy = fig.add_subplot(2, 2, 2)
    ax_xy.plot(xyz[:, 0], xyz[:, 1], label="raw", linewidth=1.8)
    ax_xy.plot(smoothed[:, 0], smoothed[:, 1], label="adjusted", linewidth=1.8)
    ax_xy.set_title("XY projection")
    ax_xy.set_xlabel("tx [m]")
    ax_xy.set_ylabel("ty [m]")
    ax_xy.axis("equal")
    ax_xy.grid(True, alpha=0.3)
    ax_xy.legend()

    ax_speed = fig.add_subplot(2, 2, 3)
    ax_speed.plot(t[1:], speed_raw, label="raw", linewidth=1.6)
    ax_speed.plot(t[1:], speed_adj, label="adjusted", linewidth=1.6)
    ax_speed.set_title("Speed")
    ax_speed.set_xlabel("time [s]")
    ax_speed.set_ylabel("m/s")
    ax_speed.grid(True, alpha=0.3)
    ax_speed.legend()

    ax_acc = fig.add_subplot(2, 2, 4)
    ax_acc.plot(t[2:], accel_raw, label="raw", linewidth=1.6)
    ax_acc.plot(t[2:], accel_adj, label="adjusted", linewidth=1.6)
    ax_acc.set_title("Acceleration magnitude")
    ax_acc.set_xlabel("time [s]")
    ax_acc.set_ylabel("m/s^2")
    ax_acc.grid(True, alpha=0.3)
    ax_acc.legend()

    fig.tight_layout()
    fig.savefig(out_dir / "trajectory_adjustment_comparison.png", dpi=180)

    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
