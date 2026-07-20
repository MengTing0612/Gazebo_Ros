#!/usr/bin/env python3
"""Fit Gaia 10-d action → RC residual adapter matrix W.

Implements Route B from the implementation doc:
  u_desired = [kx*y_err + kvx*vy, -kx*x_err - kvx*vx, kz*z_err - kvz*vz, kyaw*yaw_err]
  W = argmin ||A @ W^T - U||^2 + alpha * ||W||^2  (Ridge)

Saves: flight_bridge/gaia_adapter.yaml

Usage:
  python tools/fit_gaia_action_adapter.py logs/*.jsonl
  python tools/fit_gaia_action_adapter.py --alpha 0.1 --output gaia_adapter_test.yaml logs/*.jsonl
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path
from typing import Any

import numpy as np
import yaml

# ---------------------------------------------------------------------------
# Data loading (reuse from analyze script)
# ---------------------------------------------------------------------------


def load_and_filter_frames(paths: list[Path]) -> list[dict[str, Any]]:
    frames: list[dict[str, Any]] = []
    for p in paths:
        if not p.exists():
            print(f"WARNING: {p} not found, skipping")
            continue
        with open(p) as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    d = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if not d.get("sent"):
                    continue
                if d.get("timeout_discard"):
                    continue
                shield = d.get("shield", {})
                if shield.get("safety_triggered"):
                    continue
                if d.get("state_age_ms", 0) > 200:
                    continue
                if len(d.get("action", [])) < 10:
                    continue
                if not d.get("copter_state") or "x_err" not in d.get("copter_state", {}):
                    continue
                frames.append(d)
    return frames


def build_desired_correction(
    frames: list[dict[str, Any]],
    k_xy: float = 50.0,       # PWM/m  — 1m lateral → 50 PWM (~4.5°)
    k_vxy: float = 100.0,     # PWM/(m/s)
    k_z: float = 80.0,        # PWM/m  — 1m z_err → 80 PWM throttle
    k_vz: float = 50.0,       # PWM/(m/s)
    k_yaw: float = 200.0,     # PWM/rad — 0.1rad yaw → 20 PWM
    max_pwm: tuple[float, ...] = (30.0, 30.0, 50.0, 30.0),
) -> np.ndarray:
    """Compute desired RC residual PWM deltas from state errors.

    Gains are now physically meaningful (PWM per SI unit), derived from:
      roll/pitch:  max_angle=5° → 500/45*5 ≈ 55 PWM/deg → ~50 PWM/m
      throttle:    hover(450) ± throttle_range(550) → ~80 PWM/m
      yaw:         500 PWM / 45°/s → ~200 PWM/rad
    """
    N = len(frames)
    U = np.zeros((N, 4))
    for i, d in enumerate(frames):
        copter = d.get("copter_state", {})
        x_err = copter.get("x_err", 0)
        y_err = copter.get("y_err", 0)
        z_err = copter.get("z_err", 0)
        vx = copter.get("vx", 0)
        vy = copter.get("vy", 0)
        vz = copter.get("vz", 0)
        yaw_err = copter.get("yaw_err", 0)

        # PID-like desired correction
        roll_des = k_xy * y_err + k_vxy * vy    # roll right when drifting left
        pitch_des = -k_xy * x_err - k_vxy * vx   # pitch forward when too far back
        throttle_des = k_z * z_err - k_vz * vz   # vz>0 (climbing) → reduce throttle
        yaw_des = k_yaw * yaw_err                 # yaw toward target

        # Clamp to safe residual range
        roll_des = np.clip(roll_des, -max_pwm[0], max_pwm[0])
        pitch_des = np.clip(pitch_des, -max_pwm[1], max_pwm[1])
        throttle_des = np.clip(throttle_des, -max_pwm[2], max_pwm[2])
        yaw_des = np.clip(yaw_des, -max_pwm[3], max_pwm[3])

        U[i] = [roll_des, pitch_des, throttle_des, yaw_des]
    return U


def fit_W(
    A: np.ndarray,    # [N, 10] action matrix
    U: np.ndarray,    # [N, 4] desired residual
    alpha: float = 0.1,
    row_limits: tuple[float, ...] = (30.0, 30.0, 50.0, 30.0),
    row_norm_limits: tuple[float, ...] = (30.0, 30.0, 50.0, 30.0),
    residual_lambda: float = 0.05,
) -> tuple[np.ndarray, dict[str, float]]:
    """Fit W via Ridge, then clip and normalize rows.

    Returns (W [4,10], fit_stats).
    """
    from sklearn.linear_model import Ridge

    models = []
    W = np.zeros((4, 10))
    scores = {}
    for row in range(4):
        y = U[:, row]
        ridge = Ridge(alpha=alpha, fit_intercept=False)
        ridge.fit(A, y)
        W[row] = ridge.coef_
        scores[f"r2_row_{row}"] = float(ridge.score(A, y))
        models.append(ridge)

    # Clip individual entries
    for row in range(4):
        W[row] = np.clip(W[row], -row_limits[row], row_limits[row])

    # Row-level L1 normalization
    for row in range(4):
        row_sum = np.sum(np.abs(W[row]))
        if row_sum > row_norm_limits[row] and row_sum > 0:
            W[row] *= row_norm_limits[row] / row_sum

    return W, scores


def save_adapter(
    W: np.ndarray,
    residual_lambda: float,
    max_delta_pwm: dict[str, float],
    output_path: Path,
) -> None:
    config = {
        "adapter": {
            "type": "learned_linear",
            "action_dim": 10,
            "output": "rc_delta_pwm",
            "residual_lambda": residual_lambda,
            "max_delta_pwm": max_delta_pwm,
            "W": {
                "roll": [round(float(v), 4) for v in W[0]],
                "pitch": [round(float(v), 4) for v in W[1]],
                "throttle": [round(float(v), 4) for v in W[2]],
                "yaw": [round(float(v), 4) for v in W[3]],
            },
        }
    }
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as f:
        yaml.safe_dump(config, f, default_flow_style=False, sort_keys=False)
    print(f"Adapter saved to {output_path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> int:
    parser = argparse.ArgumentParser(description="Fit Gaia 10→4 action adapter matrix W")
    parser.add_argument("log_files", nargs="+", type=Path)
    parser.add_argument("--output", type=Path, default=Path("flight_bridge/gaia_adapter.yaml"))
    parser.add_argument("--alpha", type=float, default=0.1, help="Ridge regularization")
    parser.add_argument("--residual-lambda", type=float, default=0.05, help="Overall residual scale")
    parser.add_argument("--k-xy", type=float, default=50.0, help="PWM/m lateral gain")
    parser.add_argument("--k-vxy", type=float, default=100.0, help="PWM/(m/s) lateral velocity gain")
    parser.add_argument("--k-z", type=float, default=80.0, help="PWM/m vertical gain")
    parser.add_argument("--k-vz", type=float, default=50.0, help="PWM/(m/s) vertical velocity gain")
    parser.add_argument("--k-yaw", type=float, default=200.0, help="PWM/rad yaw gain")
    parser.add_argument("--max-roll", type=float, default=30.0)
    parser.add_argument("--max-pitch", type=float, default=30.0)
    parser.add_argument("--max-throttle", type=float, default=50.0)
    parser.add_argument("--max-yaw", type=float, default=30.0)
    args = parser.parse_args()

    row_limits = (args.max_roll, args.max_pitch, args.max_throttle, args.max_yaw)

    print(f"Loading {len(args.log_files)} log file(s)...")
    frames = load_and_filter_frames(args.log_files)
    print(f"  {len(frames)} valid frames")

    if len(frames) < 50:
        print("ERROR: too few valid frames")
        return 1

    # Build matrices
    A = np.array([d["action"][:10] for d in frames])  # [N, 10]
    U = build_desired_correction(
        frames, k_xy=args.k_xy, k_vxy=args.k_vxy,
        k_z=args.k_z, k_vz=args.k_vz, k_yaw=args.k_yaw,
        max_pwm=row_limits,
    )

    # Fit
    print(f"\nFitting W: A [{A.shape}] → U [{U.shape}]")
    print(f"  Ridge alpha={args.alpha}, residual_lambda={args.residual_lambda}")
    W, scores = fit_W(
        A, U, alpha=args.alpha, row_limits=row_limits,
        row_norm_limits=row_limits,
        residual_lambda=args.residual_lambda,
    )

    for k, v in scores.items():
        print(f"  {k}: {v:.4f}")

    print(f"\nW matrix [4x10], residual_lambda={args.residual_lambda}:")
    row_names = ["roll  ", "pitch ", "throt ", "yaw   "]
    for row, name in enumerate(row_names):
        vals = " ".join(f"{W[row][j]:+.4f}" for j in range(10))
        print(f"  {name}: [{vals}]")

    max_pwm = {
        "roll": args.max_roll,
        "pitch": args.max_pitch,
        "throttle": args.max_throttle,
        "yaw": args.max_yaw,
    }
    save_adapter(W, args.residual_lambda, max_pwm, args.output)

    # Compute raw deltas
    deltas = A @ W.T
    print(f"\nRaw delta stats (before lambda={args.residual_lambda}):")
    for row, name in enumerate(row_names):
        d = deltas[:, row]
        print(f"  {name}: mean={np.mean(d):+.2f} std={np.std(d):.2f} "
              f"abs_max={np.max(np.abs(d)):.2f} PWM")

    # Show effective after lambda
    eff_deltas = deltas * args.residual_lambda
    print(f"\nEffective delta stats (after lambda={args.residual_lambda}):")
    for row, name in enumerate(row_names):
        d = eff_deltas[:, row]
        print(f"  {name}: mean={np.mean(d):+.2f} std={np.std(d):.2f} "
              f"abs_max={np.max(np.abs(d)):.2f} PWM")

    return 0


if __name__ == "__main__":
    sys.exit(main())
