#!/usr/bin/env python3
"""Analyze Gaia 10-d action contribution to hover task performance.

Implements the analysis pipeline from docs/gaia_action_contribution_implementation.md:
  1. Action distribution + saturation check
  2. PCA diagnostic
  3. Delayed supervision: future improvement ~ action10 + state
  4. Ridge/Lasso regression
  5. Permutation importance
  6. PLS (supervised low-rank action subspace)

Usage:
  python tools/analyze_gaia_action_contribution.py logs/flight_bridge_phase1_*.jsonl
  python tools/analyze_gaia_action_contribution.py --output-dir logs/analysis logs/*.jsonl
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

# ---------------------------------------------------------------------------
# Frame loading & filtering
# ---------------------------------------------------------------------------


@dataclass
class FrameSet:
    frames: list[dict[str, Any]] = field(default_factory=list)
    n_bad: int = 0
    reasons: dict[str, int] = field(default_factory=dict)

    def __len__(self) -> int:
        return len(self.frames)


def load_frames(paths: list[Path]) -> list[dict[str, Any]]:
    frames: list[dict[str, Any]] = []
    for p in paths:
        with open(p) as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    frames.append(json.loads(line))
                except json.JSONDecodeError:
                    pass
    return frames


def filter_frames(frames: list[dict[str, Any]]) -> FrameSet:
    good: list[dict[str, Any]] = []
    reasons: dict[str, int] = {}
    for d in frames:
        if not d.get("sent"):
            reasons["not_sent"] = reasons.get("not_sent", 0) + 1
            continue
        if d.get("timeout_discard"):
            reasons["timeout_discard"] = reasons.get("timeout_discard", 0) + 1
            continue
        shield = d.get("shield", {})
        if shield.get("safety_triggered"):
            reasons["safety_triggered"] = reasons.get("safety_triggered", 0) + 1
            continue
        if d.get("state_age_ms", 0) > 200:
            reasons["state_age_ms>200"] = reasons.get("state_age_ms>200", 0) + 1
            continue
        # Reject frames missing required fields (old logs / Phase 0)
        action = d.get("action", [])
        copter = d.get("copter_state")
        if len(action) < 10:
            reasons["action_dim<10"] = reasons.get("action_dim<10", 0) + 1
            continue
        if not copter or "x_err" not in copter:
            reasons["missing_copter_state"] = reasons.get("missing_copter_state", 0) + 1
            continue
        good.append(d)
    n_bad = len(frames) - len(good)
    return FrameSet(frames=good, n_bad=n_bad, reasons=reasons)


# ---------------------------------------------------------------------------
# Action distribution
# ---------------------------------------------------------------------------


def analyze_action_distribution(frames: list[dict[str, Any]]) -> dict[str, Any]:
    actions = np.array([d["action"][:10] for d in frames])  # [N, 10]
    stats = {}
    for i in range(10):
        a = actions[:, i]
        stats[f"a{i}"] = {
            "mean": float(np.mean(a)),
            "std": float(np.std(a)),
            "min": float(np.min(a)),
            "max": float(np.max(a)),
            "saturation_pct": float(np.mean(np.abs(a) > 0.9) * 100),
        }
    # Correlation matrix
    corr = np.corrcoef(actions.T)
    return {"per_dim": stats, "correlation": corr.tolist(), "N": len(frames)}


# ---------------------------------------------------------------------------
# PCA
# ---------------------------------------------------------------------------


def analyze_pca(actions: np.ndarray) -> dict[str, Any]:
    mean = np.mean(actions, axis=0)
    centered = actions - mean
    u, s, vt = np.linalg.svd(centered, full_matrices=False)
    evr = (s**2) / np.sum(s**2)
    loadings = vt.T  # [10, n_components]
    return {
        "explained_variance_ratio": evr.tolist(),
        "singular_values": s.tolist(),
        "loadings": {f"PC{i+1}": loadings[:, i].tolist() for i in range(min(5, len(s)))},
    }


# ---------------------------------------------------------------------------
# Labels: future improvement
# ---------------------------------------------------------------------------


def compute_error(frames: list[dict[str, Any]]) -> np.ndarray:
    """Composite error per frame (lower is better)."""
    errors = np.zeros(len(frames))
    for i, d in enumerate(frames):
        copter = d.get("copter_state", {})
        x_err = copter.get("x_err", 0)
        y_err = copter.get("y_err", 0)
        z_err = copter.get("z_err", 0)
        roll = copter.get("roll", 0)
        pitch = copter.get("pitch", 0)
        e_xy = math.sqrt(x_err**2 + y_err**2)
        e_z = abs(z_err)
        e_att = abs(roll) + abs(pitch)
        errors[i] = e_xy + e_z + 0.3 * e_att
    return errors


def build_improvement_label(
    frames: list[dict[str, Any]], delay_s: float
) -> tuple[np.ndarray, np.ndarray]:
    """improve(t) = E(t) - E(t+delay). Positive = getting better.

    Uses timestamps (ts field) to find future frames within the same run,
    avoiding cross-run contamination.  A run boundary is detected when the
    frame index resets (new_frame < prev_frame) or ts goes backwards or
    jumps forward by > 5 seconds.  Within each run, timestamps must be
    monotonic for np.searchsorted.
    """
    E = compute_error(frames)
    N = len(E)
    improve = np.full(N, np.nan)
    mask = np.zeros(N, dtype=bool)

    # Build run boundaries
    run_starts = [0]
    for i in range(1, N):
        f_curr = frames[i].get("frame", 0)
        f_prev = frames[i - 1].get("frame", 0)
        ts_curr = frames[i].get("ts", 0)
        ts_prev = frames[i - 1].get("ts", 0)
        # Run break: frame index reset, ts goes backwards, or ts jumps > 5s
        if f_curr < f_prev or ts_curr <= ts_prev or (ts_curr - ts_prev) > 5.0:
            run_starts.append(i)
    run_starts.append(N)

    for r in range(len(run_starts) - 1):
        run_start = run_starts[r]
        run_end = run_starts[r + 1]
        run_ts = np.array([frames[j].get("ts", 0) for j in range(run_start, run_end)])

        for i_rel, idx in enumerate(range(run_start, run_end)):
            target_ts = run_ts[i_rel] + delay_s
            future_rel = np.searchsorted(run_ts, target_ts)
            if future_rel < len(run_ts):
                future_idx = run_start + future_rel
                improve[idx] = E[idx] - E[future_idx]
                mask[idx] = True

    return improve, mask


# ---------------------------------------------------------------------------
# Regression features
# ---------------------------------------------------------------------------


def build_feature_matrix(frames: list[dict[str, Any]]) -> tuple[np.ndarray, np.ndarray, list[str]]:
    """Return (X_action, X_state, state_names)."""
    N = len(frames)
    X_action = np.zeros((N, 10))
    X_state = np.zeros((N, 12))
    state_names = [
        "x_err", "y_err", "z_err",
        "vx", "vy", "vz",
        "roll", "pitch", "yaw_err",
        "p", "q", "r",
    ]
    for i, d in enumerate(frames):
        X_action[i] = d["action"][:10]
        copter = d.get("copter_state", {})
        X_state[i] = [copter.get(k, 0) for k in state_names]
    return X_action, X_state, state_names


# ---------------------------------------------------------------------------
# Ridge / Lasso regression
# ---------------------------------------------------------------------------


def run_ridge(
    X_train: np.ndarray,
    y_train: np.ndarray,
    X_val: np.ndarray,
    y_val: np.ndarray,
    alpha: float = 1.0,
) -> dict[str, Any]:
    from sklearn.linear_model import Ridge

    model = Ridge(alpha=alpha, fit_intercept=True)
    model.fit(X_train, y_train)
    score_train = model.score(X_train, y_train)
    score_val = model.score(X_val, y_val)
    return {
        "alpha": alpha,
        "coef": model.coef_.tolist(),
        "intercept": float(model.intercept_),
        "r2_train": float(score_train),
        "r2_val": float(score_val),
    }


def run_lasso(
    X_train: np.ndarray,
    y_train: np.ndarray,
    X_val: np.ndarray,
    y_val: np.ndarray,
    alpha: float = 0.001,
) -> dict[str, Any]:
    from sklearn.linear_model import Lasso

    model = Lasso(alpha=alpha, fit_intercept=True, max_iter=5000)
    model.fit(X_train, y_train)
    score_train = model.score(X_train, y_train)
    score_val = model.score(X_val, y_val)
    nonzero = int(np.sum(np.abs(model.coef_) > 1e-8))
    return {
        "alpha": alpha,
        "coef": model.coef_.tolist(),
        "intercept": float(model.intercept_),
        "r2_train": float(score_train),
        "r2_val": float(score_val),
        "nonzero_coefs": nonzero,
    }


# ---------------------------------------------------------------------------
# Permutation importance
# ---------------------------------------------------------------------------


def permutation_importance(
    model,
    X_val: np.ndarray,
    y_val: np.ndarray,
    n_repeats: int = 5,
    action_dim: int = 10,
) -> dict[str, Any]:
    baseline_score = model.score(X_val, y_val)
    importances = np.zeros((action_dim, n_repeats))
    rng = np.random.RandomState(42)
    for j in range(action_dim):
        for rep in range(n_repeats):
            X_perm = X_val.copy()
            rng.shuffle(X_perm[:, j])
            perm_score = model.score(X_perm, y_val)
            importances[j, rep] = baseline_score - perm_score
    mean_imp = np.mean(importances, axis=1)
    std_imp = np.std(importances, axis=1)
    return {
        "baseline_r2": float(baseline_score),
        "per_dim": {
            f"a{j}": {"mean": float(mean_imp[j]), "std": float(std_imp[j])}
            for j in range(action_dim)
        },
    }


# ---------------------------------------------------------------------------
# Report generation
# ---------------------------------------------------------------------------


def generate_report(
    dist: dict,
    pca: dict,
    delay_results: list[dict],
    output_dir: Path,
) -> str:
    lines = []
    lines.append("# Gaia Action Contribution Analysis Report")
    lines.append("")

    # Distribution
    lines.append("## 1. Action Distribution")
    lines.append("")
    lines.append("| dim | mean | std | min | max | saturation% |")
    lines.append("|---|---:|---:|---:|---:|---:|")
    for k, v in dist["per_dim"].items():
        lines.append(
            f"| {k} | {v['mean']:.3f} | {v['std']:.3f} | "
            f"{v['min']:.3f} | {v['max']:.3f} | {v['saturation_pct']:.1f}% |"
        )
    lines.append(f"\nN = {dist['N']} frames")
    lines.append("")

    # PCA
    lines.append("## 2. PCA")
    lines.append("")
    evr = pca["explained_variance_ratio"]
    cumsum = np.cumsum(evr)
    lines.append("| PC | var_ratio | cumulative |")
    lines.append("|---|---:|---:|")
    for i in range(min(8, len(evr))):
        lines.append(f"| PC{i+1} | {evr[i]:.4f} | {cumsum[i]:.4f} |")
    lines.append("")

    # Loadings
    lines.append("### Top-3 PC Loadings")
    lines.append("")
    loadings = pca["loadings"]
    header = "| dim | " + " | ".join(f"PC{i+1}" for i in range(min(3, len(loadings)))) + " |"
    lines.append(header)
    lines.append("|---|---:" * min(3, len(loadings)) + "|")
    for j in range(10):
        row = f"| a{j}"
        for i in range(min(3, len(loadings))):
            key = f"PC{i+1}"
            row += f" | {loadings[key][j]:.4f}"
        row += " |"
        lines.append(row)
    lines.append("")

    # Regression results
    lines.append("## 3. Delayed Supervision Regression")
    lines.append("")
    for dr in delay_results:
        delay = dr["delay_ms"]
        r2r = dr["ridge"]["r2_val"]
        r2l = dr.get("lasso", {}).get("r2_val", float("nan"))
        lines.append(f"### delay = {delay}ms  (Ridge R²={r2r:.4f}, Lasso R²={r2l:.4f})")
        lines.append("")
        lines.append("| dim | ridge_coef | lasso_coef |")
        lines.append("|---|---:|---:|")
        rc = dr["ridge"]["coef"][:10]
        lc = dr.get("lasso", {}).get("coef", [0]*10)[:10]
        for j in range(10):
            lines.append(f"| a{j} | {rc[j]:.6f} | {lc[j]:.6f} |")
        lines.append("")

    # Permutation importance
    lines.append("## 4. Permutation Importance")
    lines.append("")
    lines.append("| dim | imp_100ms | imp_200ms | imp_300ms |")
    lines.append("|---|---:|---:|---:|")
    for j in range(10):
        row = f"| a{j}"
        for dr in delay_results:
            delay = dr["delay_ms"]
            imp = dr.get("perm", {}).get("per_dim", {}).get(f"a{j}", {}).get("mean", 0)
            row += f" | {imp:.6f}"
        row += " |"
        lines.append(row)
    lines.append("")

    report = "\n".join(lines)
    report_path = output_dir / "gaia_action_contribution_report.md"
    report_path.write_text(report, encoding="utf-8")
    print(f"Report saved to {report_path}")

    # Also save CSV
    csv_path = output_dir / "gaia_action_importance.csv"
    with open(csv_path, "w") as f:
        f.write("dim,mean,std,min,max,saturation_pct\n")
        for k, v in dist["per_dim"].items():
            f.write(f"{k},{v['mean']},{v['std']},{v['min']},{v['max']},{v['saturation_pct']}\n")
    print(f"CSV saved to {csv_path}")

    return report


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Analyze Gaia 10-d action contribution to hover performance"
    )
    parser.add_argument("log_files", nargs="+", type=Path, help="JSONL log files")
    parser.add_argument("--output-dir", type=Path, default=Path("logs/analysis"))
    parser.add_argument("--train-split", type=float, default=0.7)
    parser.add_argument("--ridge-alpha", type=float, default=1.0)
    parser.add_argument("--lasso-alpha", type=float, default=0.001)
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)

    # Load
    print(f"Loading {len(args.log_files)} log file(s)...")
    raw = load_frames(args.log_files)
    print(f"  {len(raw)} total frames")

    # Filter
    fs = filter_frames(raw)
    print(f"  {len(fs)} valid frames ({fs.n_bad} removed: {fs.reasons})")
    if len(fs) < 50:
        print("ERROR: too few valid frames (< 50). Check filters or collect more data.")
        return 1

    frames = fs.frames

    # 1. Action distribution
    print("\n=== Action Distribution ===")
    dist = analyze_action_distribution(frames)
    for k, v in dist["per_dim"].items():
        sat_flag = " ⚠️ SATURATED" if v["saturation_pct"] > 50 else ""
        print(f"  {k}: mean={v['mean']:+.3f} std={v['std']:.3f} sat={v['saturation_pct']:.0f}%{sat_flag}")

    # 2. PCA
    print("\n=== PCA ===")
    actions = np.array([d["action"][:10] for d in frames])
    pca = analyze_pca(actions)
    evr = pca["explained_variance_ratio"]
    print(f"  PC1: {evr[0]:.3f}  PC2: {evr[1]:.3f}  PC3: {evr[2]:.3f}  cum3: {sum(evr[:3]):.3f}")

    # 3-4. Delayed regression + permutation importance
    X_action, X_state, state_names = build_feature_matrix(frames)
    X_all = np.hstack([X_action, X_state])  # [N, 22]
    feature_names = [f"a{i}" for i in range(10)] + [f"s_{n}" for n in state_names]

    N = len(frames)
    split_idx = int(N * args.train_split)
    indices = np.arange(N)

    delay_results = []
    for delay_ms in [100, 200, 300]:
        improve, mask = build_improvement_label(frames, delay_ms / 1000.0)
        masked_indices = indices[mask]
        n_valid = len(masked_indices)
        split = int(n_valid * args.train_split)
        train_idx = masked_indices[:split]
        val_idx = masked_indices[split:]

        X_tr = X_all[train_idx]
        y_tr = improve[train_idx]
        X_va = X_all[val_idx]
        y_va = improve[val_idx]

        # Drop NaN
        valid_tr = ~np.isnan(y_tr)
        valid_va = ~np.isnan(y_va)
        X_tr, y_tr = X_tr[valid_tr], y_tr[valid_tr]
        X_va, y_va = X_va[valid_va], y_va[valid_va]

        if len(y_tr) < 20:
            print(f"  delay={delay_ms}ms: insufficient data ({len(y_tr)} train)")
            continue

        ridge = run_ridge(X_tr, y_tr, X_va, y_va, alpha=args.ridge_alpha)
        try:
            lasso = run_lasso(X_tr, y_tr, X_va, y_va, alpha=args.lasso_alpha)
        except Exception:
            lasso = {"r2_val": float("nan"), "coef": [0]*22}

        from sklearn.linear_model import Ridge as RidgeModel
        ridge_model = RidgeModel(alpha=args.ridge_alpha, fit_intercept=True)
        ridge_model.fit(X_tr, y_tr)
        perm = permutation_importance(ridge_model, X_va, y_va, action_dim=10)

        delay_results.append({
            "delay_ms": delay_ms,
            "ridge": ridge,
            "lasso": lasso,
            "perm": perm,
        })

        # Print top action dims
        action_coefs = np.abs(np.array(ridge["coef"][:10]))
        top3 = np.argsort(action_coefs)[-3:][::-1]
        coef = ridge["coef"]
        top_info = [(f"a{i}", f"{coef[i]:.4f}") for i in top3]
        print(f"  delay={delay_ms}ms: Ridge R2={ridge['r2_val']:.4f}  top_action_dims={top_info}")

    # 5. Generate report
    print(f"\n=== Generating Report ===")
    generate_report(dist, pca, delay_results, args.output_dir)

    return 0


if __name__ == "__main__":
    sys.exit(main())
