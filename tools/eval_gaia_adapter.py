#!/usr/bin/env python3
"""A/B evaluation of Gaia action adapter variants.

Three groups:
  A: baseline only (no SDK residual)
  B: front-4 hard mapping
  C: learned W mapping

Usage:
  python tools/eval_gaia_adapter.py --group A logs/group_a/*.jsonl
  python tools/eval_gaia_adapter.py --group B logs/group_b/*.jsonl
  python tools/eval_gaia_adapter.py --compare A=logs/a/ B=logs/b/ C=logs/c/
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


@dataclass
class EvalMetrics:
    name: str = ""
    n_frames: int = 0
    n_valid: int = 0
    n_runs: int = 1
    success_rate: float = 1.0
    distance_mean: float = 0.0
    distance_p50: float = 0.0
    distance_p95: float = 0.0
    horizontal_p95: float = 0.0
    vertical_p95: float = 0.0
    tilt_p95: float = 0.0
    action_saturation_pct: float = 0.0
    rc_clip_count: int = 0
    fallback_count: int = 0
    crash_count: int = 0


def compute_run_metrics(frames: list[dict[str, Any]], name: str = "") -> EvalMetrics:
    m = EvalMetrics(name=name, n_frames=len(frames))

    valid = []
    crash = 0
    fallback = 0
    clip_total = 0
    sat_ratio = 0.0

    for d in frames:
        if not d.get("sent"):
            fallback += 1
            continue
        if d.get("timeout_discard"):
            fallback += 1
            continue
        shield = d.get("shield", {})
        if shield.get("safety_triggered"):
            fallback += 1
            continue
        clip_total += len(shield.get("clipped_fields", []))
        copter = d.get("copter_state", {})
        if copter:
            tilt = max(abs(copter.get("roll", 0)), abs(copter.get("pitch", 0)))
            if tilt > math.radians(25):
                crash += 1
                continue
        action = d.get("action", [])
        if action:
            sat_ratio += np.mean(np.abs(np.array(action[:10])) > 0.9)
        valid.append(d)

    m.n_valid = len(valid)
    m.fallback_count = fallback
    m.crash_count = crash
    m.rc_clip_count = clip_total
    m.success_rate = 1.0 - crash / max(1, m.n_frames)

    if valid:
        distances = []
        horizontals = []
        verticals = []
        tilts = []
        for d in valid:
            copter = d.get("copter_state", {})
            x = copter.get("x_err", 0)
            y = copter.get("y_err", 0)
            z = copter.get("z_err", 0)
            distances.append(math.sqrt(x**2 + y**2 + z**2))
            horizontals.append(math.sqrt(x**2 + y**2))
            verticals.append(abs(z))
            tilts.append(max(abs(copter.get("roll", 0)), abs(copter.get("pitch", 0))))

        d_arr = np.array(distances)
        h_arr = np.array(horizontals)
        v_arr = np.array(verticals)
        t_arr = np.array(tilts)
        m.distance_mean = float(np.mean(d_arr))
        m.distance_p50 = float(np.percentile(d_arr, 50))
        m.distance_p95 = float(np.percentile(d_arr, 95))
        m.horizontal_p95 = float(np.percentile(h_arr, 95))
        m.vertical_p95 = float(np.percentile(v_arr, 95))
        m.tilt_p95 = float(np.degrees(np.percentile(t_arr, 95)))
        m.action_saturation_pct = float(sat_ratio / len(valid) * 100)

    return m


def load_runs(paths: list[Path]) -> list[list[dict[str, Any]]]:
    """Load frames, split into runs by frame index reset."""
    all_frames: list[dict[str, Any]] = []
    for p in paths:
        if not p.exists():
            continue
        with open(p) as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    all_frames.append(json.loads(line))
                except json.JSONDecodeError:
                    pass

    # Split into runs: when frame index resets to < prev
    runs = []
    current_run = []
    prev_frame = -1
    for d in all_frames:
        f = d.get("frame", 0)
        if f < prev_frame:
            if current_run:
                runs.append(current_run)
            current_run = []
        current_run.append(d)
        prev_frame = f
    if current_run:
        runs.append(current_run)
    return runs if runs else [all_frames]


def generate_report(
    groups: dict[str, EvalMetrics],
    output_dir: Path,
) -> str:
    lines = []
    lines.append("# Gaia Adapter A/B Evaluation Report")
    lines.append("")
    lines.append("## Metrics")
    lines.append("")
    header = "| metric | " + " | ".join(groups.keys()) + " |"
    lines.append(header)
    lines.append("|---|---" + "|---" * len(groups) + "|")

    rows = [
        ("success_rate", "Success Rate"),
        ("distance_mean", "Distance Mean (m)"),
        ("distance_p95", "Distance P95 (m)"),
        ("horizontal_p95", "Horizontal P95 (m)"),
        ("vertical_p95", "Vertical P95 (m)"),
        ("tilt_p95", "Tilt P95 (deg)"),
        ("action_saturation_pct", "Action Saturation %"),
        ("rc_clip_count", "RC Clip Count"),
        ("fallback_count", "Fallback Count"),
        ("crash_count", "Crash Count"),
    ]
    for attr, label in rows:
        vals = " | ".join(f"{getattr(groups[g], attr):.4g}" if isinstance(getattr(groups[g], attr), float)
                          else str(getattr(groups[g], attr))
                          for g in groups)
        lines.append(f"| {label} | {vals} |")

    lines.append("")
    report = "\n".join(lines)
    output_dir.mkdir(parents=True, exist_ok=True)
    report_path = output_dir / "gaia_adapter_ab_report.md"
    report_path.write_text(report, encoding="utf-8")
    print(f"Report saved to {report_path}")

    csv_path = output_dir / "gaia_adapter_ab_metrics.csv"
    with open(csv_path, "w") as f:
        f.write("group," + ",".join(attr for attr, _ in rows) + "\n")
        for g_name in groups:
            m = groups[g_name]
            f.write(g_name + "," + ",".join(
                str(getattr(m, attr)) for attr, _ in rows
            ) + "\n")
    print(f"CSV saved to {csv_path}")

    return report


def main() -> int:
    parser = argparse.ArgumentParser(description="A/B evaluate Gaia adapter variants")
    parser.add_argument("--group", type=str, help="Single group name")
    parser.add_argument("log_paths", nargs="*", type=Path, help="Log files for --group")
    parser.add_argument("--compare", type=str, help="key=path,key=path,...")
    parser.add_argument("--output-dir", type=Path, default=Path("logs/analysis"))
    args = parser.parse_args()

    groups: dict[str, list[Path]] = {}

    if args.group and args.log_paths:
        groups[args.group] = args.log_paths

    if args.compare:
        for part in args.compare.split(","):
            key, path = part.strip().split("=", 1)
            groups[key] = list(Path(path).glob("*.jsonl"))

    if not groups:
        parser.print_help()
        return 1

    results: dict[str, EvalMetrics] = {}
    for g_name, g_paths in groups.items():
        print(f"\n=== {g_name} ===")
        runs = load_runs(g_paths)
        print(f"  {len(runs)} run(s), {sum(len(r) for r in runs)} total frames")

        all_frames = [d for run in runs for d in run]
        m = compute_run_metrics(all_frames, name=g_name)
        m.n_runs = len(runs)
        results[g_name] = m

        print(f"  valid: {m.n_valid}/{m.n_frames}  success_rate: {m.success_rate:.2%}")
        print(f"  distance: mean={m.distance_mean:.2f}m  p95={m.distance_p95:.2f}m")
        print(f"  horizontal_p95: {m.horizontal_p95:.2f}m  vertical_p95: {m.vertical_p95:.2f}m")
        print(f"  tilt_p95: {m.tilt_p95:.1f}deg  saturation: {m.action_saturation_pct:.1f}%")
        print(f"  rc_clips: {m.rc_clip_count}  fallback: {m.fallback_count}  crash: {m.crash_count}")

    if len(results) > 1:
        generate_report(results, args.output_dir)

    return 0


if __name__ == "__main__":
    sys.exit(main())
