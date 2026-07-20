#!/usr/bin/env python3
"""Real-time feedback visualization for flight_bridge.

Reads the JSONL log file as it's written and plots feedback (distance to
target) and key state variables in real time.

Usage:
    # In one terminal: run flight_bridge
    python -m flight_bridge.main --phase 1 --mode sitl --control rc ...

    # In another terminal: start the visualizer
    python tools/visualize_feedback.py logs/flight_bridge_phase1_*.jsonl

    # Or tail the latest log automatically
    python tools/visualize_feedback.py --watch logs/

Controls:
    q / Ctrl+C  — quit
    r           — reset view
    p           — pause / resume
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from collections import deque
from pathlib import Path
from typing import Any

import numpy as np

# ---------------------------------------------------------------------------
# Data reader — tails JSONL file
# ---------------------------------------------------------------------------


class LogTail:
    """Reads JSONL frames from a growing log file."""

    def __init__(self, path: Path) -> None:
        self._path = path
        self._pos = 0

    def poll(self) -> list[dict[str, Any]]:
        frames: list[dict[str, Any]] = []
        try:
            with open(self._path) as f:
                f.seek(self._pos)
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        frames.append(json.loads(line))
                    except json.JSONDecodeError:
                        pass
                self._pos = f.tell()
        except FileNotFoundError:
            pass
        return frames


def find_latest_log(log_dir: Path) -> Path | None:
    """Find the most recently modified JSONL file."""
    files = sorted(log_dir.glob("flight_bridge*.jsonl"), key=lambda p: p.stat().st_mtime, reverse=True)
    return files[0] if files else None


# ---------------------------------------------------------------------------
# Live plot using matplotlib
# ---------------------------------------------------------------------------


class FeedbackVisualizer:
    """Real-time matplotlib plot of feedback and state."""

    def __init__(self, max_history: int = 500) -> None:
        self._max = max_history
        self._frames: deque[int] = deque(maxlen=max_history)
        self._distances: deque[float] = deque(maxlen=max_history)
        self._z_errs: deque[float] = deque(maxlen=max_history)
        self._throttles: deque[float] = deque(maxlen=max_history)
        self._tilt_deg: deque[float] = deque(maxlen=max_history)
        self._climb: deque[float] = deque(maxlen=max_history)
        self._action_sat: deque[float] = deque(maxlen=max_history)
        self._paused = False

        import matplotlib
        matplotlib.use("Qt5Agg")
        import matplotlib.pyplot as plt

        self._fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
        self._ax_dist = axes[0]
        self._ax_pos = axes[1]
        self._ax_act = axes[2]

        self._fig.canvas.manager.set_window_title("Feedback Monitor")
        self._fig.tight_layout(pad=3.0)

        # Distance line — real 3D Euclidean distance from copter_state (m)
        (self._line_dist,) = self._ax_dist.plot([], [], "b-", linewidth=2, label="3D distance (m)")
        self._ax_dist.axhline(y=0, color="green", linestyle="--", alpha=0.5, label="target")
        self._ax_dist.set_ylabel("3D distance (m)")
        self._ax_dist.legend(loc="upper left")
        self._ax_dist.set_ylim(0, 5)
        self._ax_dist.grid(True, alpha=0.3)

        # Position errors — z_err (altitude) + tilt
        (self._line_z,) = self._ax_pos.plot([], [], "r-", linewidth=1.5, label="z_err (m)")
        self._ax_pos.axhline(y=0, color="red", linestyle="--", alpha=0.3)
        self._ax_pos_tilt = self._ax_pos.twinx()
        self._line_tilt_p, = self._ax_pos_tilt.plot([], [], "orange", linewidth=1.5, label="tilt (°)", alpha=0.7)
        self._ax_pos.set_ylabel("z_err (m)")
        self._ax_pos_tilt.set_ylabel("tilt (°)")
        self._ax_pos.grid(True, alpha=0.3)
        lines_pos = [self._line_z, self._line_tilt_p]
        labels_pos = ["z_err (m)", "tilt (°)"]
        self._ax_pos.legend(lines_pos, labels_pos, loc="upper left")

        # Action saturation + throttle
        (self._line_thr,) = self._ax_act.plot([], [], "g-", linewidth=1.5, label="throttle PWM")
        self._ax_act_rhs = self._ax_act.twinx()
        (self._line_sat,) = self._ax_act_rhs.plot([], [], "purple", linewidth=1, label="action sat%", alpha=0.6)
        self._ax_act.set_xlabel("Frame")
        self._ax_act.set_ylabel("Throttle (PWM)")
        self._ax_act_rhs.set_ylabel("Saturation %")
        self._ax_act_rhs.set_ylim(0, 105)
        self._ax_act.grid(True, alpha=0.3)
        lines_act = [self._line_thr, self._line_sat]
        labels_act = ["throttle PWM", "saturation %"]
        self._ax_act.legend(lines_act, labels_act, loc="upper left")

        self._fig.canvas.mpl_connect("key_press_event", self._on_key)
        plt.ion()
        plt.show(block=False)

    def _on_key(self, event) -> None:
        if event.key == "q":
            plt.close("all")
            sys.exit(0)
        elif event.key == "r":
            self._ax_dist.set_ylim(0, 5)
            self._ax_dist.set_xlim(0, max(50, len(self._frames)))
            self._fig.canvas.draw()
        elif event.key == "p":
            self._paused = not self._paused
            print(f"[visualizer] {'paused' if self._paused else 'resumed'}")

    def push(self, frame: dict[str, Any]) -> None:
        if self._paused:
            return

        f_id = frame.get("frame", len(self._frames))
        self._frames.append(f_id)

        # --- real 3D distance from copter_state (meters) ---
        copter = frame.get("copter_state", {})
        x_err = copter.get("x_err", 0)
        y_err = copter.get("y_err", 0)
        z_err = copter.get("z_err", 0)
        real_dist = math.sqrt(x_err**2 + y_err**2 + z_err**2)
        self._distances.append(real_dist)
        self._z_errs.append(z_err)

        # tilt
        roll = abs(copter.get("roll", 0))
        pitch = abs(copter.get("pitch", 0))
        self._tilt_deg.append(max(math.degrees(roll), math.degrees(pitch)))

        # Throttle
        cmd = frame.get("safe_command") or frame.get("candidate_command") or []
        self._throttles.append(cmd[2] if len(cmd) > 2 else 0)

        # Climb
        state = frame.get("state", {})
        self._climb.append(state.get("climb", 0))

        # Action saturation
        action = frame.get("action", [])
        if action:
            sat = np.mean(np.abs(np.array(action[:10])) > 0.9) * 100
        else:
            sat = 0
        self._action_sat.append(sat)

    def redraw(self) -> None:
        if self._paused:
            return

        xs = list(self._frames)
        if not xs:
            return

        self._line_dist.set_data(xs, list(self._distances))

        self._line_z.set_data(xs, list(self._z_errs))
        self._line_tilt_p.set_data(xs, list(self._tilt_deg))

        self._line_thr.set_data(xs, list(self._throttles))
        self._line_sat.set_data(xs, list(self._action_sat))

        # Auto-scale x
        x_min = max(0, xs[-1] - self._max)
        x_max = xs[-1] + 10
        self._ax_dist.set_xlim(x_min, x_max)

        # Auto-scale y for distance
        dists = list(self._distances)[-100:]
        if dists:
            y_max = max(0.5, max(dists) * 1.2)
            self._ax_dist.set_ylim(0, y_max)

        self._fig.canvas.draw_idle()
        self._fig.canvas.flush_events()


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> int:
    parser = argparse.ArgumentParser(description="Real-time feedback visualization")
    parser.add_argument("log", nargs="?", type=Path, help="JSONL log file to watch")
    parser.add_argument("--watch", type=Path, help="Watch directory for latest log")
    parser.add_argument("--history", type=int, default=500, help="Max frames to display")
    args = parser.parse_args()

    if args.watch:
        log_path = find_latest_log(args.watch)
        if not log_path:
            print(f"No log files found in {args.watch}")
            return 1
        print(f"Watching {log_path}")
        tail = LogTail(log_path)
        # Re-check for new files periodically
        check_interval = time.time()
    elif args.log:
        log_path = args.log
        print(f"Watching {log_path}")
        tail = LogTail(log_path)
        check_interval = None
    else:
        # Try logs/ directory
        log_path = find_latest_log(Path("logs"))
        if not log_path:
            print("No log files found. Usage: python tools/visualize_feedback.py <log.jsonl>")
            return 1
        print(f"Auto-detected: {log_path}")
        tail = LogTail(log_path)
        check_interval = time.time()

    viz = FeedbackVisualizer(max_history=args.history)
    last_draw = 0.0
    draw_interval = 0.1  # 10Hz refresh

    try:
        while True:
            frames = tail.poll()
            for f in frames:
                viz.push(f)

            now = time.time()
            if now - last_draw > draw_interval:
                viz.redraw()
                last_draw = now

            # Check for new log file in watch mode
            if args.watch and now - check_interval > 3.0:
                new_path = find_latest_log(args.watch)
                if new_path and new_path != log_path:
                    print(f"Switching to new log: {new_path}")
                    log_path = new_path
                    tail = LogTail(new_path)
                check_interval = now

            time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nStopped.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
