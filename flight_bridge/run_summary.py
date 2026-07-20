"""Loop timing and action summary helpers for the flight bridge."""

from __future__ import annotations

import statistics
import time
from dataclasses import dataclass, field


@dataclass
class ActionSummary:
    frames: int = 0
    action_dim: int | None = None
    latency_ms: list[float] = field(default_factory=list)
    action_min: float | None = None
    action_max: float | None = None
    shield_clips: int = 0
    shield_filtered: int = 0
    shield_safety_triggers: int = 0
    commands_sent: int = 0
    gazebo_resets: int = 0

    def update(self, action: list[float], latency_ms: float) -> None:
        self.frames += 1
        self.latency_ms.append(latency_ms)
        self.action_dim = len(action)
        if action:
            frame_min = min(action)
            frame_max = max(action)
            self.action_min = frame_min if self.action_min is None else min(self.action_min, frame_min)
            self.action_max = frame_max if self.action_max is None else max(self.action_max, frame_max)

    def update_shield(self, clipped: bool, filtered: bool, safety: bool) -> None:
        if clipped:
            self.shield_clips += 1
        if filtered:
            self.shield_filtered += 1
        if safety:
            self.shield_safety_triggers += 1

    def update_sent(self) -> None:
        self.commands_sent += 1

    def update_gazebo_reset(self) -> None:
        self.gazebo_resets += 1

    def format_line(self) -> str:
        latency_mean = statistics.fmean(self.latency_ms) if self.latency_ms else 0.0
        latency_p95 = percentile(self.latency_ms, 95.0) if self.latency_ms else 0.0
        parts = [
            f"frames={self.frames}",
            f"action_dim={self.action_dim}",
            f"latency_mean_ms={latency_mean:.2f}",
            f"latency_p95_ms={latency_p95:.2f}",
            f"action_min={_format_optional(self.action_min)}",
            f"action_max={_format_optional(self.action_max)}",
        ]
        if self.shield_clips:
            parts.append(f"shield_clips={self.shield_clips}")
        if self.shield_filtered:
            parts.append(f"shield_filtered={self.shield_filtered}")
        if self.shield_safety_triggers:
            parts.append(f"shield_safety={self.shield_safety_triggers}")
        if self.commands_sent:
            parts.append(f"commands_sent={self.commands_sent}")
        if self.gazebo_resets:
            parts.append(f"gazebo_resets={self.gazebo_resets}")
        return " ".join(parts)


def should_print_summary(frame_index: int, summary_every: int) -> bool:
    return frame_index == 0 or (frame_index + 1) % summary_every == 0


def format_frame_summary(
    frame_index: int,
    vehicle_mode: str,
    obs: list[float],
    action: list[float],
    latency_ms: float,
) -> str:
    action_min = min(action) if action else 0.0
    action_max = max(action) if action else 0.0
    return (
        f"frame={frame_index:06d} vehicle_mode={vehicle_mode} "
        f"obs_dim={len(obs)} action_dim={len(action)} latency_ms={latency_ms:.2f} "
        f"action_min={action_min:.3f} action_max={action_max:.3f}"
    )


def sleep_remaining(frame_started_at: float, period_s: float) -> None:
    elapsed_s = time.perf_counter() - frame_started_at
    remaining_s = period_s - elapsed_s
    if remaining_s > 0:
        time.sleep(remaining_s)


def percentile(values: list[float], percentile_value: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = (len(ordered) - 1) * (percentile_value / 100.0)
    lower_index = int(index)
    upper_index = min(lower_index + 1, len(ordered) - 1)
    fraction = index - lower_index
    return ordered[lower_index] * (1.0 - fraction) + ordered[upper_index] * fraction


def _format_optional(value: float | None) -> str:
    return "n/a" if value is None else f"{value:.3f}"
