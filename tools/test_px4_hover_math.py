#!/usr/bin/env python3
"""Small dependency-light test for PX4 NED/FRD to SDK FLU control math."""

from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
ROS_PACKAGE = ROOT / "ros2_ws" / "src" / "micoair_hover"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROS_PACKAGE))

from flight_bridge.command_types import VelocityCmd
from micoair_hover.frames import body_flu_to_world_ned, world_ned_to_body_flu
from micoair_hover.hover_logic import (
    LocalNedState,
    SdkResidualSchedule,
    SitlVisualDisturbance,
    add_velocity_commands,
    build_sdk_state,
    capture_hover_target,
    position_setpoint_velocity,
    scale_velocity_command,
)


def assert_close(actual: float, expected: float, label: str) -> None:
    if not math.isclose(actual, expected, abs_tol=1e-9):
        raise AssertionError(f"{label}: expected {expected}, got {actual}")


def test_frame_round_trip() -> None:
    original = (0.23, -0.11, 0.07)
    yaw_ned_rad = math.radians(37.0)
    ned = body_flu_to_world_ned(*original, yaw_ned_rad)
    restored = world_ned_to_body_flu(*ned, yaw_ned_rad)
    for index, (actual, expected) in enumerate(zip(restored, original)):
        assert_close(actual, expected, f"frame round trip component {index}")


def test_hover_target_and_sdk_state() -> None:
    state = LocalNedState(
        north_m=4.0,
        east_m=-2.0,
        down_m=0.0,
        north_mps=0.0,
        east_mps=0.0,
        down_mps=0.0,
        yaw_ned_rad=0.0,
    )
    target = capture_hover_target(state, target_altitude_m=2.0)
    assert_close(target.down_m, -2.0, "hover target down")
    sdk_state = build_sdk_state(state, target, armed=False, mode="POSCTL")
    assert_close(sdk_state.x_err, 0.0, "hover x error")
    assert_close(sdk_state.y_err, 0.0, "hover y error")
    assert_close(sdk_state.z_err, 2.0, "hover z error")


def test_residual_limit() -> None:
    limited = position_setpoint_velocity(
        VelocityCmd(vx=1.0, vy=1.0, vz=1.0, yaw_rate=1.0),
        max_horizontal_mps=0.1,
        max_vertical_mps=0.05,
        max_yaw_rate_rad_s=0.1,
    )
    assert_close(math.hypot(limited.vx, limited.vy), 0.1, "horizontal limit")
    assert_close(limited.vz, 0.05, "vertical limit")
    assert_close(limited.yaw_rate, 0.1, "yaw limit")


def test_sdk_residual_schedule() -> None:
    schedule = SdkResidualSchedule(start_weight=1.0, hold_s=2.0, decay_s=20.0, final_weight=0.0)
    assert_close(schedule.weight_at(0.0), 1.0, "SDK initial weight")
    assert_close(schedule.weight_at(2.0), 1.0, "SDK hold weight")
    assert_close(schedule.weight_at(12.0), 0.5, "SDK midpoint weight")
    assert_close(schedule.weight_at(22.0), 0.0, "SDK final weight")

    scaled = scale_velocity_command(VelocityCmd(0.08, -0.04, 0.02, 0.1), 0.5)
    assert_close(scaled.vx, 0.04, "SDK scaled vx")
    assert_close(scaled.vy, -0.02, "SDK scaled vy")
    assert_close(scaled.vz, 0.01, "SDK scaled vz")
    assert_close(scaled.yaw_rate, 0.05, "SDK scaled yaw rate")


def test_sitl_visual_disturbance() -> None:
    disturbance = SitlVisualDisturbance(
        hold_s=2.0,
        decay_s=4.0,
        horizontal_mps=0.5,
        vertical_mps=0.1,
        period_s=8.0,
    )
    initial = disturbance.command_at(0.0)
    assert_close(initial.vx, 0.5, "visual demo initial vx")
    assert_close(initial.vy, 0.0, "visual demo initial vy")
    midpoint = disturbance.command_at(4.0)
    assert_close(midpoint.vx, -0.25, "visual demo attenuated vx")
    assert_close(midpoint.vy, 0.0, "visual demo attenuated vy")
    final = disturbance.command_at(6.0)
    assert_close(final.vx, 0.0, "visual demo final vx")
    assert_close(final.vy, 0.0, "visual demo final vy")
    if not disturbance.is_complete(6.0):
        raise AssertionError("visual demo should report completion after hold plus decay")

    combined = add_velocity_commands(initial, VelocityCmd(-0.2, 0.1, 0.05, 0.0))
    assert_close(combined.vx, 0.3, "combined vx")
    assert_close(combined.vy, 0.1, "combined vy")
    assert_close(combined.vz, 0.05, "combined vz")


def main() -> int:
    test_frame_round_trip()
    test_hover_target_and_sdk_state()
    test_residual_limit()
    test_sdk_residual_schedule()
    test_sitl_visual_disturbance()
    print("PX4 hover math smoke test passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
