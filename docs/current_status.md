# Flight Bridge Current Status

中文版本: `docs/current_status_zh.md`

This document is the short, current source of truth for the active flight
bridge workflow. Older Rover-only MVP notes are archived under `docs/archive/`.

## Active Scope

The active path is Gazebo X3 + ArduCopter 2.74b SITL + Stabilize/Gaia SDK.
The SDK is used as a small residual outer-loop signal. ArduCopter remains
responsible for stabilization, attitude control, and motor mixing.

Primary control path:

```text
Gazebo odometry / MAVLink state
    -> CopterState
    -> obs16 + sdk_feedback1
    -> Stabilize/Gaia SDK
    -> action10
    -> ActionAdapter
    -> RC residual command
    -> SafetyShield
    -> MAVLink RC_CHANNELS_OVERRIDE
    -> ArduCopter SITL
```

Modern guided velocity control is present in code as an experimental backend,
but it is not the preferred path for ArduCopter 2.74b. The reliable first-line
control entry is RC override around an ArduCopter baseline.

## Main Documents

Keep these as the active design references:

- `docs/stabilize_ai_sdk_drone_control_design.md`: overall architecture and
  safety/control strategy.
- `docs/gaia_drone_task_adaptation_design.md`: Gaia downstream drone-task
  adaptation strategy, staged obs/feedback, reset handling, and residual
  adapter direction.
- `docs/fastapi_control_service.md`: FastAPI control-service operation guide,
  HTTP API examples, data-transfer path, logging fields, and SITL/SDK usage.

Analysis appendix:

- `docs/gaia_action_contribution_implementation.md`: action10 contribution
  fitting notes and learned adapter A/B criteria.
- `docs/hover_anchor_residual_plan.md`: current hover-task definition,
  reset/home XY anchor plan, baseline-first testing order, and SDK residual
  rollout criteria.
- `docs/open_source_control_integration_research.md`: mature open-source
  flight-control integration options and the recommended migration path away
  from hand-written RC XY control.
- `docs/sim2real_control_interface_spec.md`: v0.2 communication contract for
  sending simulator/SDK-side safe commands to a real-vehicle agent.

Archived Rover-era notes:

- `docs/archive/flight_bridge_implementation.md`
- `docs/archive/flight_bridge_project_plan.md`

## Common Commands

Dry-run SDK smoke test:

```bash
python -m flight_bridge.main --phase 0 --mode dry-run --control off --sdk-mode mock --iterations 100
```

Phase 1 SITL, RC residual path:

```bash
python -m flight_bridge.main \
  --phase 1 \
  --mode sitl \
  --control rc \
  --sdk-mode mock \
  --target-alt 3.0 \
  --reset-on-flip
```

Recommended infinite SITL run with Gaia SDK, Gazebo GUI, residual RC, Gazebo
truth, post-reset guard, and realtime visualization:

```bash
scripts/start_residual_infinite_test.sh
```

`scripts/start_sdk_direct_infinite_test.sh` is kept as a compatibility entry,
but it now also starts the residual path. `sdk-direct` remains a diagnostic
mode only; it directly interprets `action[0..3]` as aircraft sticks and should
not be used as the main stability evaluation.

Use the real SDK server by replacing `--sdk-mode mock` with the configured SDK
mode and credentials for manual commands. Keep `--control rc` explicit; control
is never implied.

Live visualizer:

```bash
python tools/visualize_feedback.py --watch logs/
```

## Runtime Feedback Semantics

There are two related but distinct quantities:

- `feedback`: the one-dimensional SDK feedback input. In Phase 1 this is built
  by `HoverFeedbackBuilder` as a reward/cost scalar for hover quality.
- `distance_m`: a real physical 3D distance reconstructed from `copter_state`:

```text
distance_m = sqrt(x_err^2 + y_err^2 + z_err^2)
```

The live visualizer plots the real `distance_m`, not the SDK feedback scalar
and not a tanh-normalized value.

## Key JSONL Fields

Each Phase 1 frame should preserve enough data for online debugging and offline
action contribution analysis:

```json
{
  "ts": 0.0,
  "frame": 0,
  "phase": 1,
  "mode": "sitl",
  "control": "rc",
  "rc_control_mode": "residual",
  "vehicle_mode": "STABILIZE",
  "armed": true,
  "state": {},
  "gazebo_odom": {},
  "copter_state": {
    "x_err": 0.0,
    "y_err": 0.0,
    "z_err": 0.0,
    "vx": 0.0,
    "vy": 0.0,
    "vz": 0.0,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw_err": 0.0,
    "p": 0.0,
    "q": 0.0,
    "r": 0.0,
    "altitude_m": 0.0
  },
  "obs": [],
  "feedback": [],
  "feedback_diagnostics": {},
  "action": [],
  "latency_ms": 0.0,
  "state_age_ms": 0.0,
  "gazebo_age_ms": 0.0,
  "timeout_discard": false,
  "baseline_rc": [1500, 1500, 1700, 1500],
  "candidate_command": [1500, 1500, 1700, 1500],
  "safe_command": [1500, 1500, 1700, 1500],
  "rc_delta": [0.0, 0.0, 0.0, 0.0],
  "shield": {
    "filtered": false,
    "saturated": false,
    "safety_triggered": false,
    "clipped_fields": []
  },
  "sent": true,
  "cmd_method": "RC_CHANNELS_OVERRIDE",
  "error": null
}
```

For contribution analysis, `action` must keep all 10 dimensions.

## Live Visualizer Panels

`tools/visualize_feedback.py` shows three panels:

1. Top: `3D distance (m)`, computed from `copter_state`; green line is the
   target distance of zero.
2. Middle: `z_err (m)` in red and `tilt (deg)` in orange.
3. Bottom: `throttle PWM` in green and `action saturation %` in purple.

Action saturation is the percentage of action dimensions with absolute value
greater than `0.9`.

## Reset Gates

With `--reset-on-flip`, Gazebo episode reset can trigger on excessive tilt,
horizontal runaway, or ground-stuck detection. The recommended SITL scripts use
Gazebo odometry truth for reset gates and visual metrics. The horizontal
runaway gate uses XY error only:

```text
xy_error_m = sqrt(x_err^2 + y_err^2)
```

It does not include altitude error. The current relaxed threshold is
`--reset-xy-runaway-m 20.0`, held for `--reset-runaway-hold-s 2.0` seconds.
When running with Gazebo truth and explicit `--target-x/--target-y`, XY error
is relative to that target. When running with `--no-use-gazebo-truth`, XY error
falls back to the MAVLink home position recorded after startup/reset.

After a reset:

```text
trigger frame: feedback = terminal_value = 0.0
next frames:   feedback warm-up = neutral_value = 0.5
guard window:  --post-reset-guard-s 0.75 suppresses SDK and sends baseline RC
history:       last_rc_delta/action history cleared
```

## Current Priorities

1. Use reset/home XY as the default hover anchor instead of hard-coding world
   `(0, 0)` unless an explicit fixed-point task is being tested.
2. Keep baseline RC altitude and XY speed-limiting behavior stable before
   increasing SDK authority.
3. Use low residual weight while collecting data.
4. Evaluate `front-4` vs learned `W @ action10` with logged metrics:
   `success_rate`, `distance_p95`, `tilt_p95`, saturation, and safety triggers.
5. Do not treat X3 parameters or learned adapter gains as directly transferable
   to a real custom quadrotor.
