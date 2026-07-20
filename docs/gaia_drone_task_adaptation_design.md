# Gaia Drone Task Adaptation Design

This document consolidates the current project findings and the next design
direction for adapting Gaia/Stabilize SDK actions to an ArduCopter hover task.

## 1. Background

The current SITL pipeline is already connected end to end:

```text
Gazebo X3 / ArduCopter SITL
    -> MAVLink state
    -> CopterState
    -> obs16 + feedback1
    -> Gaia SDK
    -> action10
    -> RC_CHANNELS_OVERRIDE
    -> ArduCopter native RC / attitude / motor pipeline
```

The important distinction is:

```text
Gaia action10 -> RC PWM            project adapter layer
RC PWM -> attitude -> motor PWM    original ArduCopter control stack
```

The project already reuses ArduCopter's native RC input, stabilization, throttle
control, and motor mixing path. The weak point is the adapter that interprets
Gaia's normalized `action10`.

Gaia was understood to be trained originally on a push-ball style task. Its
actions are normalized and intended to be adapted to downstream tasks, not
directly treated as aircraft roll/pitch/throttle/yaw sticks. Therefore,
`sdk-direct` is useful as a stress test, but it should not remain the primary
control design.

## 2. Design Principle

Do not let Gaia directly command the aircraft.

Use Gaia as a residual or high-level correction source around a stable
ArduCopter baseline:

```text
baseline hover RC
    + small Gaia residual
    + SafetyShield
    -> RC_CHANNELS_OVERRIDE
    -> ArduCopter native control stack
```

This keeps mature flight-control logic responsible for fast attitude
stabilization and motor mixing, while Gaia contributes task-specific correction.

## 3. Recommended Control Architecture

### 3.1 Primary Path

```text
CopterState
    -> staged obs
    -> staged feedback
    -> Gaia action10
    -> learned/residual adapter W @ action10
    -> bounded rc_delta
    -> baseline_rc + rc_delta
    -> SafetyShield.filter_rc
    -> MAVLink RC_CHANNELS_OVERRIDE
```

The adapter output should be small:

```text
roll_delta_pwm      <= 10..30
pitch_delta_pwm     <= 10..30
throttle_delta_pwm  <= 20..50
yaw_delta_pwm       <= 10..30
```

The current `sdk-direct` path should be retained only for debugging action
saturation and model behavior, not for stability evaluation.

### 3.2 Adapter Options

Preferred order:

1. `baseline only`: verify ArduCopter hover and reset stability.
2. `front-4 residual`: action[0..3] mapped to small RC residual with low lambda.
3. `learned W`: fit `W @ action10 -> rc_delta` from logs and A/B test.
4. `sdk-direct`: diagnostic only.

## 4. Staged Observation Design

The obs should reflect task phase. The same physical quantities can be present
in every phase, but their meaning and feedback weight should change.

### 4.1 Takeoff

Goal: safely reach a flyable altitude without lateral authority.

Important signals:

```text
z_err, altitude_m, vz, roll, pitch, p, q, throttle history
```

Control behavior:

```text
roll/pitch/yaw neutral below the low-altitude gate
baseline throttle owns climb
Gaia receives neutral feedback during warm-up
```

### 4.2 Climb / Settle

Goal: reach target altitude and reduce vertical velocity.

Important signals:

```text
z_err, vz, tilt, throttle PWM, action saturation
```

Control behavior:

```text
allow small throttle residual
keep lateral residual limited
```

### 4.3 Hover

Goal: minimize 3D error while keeping attitude and command effort small.

Important signals:

```text
x_err, y_err, z_err, vx, vy, vz, roll, pitch, yaw_err, rates, last_rc_delta
```

Control behavior:

```text
allow learned small RC residual
penalize saturation and safety intervention
```

### 4.4 Reset / Recovery

Goal: avoid turning reset into a reward exploit and avoid carrying stale action
history across episodes.

Behavior:

```text
reset trigger frame: terminal feedback
post-reset frames: neutral feedback warm-up
clear last_rc_delta / action history
optionally hold neutral or baseline RC briefly before SDK resumes
```

## 5. Feedback Design

Use `reward_01` semantics:

```text
1.0 = good hover / low cost
0.0 = terminal or bad state
0.5 = neutral / warm-up
```

Current recommended behavior:

```text
terminal_value = 0.0
neutral_value = 0.5
startup_warmup_frames = 10
```

Feedback should include:

```text
distance / xy / z tracking
velocity
tilt and angular rates
command effort
action saturation
safety/reset flags
```

Avoid giving high initial feedback; it leaves little room for improvement.
Avoid giving low neutral feedback after reset; it can teach the model to seek
fast reset. Use a terminal penalty only on the failing frame, followed by
neutral warm-up.

## 6. Reset Design

Current reset gates:

```text
flip:       roll/pitch exceeds threshold
xy_runaway: sqrt(x_err^2 + y_err^2) exceeds threshold for hold time
ground_stuck: high throttle, low altitude, low vertical speed
```

Current implementation:

1. In SITL, the recommended scripts use Gazebo truth for reset distance gates.
2. Reset trigger frames are logged with terminal feedback.
3. A short post-reset guard runs even when full reset recovery is disabled:

```text
--post-reset-guard-s 0.75
send baseline RC
clear last_rc_delta
neutral feedback warm-up
no SDK authority during guard
```

This prevents immediate saturated SDK actions from acting on stale or
half-synchronized state after a pose reset.

## 7. Data Collection Plan

Collect logs with:

```text
action10
copter_state
feedback + diagnostics
baseline_rc
candidate_command
safe_command
rc_delta
gazebo_reset reason
future distance / future xy / future z
```

For contribution analysis, align action at time `t` with outcomes at:

```text
t + 100 ms
t + 200 ms
t + 300 ms
```

Targets:

```text
future distance improvement
future z improvement
future xy improvement
tilt and saturation penalties
```

Then fit:

```text
rc_delta = W @ action10
```

and evaluate action dimensions with supervised importance, not only PCA.

## 8. A/B Test Plan

Run the same initial conditions and reset settings for:

```text
A: baseline only
B: front-4 residual
C: learned W residual
D: sdk-direct diagnostic
```

Metrics:

```text
success_rate
time_to_stable_hover
distance_p50 / p95 / max
xy_p95
z_err_p95
tilt_p95
action_saturation_pct
safety_clip_count
reset_count and reason distribution
```

Accept learned W only if it improves distance or stability without increasing
reset count, saturation, or tilt risk.

## 9. Immediate Next Steps

1. Stop treating `sdk-direct` as the main evaluation mode.
2. Collect a clean residual dataset with `scripts/start_residual_infinite_test.sh`.
3. Fit `W @ action10 -> rc_delta`.
4. A/B test front-4 residual vs learned W residual.
5. Keep `sdk-direct` for diagnostic stress tests only.
