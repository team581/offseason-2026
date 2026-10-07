# Trailblazer 2: Auto Specification and Execution Design

Oct 7, 2026 · @Saikiran

## Summary and goals

Trailblazer 2 (working name) keeps the property that made Trailblazer dependable, driving from the robot's measured state to the next goal with no stored trajectory, and replaces the parts that made it hard to tune.

Every loop the planner takes the measured pose and velocity, the auto spec and a drivetrain limits record. It builds a short path through the next goals, solves a feasible speed along it, and returns a velocity command. A shove, a vision jump or a moving goal costs nothing extra, because the next loop starts from where the robot actually is.

**Goals**

- **Robust by construction.** The only state is the segment index, a latched leg start and the last command. There is no plan to invalidate.
- **Feasible by construction.** Commands respect acceleration, braking, cornering and rotation limits that are measured once for the drivetrain, not tuned per auto.
- **Few knobs.** Authors state intent (stop here, pass within 0.3 m, face the target by 1 m before the goal) and physics decides speeds.
- **Easy to change.** Edit a goal, see the motion in simulation within seconds, and let CI say whether the auto still works.
- **Incremental.** The `Trailblazer` facade and the tracker/follower split stay, so each piece can land on its own.

**Not goals:** obstacle-avoiding global planning, precomputed time-optimal trajectories, and replacing pose estimation.

## What the current code does

The current design is robust because its state is tiny, and its tuning cost traces to eight specific mechanisms. These come from reading the `trailblazer` package and `LeftNormalAuto`; I did not run anything.

| Mechanism | Where | Effect |
| --- | --- | --- |
| Tolerance both advances and shapes the path | `HeuristicPathTracker`, `AutoPoint.transitionTolerance` | A goal counts as reached only inside its tolerance, so tolerances decide how tightly corners are cut. One `LeftNormalAuto` segment mixes 0.2 m / 30° with 0.75 m / 100°. |
| Speed is a PID on remaining distance | `PidPathFollower` | `translationController.calculate(distanceToEnd, 0)` sets speed, so the speed profile is whatever the gain produces, then clamped. |
| Acceleration is limited upward only | `ConstraintsCalculator.constrainLinearVelocity` | The command is min(desired, last + a·dt), so any drop in requested speed passes through instantly. The stopping cap uses distance to the segment end, and a slower per-point cap applies only once that point is current, so the robot meets it at full speed. A 0.5 m/s floor also lets a stationary robot jump to 0.5 m/s. |
| Velocity direction is unconstrained | `PidPathFollower`, `PolarChassisSpeeds` | Magnitude is limited but direction is set toward the target, so nothing bounds cornering acceleration. I did not read `MathHelpers.getDriveDirection`. |
| Rotation pacing is hand-built | `PidPathFollower` | Two overlapping caps, 0.4 m/s floors, a "command zero inside tolerance" rule, and a 360° default tolerance that disables the cap. |
| Arc progress is heuristic | `HeuristicPathTracker`, `AutoPoint.withArcExtension` | t = d\_start / (d\_start + d\_target), ratcheted forward; the rotation target flips at t = 0.5; Bezier control points come from hand-typed offsets. |
| Limits are chosen per auto | `LinearConstraintOptions`, `LeftNormalAuto` | The default is 4.75 m/s and 4 m/s², segments in `LeftNormalAuto` use 4.5 m/s and 8 m/s², and individual points override down to 2.0 to 2.5 m/s. Nothing ties these to the drivetrain's real limits. |
| Geometry is code | `LeftNormalAuto` | Goals are red-frame meter constants and lambdas (`getCollisionPoint`, `BUMP_OFFSET`). I found no offline preview of the resulting motion in the files I read. |

Keep: the `Trailblazer` facade, segment end conditions, enum markers, and supplier-based goals for dynamic targets.

## Design principles

Six rules decide every later choice in this document.

1. **Re-solve, do not replay.** The planner is a pure function of measured state, spec and limits. Its only memory is the segment index, a latched leg start, a progress high-water mark and the last commanded velocity.
2. **Intent in the spec, geometry derived.** Tolerances answer "is it done", never "what shape is the path". Corner radii and pass speeds come from physics and from how close the author needs the robot to pass.
3. **Physical parameters are measured once.** Acceleration, braking, cornering, rotation rate and drive lag belong to the drivetrain, not to each auto.
4. **Commands are feasible by construction.** A two-pass speed profile and a vector limiter make an over-limit command impossible, not merely unlikely.
5. **One planner everywhere.** The code that runs on the robot also runs in simulation, in CI and in log replay, with time and enable state injected.
6. **Fail visibly.** Every loop logs which constraint is limiting speed, and abnormal states (stall, pose jump, infeasible goal) raise named events instead of silently degrading.

## Auto specification model

An auto is a list of segments, and a segment is an ordered list of goals plus a limits profile, optional zones and an end condition. The spec says what the robot should do; geometry and speeds are derived at run time.

### Goal

| Field | Meaning | Replaces |
| --- | --- | --- |
| `pose` | Position and nominal heading. A constant, a named field anchor with an offset, or a supplier for dynamic targets. Authored in one alliance frame and mirrored by the loader. | `poseSupplier`, `Point.ofRed` / `ofBlue` at call sites |
| `pass` | `stop`, `through` or `through(minSpeed)`. A stop has exit speed 0. A through goal gets the fastest speed the geometry allows; `minSpeed` is a request that CI flags if the geometry cannot support it. | per-point `LinearConstraintOptions`, tolerance as speed shaper |
| `within` | For through goals, the maximum distance in meters the path may deviate from the goal. It caps the corner radius and therefore the pass speed. | linear half of `transitionTolerance` |
| `finish` | For stop goals, the position, heading and speed tolerances that count as arrived. Drives `atGoal`. | `untilFinished(PoseErrorTolerance)` |
| `heading` | A heading plan (next subsection). | angular tolerance, `arcMidpoint` rotation |
| `marker` | An enum fired when progress passes the goal, or a set distance before or after it. | `withMarker` |

### Heading plan

Heading is a schedule over path progress, not a target at every goal. A keyframe has an anchor, a heading, a turn direction and a gate.

- **Anchor:** at a goal, or a distance before or after it.
- **Heading:** absolute, `faceTravel`, `facePoint(x, y)` or `hold`.
- **Turn:** `shortest`, `cw` or `ccw`. This settles rotation direction explicitly instead of through a midpoint rotation.
- **Gate:** `hard` slows translation so the robot is within tolerance by the anchor, waiting if needed. `soft` rotates as fast as limits allow and never slows translation.

### Zones

A zone is a field-fixed circle or polygon with its own rule: a speed cap, an acceleration scale, a heading hold or a keepout. Zones live apart from goals, so moving a goal does not change what the bump does. Keepouts are checked in simulation and CI, not enforced at run time.

### Limits profiles

A segment names a profile such as `fast`, `careful` or `intake`, defined as fractions of the measured drivetrain limits (for example 0.8 × cornering acceleration). Specs do not accept absolute m/s numbers, and CI rejects any profile above 1.0 of a measured limit.

### Markers and end conditions

Segments end `forever` or when the last stop goal's `finish` is met, as today. Markers keep the enum API (`passedMarker(READY_TO_CROSS_BUMP)`) but fire on progress, so each fires once and in order.

### Example

A sketch of the syntax with values borrowed from `LeftNormalAuto`. It is not a tested translation.

```yaml
segment: intake_first_cycle
frame: red
limits: fast
end: stop-finish
goals:
  - id: trench
    pose: {x: 10.489, y: depot_trench.y, heading: 90deg}
    pass: through
    within: 0.75
  - id: ball_a
    pose: collision_a            # dynamic anchor, resolved in Java each loop
    pass: through
    within: 0.3
    heading: {to: 115deg, before: 1.0, turn: shortest, gate: soft}
    marker: PRIORITIZE_INTAKE
  - id: bump_ready
    pose: {x: 13.576, y: depot_bump.y - 0.13, heading: 0deg}
    pass: stop
    finish: {pos: 0.2, heading: 3deg, speed: 0.2}
    marker: READY_TO_CROSS_BUMP
```

## Runtime changes

Each loop runs five pure steps in order: build the path, update progress, solve the speed limit, plan heading, then limit and emit the command.

&#91;embedded content: per-loop pipeline · 5 steps, 1 state box\]

Only the state box persists between loops; nothing is stored as a trajectory.

### Path geometry

The path is a polyline from the robot through the goals in the horizon, with each interior corner rounded by a circular fillet. Each leg's line is latched when the robot enters it: from the robot's actual position for the first leg, from the previous goal for later legs. Latching gives cross-track error a meaning, since a path rebuilt from the robot's position every loop would always show zero error.

For a corner with deflection angle θ and fillet radius r, the tangent length and the path's deviation from the corner are:

```latex
t = r \tan(\theta / 2)
d_{dev} = r \left( \frac{1}{\cos(\theta / 2)} - 1 \right)
```

The radius is bounded three ways. The goal's `within` δ gives r ≤ δ / (1 / cos(θ/2) − 1). Leg length gives t no more than half of each adjacent leg, shared fairly. The requested pass speed gives r ≥ v² / a\_lat. A `stop` goal has r = 0 and exit speed 0, and a reversal (θ near 180°) is forced to a stop.

The horizon runs until the path is longer than the braking distance at top speed, v\_max² / (2 · a\_brake), or reaches the next stop goal. Goals farther out cannot change the current command, because the robot can always brake in time. Dynamic goals rebuild the geometry each loop. Latched starts stay fixed, so a moving goal rotates its leg about the start instead of resetting it.

The planner re-latches when cross-track error exceeds `e_relatch`, when the pose jumps (a vision correction), or when the segment changes. Velocity is never reset, so a re-latch shows up as a direction change that the vector limiter bounds.

### Progress and goal advance

Progress `s` is the robot's projection onto the path and only moves forward, except across a re-latch. A through goal counts as passed when `s` passes the end of its fillet, so a late or off-line robot never stalls waiting to enter a circle. A stop goal completes when its `finish` holds for position, heading and speed. Markers fire when `s` crosses their anchor, and `getCurrentPointIndex()` returns the leg that contains `s`.

### Velocity profile

The horizon is sampled at a fixed arc-length spacing ds. Each sample k gets a speed limit from the global maximum, zone caps, the corner limit √(a\_lat / κ), and the pass speed at goal samples. A stop goal pins its sample to 0. A backward pass then a forward pass makes the profile feasible:

```latex
v_{lim,k} = \min\left( v_{max},\ \sqrt{a_{lat} / \kappa_k},\ v_{zone},\ v_{goal} \right)
a_{t,k} = \sqrt{ a_{fric}^2 - (v_k^2 \kappa_k)^2 }
v_k^2 \le v_{k+1}^2 + 2\, a_{brake,k}\, ds \quad \text{(backward)}
v_{k+1}^2 \le v_k^2 + 2\, a_{acc,k}\, ds \quad \text{(forward, } v_0 = \text{last command)}
```

Here a\_brake,k and a\_acc,k are capped by the along-track acceleration a\_t,k that remains after cornering, and a\_acc also follows the motor curve a0 (1 − v / v\_free). The commanded speed is the profile value at s + v · (Δt + τ\_lag), which compensates control latency. The forward pass starts from the last commanded speed rather than the measured one, so noisy velocity cannot cause chatter; if the two diverge past a threshold (a shove or wheel slip) it starts from the measured speed. Buffers are preallocated, and the sample count should be benchmarked on the target controller.

## Tooling: how a human edits an auto

An author edits a text file or records goals from the robot, sees the motion in simulation within seconds, and reads a short list of metrics instead of tuning gains on the field.

### Spec as data

Static geometry lives in `*.auto.yaml` files under version control, validated by a JSON Schema in the IDE and in CI. Named field anchors (`depot_bump`, `hub_center`) live in one `field.yaml`, so no auto retypes meter constants. Dynamic targets are named anchors resolved in Java, such as `collision_a`.

Autos are authored in one alliance frame and the loader mirrors them for the other alliance and for left/right variants, which removes `Point.ofRed` and `ofBlue` from call sites. Java keeps the logic: the `BaseImperativeAuto` state machines choose segments and react to markers.

### The edit loop

1. Edit the file, or record goals from the robot (next subsection).
2. Run `./gradlew previewAuto -Pauto=<name>`. A headless simulation writes a `.wpilog` and a PNG.
3. Open the log in AdvantageScope for the ghost path, robot footprint, speed against limit, and the limiting factor over time.
4. Read the metrics block: duration, peak speed, peak acceleration, minimum clearance to keepouts, and the slowest goal with the reason.
5. Commit. CI repeats steps 2 to 4 for every changed auto and posts the PNG and metrics to the pull request.

### Record from teleop

A driver button publishes the current pose to NetworkTables, in simulation or on the real robot. A small script appends it to a chosen segment as a `through` goal with the default `within`, and the author adjusts from there.

### Symptom to knob

Tuning becomes a lookup. Anything that needs a global fix points at calibration, not at the auto.

| Symptom | Knob | Scope |
| --- | --- | --- |
| Cuts a corner into an obstacle | Lower `within` on that goal | This goal |
| Too slow through a corner | Raise `within`, or move neighboring goals farther apart so a wider arc fits | This goal |
| Slows or waits before a goal for rotation | Move the heading anchor earlier, or change the gate to `soft` | This keyframe |
| Rotates the long way | Set `turn: cw` or `ccw` | This keyframe |
| Overshoots a stop | Recalibrate the braking limit | Drivetrain |
| Wobbles on a straight leg | Check τ\_lag and τ\_xt | Drivetrain |

### Editor, in two stages

Stage one is the IDE plus the schema plus AdvantageScope, which is enough for most teams. Stage two, only if authors ask for it, is a field-image editor with draggable goals that reads and writes the same YAML. It must load the planner itself, so its preview is the real planner and never a re-implementation.

### Calibration routine

A `Calibrate` auto drives the tests behind the parameter table and writes `drivetrain-limits.json`. Limit changes are reviewed like any other diff, and CI replays every auto against the new file. When the robot changes (mass, wheels, battery), recalibrate; the specs do not change.

## Testing and CI

Because the planner is a deterministic function of state, spec and limits, every auto can run headless in CI against a drivetrain model, and a failing auto blocks the merge.

### Test layers

| Layer | What it checks | When |
| --- | --- | --- |
| Unit | `PathBuilder` geometry (tangent lengths, the deviation formula, collinear goals, reversals, short legs). `VelocityProfiler` (never exceeds limits, reaches stops, matches the closed-form trapezoid on a straight). `HeadingPlanner` (turn direction, gate cap). `ProgressTracker` (monotone `s`, re-latch). | Every push |
| Property / fuzz | Random goal lists and start states. Invariants: no NaN, acceleration and speed within limits, the run terminates. | Small on every push, large nightly |
| Closed-loop sim per auto | Planner plus a drivetrain model (first-order lag τ\_lag, acceleration and friction limits) with pose noise, vision jumps and pushes. | Every push |
| Golden metrics | Per-auto duration, peak acceleration and clearance against stored baselines. | Every push |
| Log replay | Recorded `.wpilog` poses and velocities fed to the planner, commands compared with the logged ones, and achieved against commanded acceleration to spot drift in the limits. | Nightly or on demand |
| On robot | The checklist below. | Before events |

### Per-auto assertions

Each is declared in the spec or defaulted, and each fails the build when violated.

- The auto finishes within its time budget (`budget` on the segment).
- Every stop goal meets its `finish` tolerance.
- The path passes within `within` of every through goal.
- Peak acceleration, lateral acceleration and rotation rate stay inside the drivetrain limits.
- The robot footprint stays out of every keepout zone.
- Markers fire once, in order.
- All of the above still hold across N seeded perturbation runs: start offsets, pose noise, a shove, a vision jump.
- Blue and red runs are mirror images with equal metrics.
- Dynamic anchors swept across their plausible range still pass.

### CI pipeline

1. Build and run unit tests.
2. Schema-validate every spec and anchor file.
3. Run every auto in simulation with fixed seeds, in parallel.
4. Compare metrics with golden baselines. A change past the threshold fails unless the baseline update is in the same pull request.
5. Publish a path PNG and a metrics table as pull request artifacts.
6. Nightly: large fuzz and perturbation sweeps, then log replay on the latest field logs.

Determinism needs injected time. `ConstraintsCalculator` reads `MathSharedStore.getTimestamp()` and `DriverStation.isDisabled()` directly today. The new planner takes a `TimeSource` and an enabled flag so the same code runs in CI. The tests live in `shared`, so both robot projects get them.

### What CI cannot promise

The drivetrain model is an approximation. CI catches infeasible specs, regressions and over-limit commands, and it removes most pre-event tuning, but it does not prove an auto works on carpet. Track a model residual (simulation against logged real runs) and keep the on-robot checklist.

### On-robot checklist

- Recalibrate after any drivetrain change and commit the limits file.
- Run three reference autos and compare logs with simulation: duration, peak acceleration, finish error.
- Read the limiting-factor graph for each auto and look for unexpected binds.
- Run each alliance and each start position once.

## Migration, risks and non-goals

Five phases move from Trailblazer to Trailblazer 2, and each ends at a gate that the CI harness can check. Phase 2 alone removes the acceleration/braking asymmetry, so the plan pays off even if later phases slip.

1. **Instrument.** Inject `TimeSource`, add limiting-factor logging, build the replay harness and record baseline metrics for the current autos. Gate: simulation reproduces the logged behavior of current autos.
2. **Limits.** Add `DrivetrainLimits` and the symmetric vector limiter inside the current follower. Gate: logs show acceleration and braking inside limits and autos still pass.
3. **Planner.** Add `PathBuilder`, `ProgressTracker` and `VelocityProfiler` behind the existing interfaces, with an adapter that maps `AutoPoint` to goals. Gate: every auto is green in CI simulation and one has run on the field.
4. **Spec and tools.** Add heading plans, zones, the YAML loader, the preview task and record-from-teleop. Gate: someone who did not write the planner authors a new auto.
5. **CI depth and cleanup.** Add perturbation sweeps and the calibration routine, then delete `PidPathFollower`, `HeuristicPathTracker` and `ConstraintsCalculator`. Gate: old classes gone and the nightly job is green.

&#91;embedded content: migration roadmap · 5 phases, 5 gates\]

Phases run in order, and each gate is a CI result or a field run, not a date.

### Risks

| Risk | Mitigation |
| --- | --- |
| A fillet cuts through an obstacle | `within` is a geometric maximum deviation, keepout assertions run in CI, and the preview shows the swept footprint. |
| Profile chatter from noisy velocity | The forward pass starts from the last command; measured speed is used only when it diverges. Hysteresis tests cover it. |
| Simulation passes but the robot does not | Track the model residual, recalibrate, keep the on-robot checklist and the log replay. |
| Dynamic goals jitter (vision) | A re-latch deadband and a low-pass on the goal, with the vector limiter bounding the effect. |
| Loop time on the target controller | Preallocate buffers, cap the sample count, and run the profile at a lower rate with interpolation if needed. |
| Newcomers cannot use it | The symptom-to-knob table, worked examples and the preview make the first auto a copy-and-edit. |

### Deliberately not doing

- A global planner that routes around obstacles.
- Precomputed time-optimal trajectories.
- Model-predictive control in the first version. It could later replace the cross-track term and reuse the same profile and limits.
- Per-module torque modeling beyond what a swerve setpoint generator provides.

## Open questions

Seven decisions change the plan, and each has a default so work can start.

| Decision | Default | Why it matters |
| --- | --- | --- |
| Evolve Trailblazer in place, or build a clean-room tool for a future team? | In place, as the migration plan assumes | A clean-room build skips phases 1 to 3 but loses the existing autos as a regression baseline. |
| YAML plus Java loader, or a Java DSL only? | YAML for geometry, Java for logic and dynamic anchors | Files are diffable, mirrorable and editable by a visual tool. |
| Is a drag-and-drop editor worth building? | No; use AdvantageScope plus record-from-teleop first | The editor is the largest tooling cost and the preview already shows the motion. |
| Does `Swerve` already have module-level limits or a setpoint generator? | Unknown; I did not read it | It decides whether the vector limiter alone is enough on the drivetrain side. |
| What is the pose estimator's noise and vision latency? | Measure from logs | They set the re-latch threshold and the pose-jump detector. |
| Which goals are hard heading gates (for example before a shot)? | Treat shooting goals as `hard`, intake approaches as `soft` | Wrong gates either stall autos or fire shots off-heading. |
| Which autos form the reference set for baselines? | The four left/right normal and special autos | Golden metrics and log replay need a stable set. |

### Heading and rotation coupling

Heading keyframes evaluate to a target θ\*(s). A trapezoidal profile from the measured heading and rate toward θ\*, limited by ω\_max and α\_max, supplies the rotation feedforward, and a P term on heading error corrects it. For a `hard` gate the planner computes the minimum rotation time t\_rot and caps translation so the robot reaches the anchor no sooner than that:

```latex
v_{cap} = d_{anchor} / t_{rot}
```

This replaces the 0.4 m/s floors and the zero-speed rule. If heading is late the robot slows to a stop at the anchor and waits, by declared design. A `soft` gate never caps speed. A gate that does not clear within its timeout raises `STALLED`.

### Command and limiter

The translation command is the profile speed along the leg direction plus a cross-track correction toward the latched line, and the rotation command is the heading feedforward plus its P term:

```latex
v_{cmd} = v_{profile}\, \hat{t} + \mathrm{clamp}\left( e_{xt} / \tau_{xt} \right) \hat{n}
```

The cross-track gain is a time constant τ\_xt, not a unitless kP. A vector limiter then bounds the change in the command, |v\_cmd − v\_prev| ≤ a\_fric · dt, so speed changes and direction changes share one friction budget and acceleration and braking are symmetric. Module-level limits (steer rate, torque) belong in a swerve setpoint generator inside `Swerve`, downstream of Trailblazer.

### Failure handling

| Condition | Detection | Response |
| --- | --- | --- |
| Stalled | `s` advances less than a threshold for a set time while the command is above a stuck speed | `STALLED` event. Hooks can re-latch, skip a goal or start a recovery behavior. |
| Pose jump | Pose changes beyond a threshold in one loop | Re-latch, keep velocity continuity, log it. |
| Infeasible goal | The profile cannot reach the exit speed, for example a stop inside braking distance | Brake at the braking limit and raise `INFEASIBLE`. CI fails the spec. |
| Bad input | NaN pose, or a goal supplier that throws | Hold position and log the reason. |
| Disabled | Enabled flag from the injected state | Zero the last-command state, as `ConstraintsCalculator` does today. |

### Interfaces and structure

The `Trailblazer` facade keeps `setActiveSegment`, `getFieldRelativeSetpoint`, `atGoal`, `passedMarker` and `getCurrentPointIndex`, along with the tracker/follower split.

- `ProgressTracker` (latching, `s`, advance) replaces `HeuristicPathTracker`.
- `ProfiledFollower` (profile, heading, limiter) replaces `PidPathFollower`.
- `ConstraintsCalculator` is retired.
- New pure classes: `PathBuilder`, `VelocityProfiler`, `HeadingPlanner` and `DrivetrainLimits`.
- Time and enabled state enter through an injected `TimeSource`, replacing the `MathSharedStore.getTimestamp()` and `DriverStation` calls inside the planner.

Every loop also logs the active limiting factor (corner, zone, heading gate, stop, acceleration), so "why was it slow here" is one graph.

### Parameters measured once

| Parameter | Meaning | How to get it |
| --- | --- | --- |
| v\_max | Top speed | Drivetrain config, confirmed by a sprint |
| a0, v\_free | Acceleration curve a0 (1 − v / v\_free) | Straight-line sprints from rest |
| a\_brake | Braking limit | Hard stops from speed |
| a\_fric, a\_lat | Friction circle and cornering limit | Circle runs at rising speed until the wheels slip |
| ω\_max, α\_max | Rotation rate and acceleration | Spin test |
| τ\_lag | Drive response lag | Velocity step responses |
| τ\_xt, e\_relatch | Cross-track time constant and re-latch distance | Start from a small multiple of τ\_lag, then tune once in simulation |
