# 04 — Runtime Integration and Teleop Assist

**Applies To:** `com.team581.trailblazer3.runtime`, `TimeSource`, `RobotStateSnapshot`, `TickInput`, `TickResult`, `DrivetrainConsumer`, `TeleopAssist`  

---

## 1. Overview

- Runtime owns state, updates once per tick, and manages activation, completion, cancellation, and faults.
- The 2027 robot adapter supplies time-normalized measurements and pre-mapped driver translation, binds controls, selects command ownership, and feeds commands to the closed-loop drivetrain consumer.
- Teleop uses the same motion calculations and physical limits as auto.
- `driveTo` aligns to a fixed field pose. `snapAngle` rotates to a fixed field heading with driver-controlled translation.

---

An auto state machine activates a named `AutoSegment` once on state entry. The robot adapter resolves alliance and clearance, then ticks one shared runtime once per loop. The next state can wait for a marker, a completed stop, or mechanism readiness. Activating a new segment clears the preceding segment's markers and completion, and seeds from applied velocity or measured motion. An intermediate stop inside a segment requires explicit release. Disabled, rejected, and faulted autos cannot advance states.

## 2. Persistent State

- Runtime owns motion state and passes it to pure calculators.
- Runtime owns activation/goal IDs, the set of markers recorded as passed, dynamic-goal snapshots, applied-command history, and timeout/dwell timers.
- Recovery remaps progress and preserves goal and event identity. See [01 — Geometry](01-path-geometry-and-kinematics.md#6-anchors-geometry-generations-and-recovery).
- Localization supplies validity and reset signals, with vision-noise thresholds owned by localization.

```java
public final class TrailblazerState {
    String activeGoalId;                              // Currently active goal ID in the horizon
    Pose2d horizonOrigin;                             // Fixed origin at activation, stop release, or recovery
    long geometryGeneration;                          // Prevent comparing old and rebuilt arc lengths
    double progressHighWaterMark;                     // Corridor-gated monotonic progress in this generation
    Set<Enum<?>> passedMarkers;                       // Markers recorded as passed across rebuilds
    Map<String, GoalPassageStatus> goalStatuses;      // Recorded passage diagnostics per goal ID
    Map<String, Double> closestApproachByGoalId;      // Minimum measured segment distance to active through goals
    Map<String, Pose2d> dynamicTargetSnapshots;       // Goal poses captured when geometryGeneration was built
    HardGateState hardGateState;                      // Active hard-gate hold, dwell timer, and open latch
    String activeHeadingKeyframeId;                   // Keyframe ID currently holding directional branch latch
    double unwrappedHeadingTargetRad;                 // Continuous target evolved by continuity after branch latch
    OptionalDouble latchedHeadingRad;                 // Captured angle for hold() or SINGULAR_HOLD
    boolean terminalConvergenceLatched;               // Latch bit + hysteresis for terminal 2D blend
    double headingReferenceRad;                       // Continuous unwrapped heading reference state theta_ref
    double headingReferenceVelocity;                  // Angular reference velocity
    ChassisSpeeds lastCommand;                        // Previous loop's emitted velocity vector
    double lastTimestamp;                             // Previous update timestamp from TimeSource (NaN on activation)
}
```

## 3. Tick Contract

- Run one update per control-loop tick across target 2027 control rates (`50–250 Hz`, configured `nominalDtSeconds` in `[0.002 s, 0.025 s]`). Accumulate all dwells and watchdogs in elapsed `TimeSource` seconds, never tick counts.
- **First-Tick & Duplicate-Tick `dt` Handling:** On the first `update` after `activate` or stop `release`, step envelopes with `dt = nominalDtSeconds`. On subsequent ticks, fault if `dt < 0` or `dt > max(0.10 s, 5 * nominalDtSeconds)`; if `0 <= dt < 1e-4 s` (same-tick `activate` + `update`), return the cached `TickResult` without dividing by zero; otherwise integrate with `dt_step = min(dt, 2 * nominalDtSeconds)`.
- Supply the plan's goals, heading plan, and zones to the pipeline.
- Capture one coherent `TickInput` containing the `TimeSource` timestamp, `RobotStateSnapshot`, resolved dynamic targets, and optional `driverFieldVelocityMpS`.
- **Monotonic Clock Normalization:** Convert all incoming timestamps (localization pose timestamp and dynamic target timestamps) into the single `TimeSource` monotonic controller clock domain (seconds) at the robot-integration boundary before building `TickInput`—never compare raw CAN FD, IMU, or vision coprocessor epochs directly against controller time.
- **Power-State Contract:** Optional `swerveSupplyCurrentLimitAmps` must represent the **instantaneous per-drive-motor DC supply ceiling** currently enforced by hardware (accounting for any asynchronous bus-update latency when lowering limits dynamically), never a delayed continuous thermal floor; pair with optional resting/compensated `restingBatteryVoltageVolts`.
- Use meters, seconds, radians, and counterclockwise-positive rotation.
- At activation, supply robot limits (`v_max`, `a_fric`, `a_brake`, `tau_lag`, `delta_dot_eff`, and `DriveMotorModel`), tracking policy, field bounds and keepouts, robot footprint, and module geometry.
- Run pure footprint checks over the route and permitted headings before activation or rebuild.
- Cache the `TickResult` with field-relative `ChassisSpeeds`, noise-free `ChassisAccelerations`, optional per-module `WheelForces`, status, readiness, `GoalPassageStatus` events, and diagnostics.
- Return the cached result from repeated getters without advancing execution.
- **Zero-Deadband 2027 Drivetrain Consumer:** Route `(commandedFieldSpeeds, commandedFieldAccelerations, wheelForces)` to the 2027 drivetrain subsystem's dedicated closed-loop trajectory consumer with **zero command deadband** (human joystick deadbands belong solely to operator input).
- **Command Continuity:** Evolve `lastCommand` inside `TrailblazerState` during continuous TB3 control (avoiding azimuth-cosine-scaled module-target rest deadlock; seeding from `appliedFieldSpeeds` / `measuredFieldSpeeds` only on activation, handoff, or sustained divergence per [02 §3.4](02-velocity-profiler-and-limits.md#34-initial-velocity-selection-and-chatter-elimination)).
- Expose structured telemetry and active limiting factors (`profileLimitingFactor`, `commandLimitingFactor`) in `TickResult` for logging tools.
- Emit applicable runtime events such as `STALLED`, `RELATCH`, `KEEPOUT_MARGIN_WARNING`, and `INFEASIBLE`.

### 3.1 Time and State Injection

- Inject monotonic time and measured state snapshots through the 2027 robot adapter. Pass explicit snapshots and `dt` to the pure calculators.

```java
@FunctionalInterface
public interface TimeSource {
    double getTimestampSeconds(); // Bound on hardware to the 2027 controller monotonic clock
}

public record RobotStateSnapshot(
    double poseTimestampSeconds,                 // Normalized to TimeSource epoch
    Pose2d measuredPose,                         // Blue-origin field frame
    ChassisSpeeds measuredFieldSpeeds,           // Blue-origin field frame
    boolean localizationValid,
    boolean localizationResetOccurred,
    Optional<ChassisSpeeds> appliedFieldSpeeds,
    OptionalDouble swerveSupplyCurrentLimitAmps, // Instantaneous per-drive-motor ceiling
    OptionalDouble restingBatteryVoltageVolts,   // Open-circuit / compensated V_oc
    boolean enabled
) {}

public record TickInput(
    double timestampSeconds,                     // From TimeSource
    RobotStateSnapshot robotState,
    Map<String, TimestampedTarget<Pose2d>> resolvedGoalTargets,
    Map<String, TimestampedTarget<Translation2d>> resolvedAimTargets,
    Optional<Translation2d> driverFieldVelocityMpS // Pre-mapped Blue-origin (vx, vy) for snapAngle, ||v|| <= v_max
) {}

public record TickResult(
    ChassisSpeeds commandedFieldSpeeds,          // Zero-deadband closed-loop target
    ChassisAccelerations commandedFieldAccelerations, // Noise-free (ax, ay, alpha_ref)
    Optional<WheelForces> wheelForces,           // Optional per-module field force vectors (N)
    RuntimeStatus status,                        // RUNNING, AT_GOAL, FAULT, CANCELLED, DISABLED
    boolean infeasible,
    LimitingFactor profileLimitingFactor,
    LimitingFactor commandLimitingFactor,
    Set<Enum<?>> newlyPassedMarkers,
    Map<String, GoalPassageStatus> goalStatuses
) {}
```

## 4. Stops and Markers

- Determine stop completion from measured position, linear and angular speed, heading readiness, and dwell.
- Release intermediate stops explicitly.
- Record each marker as passed once per activation, in the order defined by the plan. Once recorded, `passedMarker(marker)` returns `true`.
- For `Goal.through`, record the marker as passed when corridor-gated progress reaches or exceeds the goal anchor ($s_{high\_water} \ge s_{anchor}$) and the robot's closest approach to the goal is within the capture radius $d_{capture}$. Record `PASSED_WITHIN_TOLERANCE` if the closest approach is within the goal tolerance, or `PASSED_OUTSIDE_TOLERANCE` if it is outside the tolerance but still within the capture radius. See [01 §6](01-path-geometry-and-kinematics.md#6-anchors-geometry-generations-and-recovery) for the passage and recovery rules.
- For `Goal.finishThrough`, record the terminal marker as passed when the robot passes the anchor within the capture radius.
- For `Goal.stop`, record the marker as passed only after the stop completion criteria are met.
- Preserve which markers have passed and their recorded `GoalPassageStatus` values across path rebuilds.

## 5. Lifecycle and Faults

- Start, update, and cancel explicitly; support mid-plan resumption via `activate(AutoSegment plan, String startAtGoalId)`.
  - Validate that `startAtGoalId` exists, build the horizon from current pose through the next terminal goal via `startAtGoalId`, mark all strictly preceding goals as skipped/completed, **pre-populate their markers as passed** (`passedMarker == true`), mark any `HARD` heading gates strictly preceding `startAtGoalId` as already passed, and inherit the latest `HeadingKeyframe` target angle as `SOFT` if none activates immediately.
- Latch completion until restart.
- On cancellation, hand off through a seeded controller or use a controlled stop.
- Use finite pose and velocity immediately at startup and during tracking. Report old, unready, or mistimed odometry through telemetry. Fault on inputs that prevent finite motion calculations.
- Report faults and timeouts as failure.
- The selected auto **must** stage its initial drivetrain demand during disabled autonomous preparation, before its first segment. Physical output stays inhibited until enable. Refresh staged requests on selection or alliance changes. Issue later requests at their authored steps or markers.
- For the 2026 trench autos, stage the initial swerve demand while disabled. On the first enabled loop, the drive step requests deploy and shooter hood homing through the existing RobotManager APIs and the planner issues its first live drive command.
- Build paths and return the first command in the same enabled update. Apply the existing planner latency target of `min(1.0 ms, 20% of the nominal loop period)` to starts, replans, and tracking. Keep the enabled loop free of blocking waits and I/O. Disabled planner warmup is optional.
- Require reactivation before resuming after disable.

---

## 6. Teleop Assist API

```java
public class Trailblazer {
    public void driveTo(Pose2d targetPose, MotionLimits profile);
    public void snapAngle(Rotation2d targetHeading, MotionLimits profile);
    public void cancel();
}
```

### 6.1 Full Alignment

- Latch the target field pose on activation.
- Build a single-segment plan from the current pose to `Goal.stop` with a `HARD` heading gate.
- Use default completion tolerances of `0.03 m` position error, `2°` heading error, and `0.1 m/s` linear speed.
- Apply angular-speed and dwell requirements from [stop completion](#4-stops-and-markers).

### 6.2 Rotation Snap

- Latch the absolute field heading on activation.
- Use the shortest turn direction and the [bounded heading reference and angular controller](03-heading-and-rotation.md).
- Accept the operator-input layer's alliance-oriented Blue-origin field-relative desired translational velocity `driverFieldVelocityMpS` (bounded to $\|\vec{v}_{driver}^{field}\| \le v_{max}$ of the active `MotionLimits`) inside `TickInput`, pass its magnitude to `HeadingPlanner` for $\rho(v)$ budget blending ([03 §5.4](03-heading-and-rotation.md#5-swerve-wheel-speed-friction--spatial-budget-allocation)), and project jointly with $\omega_{cmd}$ in the 7-set 2D limiter.
- Complete when measured heading error and angular speed stay within the configured tolerances for the configured dwell.
- Omit position goals, position and linear-speed completion checks, and wheel lock.
- Return rotation to manual control through the seeded handoff on completion or cancellation.

### 6.3 Activation and Ownership

- Start each assist on explicit activation. Periodic updates and held buttons do not restart it.
- Replace an active assist through a seeded handoff on new activation.
- Latch completion until the next activation.
- Route manual commands for axes the assist does not own.
- Release assist ownership on completion, cancellation, timeout, fault, or disable.

| Operation | Translation owner | Rotation owner | Driver cancellation |
| --- | --- | --- | --- |
| Full alignment | Trailblazer 3 | Trailblazer 3 | Translation or rotation intervention cancels the whole alignment |
| Rotation snap | Driver | Trailblazer 3 | Rotation intervention cancels the snap; translation input does not |

---

## 7. Driver Preemption & Continuous Velocity Handoff

### 7.1 Preemption Detection (Decoupled Translation & Rotation)

- Check translation and rotation sticks separately in robot integration.
- Use a configured deadband. The example uses `0.15`.
- Apply the cancellation rules in §6.3 before selecting the current tick's drivetrain command.

```java
public boolean isTranslationIntervention() {
    return Math.hypot(driverController.getLeftX(), driverController.getLeftY()) > 0.15;
}

public boolean isRotationIntervention() {
    return Math.abs(driverController.getRightX()) > 0.15;
}

public boolean isDriverIntervention() {
    return isTranslationIntervention() || isRotationIntervention();
}
```

### 7.2 Continuous Velocity Handoff

Seeding the receiving limiter keeps the velocity command from jumping when control changes hands.

- Seed the receiving limiter on manual-to-assist and assist-to-manual transitions. Use the last applied field-relative velocity, falling back to measured velocity.
- Seed the heading reference with measured heading and the last applied angular velocity. Fall back to measured angular velocity.
- Select one command source per axis. During a snap, pair the pre-mapped Blue-origin `driverFieldVelocityMpS` (bounded to active `v_max`) from `TickInput` with assist rotation.
- Combine translation and rotation each tick before applying the [drivetrain limits](02-velocity-profiler-and-limits.md).

---

## 8. Resolved Plan Inputs

- Evaluate each autonomous target supplier once per tick.
- Require timestamps (normalized to `TimeSource`) and validity for dynamic targets. Pass resolved values to the calculators.
- Handle transient target dropouts using each dynamic goal's configured `TargetLossPolicy` (`sealed interface`):
  - `HoldLastValid(double graceSeconds)`: latch the last valid resolved pose while `tickTime - lastValidTargetTime <= graceSeconds`, then transition to fallback or `ControlledStop`.
  - `FallbackNominal(Pose2d nominalPose)`: fall back to the static nominal pose on stale/lost input.
  - `ControlledStop()`: brake to a controlled stop and require explicit reactivation on reacquisition.

---

## 9. Teleop Limits & Watchdogs

- **Timeout.** Cancel unfinished `driveTo` and `snapAngle` operations at the configured timeout (default `3.0 s`). Report failure and return control through the seeded limiter.
- **Keepouts.** For full alignment, check the swept robot footprint along the entire path against field bounds and keepouts. Reject blocked paths with a diagnostic and leave alternate routing to the caller.
- **Wheel lock.** After successful `driveTo` completion (`atGoal() == true`), swerve may apply the configured X-Stop while the driver requests no manual motion. Release the lock on driver input through the normal teleop handoff.
