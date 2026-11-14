# 04 — Runtime Integration and Teleop Assist

**Applies To:** `com.team581.trailblazer2.runtime`, `TimeSource`, `RobotStateSource`, `TrailblazerDriveSource`, `TeleopAssist`, `Swerve`, `DriverOverride`  

---

## 1. Overview

- Runtime owns state, updates once per tick, and manages activation, completion, cancellation, and faults.
- Robot integration supplies measurements, reads driver input, binds buttons, selects the drive source, and routes commands to swerve.
- Teleop uses the same motion calculations and physical limits as auto.
- `driveTo` aligns to a fixed field pose. `snapAngle` rotates to a fixed field heading with driver-controlled translation.

---

## 2. Persistent State

- Runtime owns motion state and passes it to pure calculators.
- Runtime owns activation/goal IDs, the set of markers recorded as passed, applied-command history, and timeout/dwell timers.
- Recovery remaps progress and preserves goal and event identity. See [01 — Geometry](01-path-geometry-and-kinematics.md#6-anchors-geometry-generations-and-recovery).
- Localization supplies validity and reset signals, with vision-noise thresholds owned by localization.

```java
public final class TrailblazerState {
    String activeGoalId;                          // Currently active goal ID in the horizon
    Pose2d horizonOrigin;                         // Fixed origin at activation, stop release, or recovery
    long geometryGeneration;                      // Prevent comparing old and rebuilt arc lengths
    double progressHighWaterMark;                 // Corridor-gated monotonic progress in this generation
    Set<Enum<?>> passedMarkers;                   // Markers recorded as passed across rebuilds
    Map<String, GoalPassageStatus> goalStatuses;  // Recorded passage diagnostics per goal ID
    HardGateState hardGateState;                  // Active hard-gate hold, dwell timer, and open latch
    double headingReferenceRad;                   // Continuous unwrapped heading reference state theta_ref
    double headingReferenceVelocity;              // Angular reference velocity
    ChassisSpeeds lastCommand;                    // Previous loop's emitted velocity vector
    double lastTimestamp;                         // Previous loop timestamp for dynamic dt
}
```

## 3. Tick Contract

- Run one update per control-loop tick, nominally every 20 ms (50 Hz) or 10 ms (100 Hz).
- Supply the plan's goals, heading plan, and zones to the pipeline.
- Capture one coherent `TickInput` with the injected `TimeSource` timestamp, measured field pose and speeds, enabled and localization status, resolved targets, and optional `swerveSupplyCurrentLimitAmps`, `batteryVoltage`, and applied-command telemetry.
- **Clock-Domain Normalization:** Convert all timestamps in `TickInput` (tick time, localization snapshot age, and dynamic target timestamps) into the single `TimeSource` (FPGA seconds) clock domain at the robot-integration boundary; never compare raw Phoenix 6 `Utils.getCurrentTimeSeconds()` directly against `RobotController.getFPGATime() / 1e6`.
- **Power-Limit Hardware Contract:** Include active `PowerManager` supply current when available (noting asynchronous CAN delay and that Phoenix 6 `SupplyCurrentLimit` changes take immediate effect only when `SupplyCurrentLowerLimit` / `SupplyCurrentLowerTime` are configured consistently).
- Use meters, seconds, radians, and counterclockwise-positive rotation.
- At activation, supply robot limits (`v_max`, `a_fric`, `a_brake`, `tau_lag`, `delta_dot_eff`, and `DriveMotorModel`), tracking policy, field bounds and keepouts, robot footprint, and module geometry.
- Run pure footprint checks over the route and permitted headings before activation or rebuild.
- Cache the `TickResult` with field-relative `ChassisSpeeds`, noise-free `ChassisAccelerations` (plus optional per-module wheel-force feedforwards), status, readiness, `GoalPassageStatus` events, and diagnostics.
- Return the cached result from repeated getters without advancing execution.
- Route commands to swerve through a **dedicated zero-deadband closed-loop request** (e.g., Phoenix 6 `SwerveRequest.ApplyFieldSpeeds` with `DriveRequestType.Velocity` and `0.0` linear/rotational deadbands; never reuse teleop `fieldCentric`'s `0.07 m/s` or `0.05 rad/s` deadbands).
- Track `lastCommand` inside `TrailblazerState` during continuous TB2 control (using `applied` / measured speeds only to seed on activation/handoff and reseed on sustained divergence per [02 §3.4](02-velocity-profiler-and-limits.md#34-initial-velocity-selection-and-chatter-elimination)).
- Log diagnostics and the active limiting factor via DogLog.
- Emit applicable runtime events such as `STALLED`, `RELATCH`, `KEEPOUT_MARGIN_WARNING`, and `INFEASIBLE`.

### 3.1 Time and State Injection

- Inject time and measured state through robot integration. Pass explicit values and `dt` to the calculators.

```java
@FunctionalInterface
public interface TimeSource {
    double getTimestampSeconds();
}

public interface RobotStateSource {
    Pose2d getMeasuredPose();
    ChassisSpeeds getMeasuredSpeeds(); // Field-relative
    boolean isEnabled(); // Auto and teleop assists
}
```

- On the robot, bind `TimeSource` to `RobotController.getFPGATime() / 1e6`.

## 4. Stops and Markers

- Determine stop completion from measured position, linear and angular speed, heading readiness, and dwell.
- Release intermediate stops explicitly.
- Record each marker as passed once per activation, in the order defined by the plan. Once recorded, `passedMarker(marker)` returns `true`.
- For `Goal.through`, record the marker as passed when corridor-gated progress reaches or exceeds the goal anchor ($s_{high\_water} \ge s_{anchor}$) and the robot's closest approach to the goal is within the capture radius $d_{capture}$. Record `PASSED_WITHIN_TOLERANCE` if the closest approach is within the goal tolerance, or `PASSED_OUTSIDE_TOLERANCE` if it is outside the tolerance but still within the capture radius. See [01 §6](01-path-geometry-and-kinematics.md#6-anchors-geometry-generations-and-recovery) for the passage and recovery rules.
- For `Goal.cruiseThrough`, record the terminal marker as passed when the robot passes the anchor within the capture radius.
- For `Goal.stop`, record the marker as passed only after the stop completion criteria are met.
- Preserve which markers have passed and their recorded `GoalPassageStatus` values across path rebuilds.

## 5. Lifecycle and Faults

- Start, update, and cancel explicitly; support mid-plan resumption via `activate(MotionPlan plan, String startAtGoalId)`.
  - Validate that `startAtGoalId` exists, build the horizon from current pose through the next terminal goal via `startAtGoalId`, mark all strictly preceding goals as skipped/completed, **pre-populate their markers as passed** (`passedMarker == true`), and inherit the latest `HeadingKeyframe` preceding `startAtGoalId` if none activates immediately.
- Latch completion until restart.
- On cancellation, hand off through a seeded controller or use a controlled stop.
- Fault on invalid input, stale localization, or invalid or excessive `dt` without advancing events.
- Report faults and timeouts as failure.
- Inhibit output while disabled.
- Require reactivation before resuming after disable.

---

## 6. Teleop Assist API

```java
public class Trailblazer {
    public void driveTo(Pose2d targetPose, LimitsProfile profile);
    public void snapAngle(Rotation2d targetHeading, LimitsProfile profile);
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
- Map operator stick translation into **Blue-origin field-relative closed-loop `ChassisSpeeds`** by rotating by `OperatorPerspective` (`180°` on Red Alliance, `0°` on Blue) and scaling full stick deflection to TB2's active `LimitsProfile` `v_max` (not teleop open-loop `4.75 m/s`) before TB2's 7-set 2D limiter.
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
| Full alignment | Trailblazer 2 | Trailblazer 2 | Translation or rotation intervention cancels the whole alignment |
| Rotation snap | Driver | Trailblazer 2 | Rotation intervention cancels the snap; translation input does not |

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
- Select one command source per axis. During a snap, convert operator sticks to Blue-origin field-relative closed-loop speeds rotated by `OperatorPerspective` and scaled to TB2's active `v_max`, pairing them with assist rotation.
- Combine translation and rotation each tick before applying the [drivetrain limits](02-velocity-profiler-and-limits.md).

---

## 8. Resolved Plan Inputs

- Evaluate each autonomous target supplier once per tick.
- Require timestamps (normalized to `TimeSource`) and validity for dynamic targets. Pass resolved values to the calculators.
- Handle transient target dropouts using each dynamic goal's configured `TargetLossPolicy`:
  - `HOLD_LAST_VALID(graceSeconds)`: latch the last valid resolved pose while `tickTime - lastValidTargetTime <= graceSeconds`, then transition to fallback or `CONTROLLED_STOP`.
  - `FALLBACK_NOMINAL(Pose2d)`: fall back to the static nominal pose on stale/lost input.
  - `CONTROLLED_STOP`: brake to a controlled stop and require explicit reactivation on reacquisition.

---

## 9. Teleop Limits & Watchdogs

- **Timeout.** Cancel unfinished `driveTo` and `snapAngle` operations at the configured timeout (default `3.0 s`). Report failure and return control through the seeded limiter.
- **Keepouts.** For full alignment, check the swept robot footprint along the entire path against field bounds and keepouts. Reject blocked paths with a diagnostic and leave alternate routing to the caller.
- **Wheel lock.** After successful `driveTo` completion (`atGoal() == true`), swerve may apply the configured X-Stop while the driver requests no manual motion. Release the lock on driver input through the normal teleop handoff.
