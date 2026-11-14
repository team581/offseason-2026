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
- Runtime owns activation/goal IDs, fired markers, applied-command history, and timeout/dwell timers.
- Recovery remaps progress and preserves goal and event identity. See [01 — Geometry](01-path-geometry-and-kinematics.md#6-anchors-geometry-generations-and-recovery).
- Localization supplies validity and reset signals, with vision-noise thresholds owned by localization.

```java
public final class TrailblazerState {
    int segmentIndex;                     // Currently active segment in state machine
    int latchedSubSegmentIndex;           // Active waypoint-to-waypoint sub-segment index k
    Translation2d latchedSubSegmentStart; // Origin of sub-segment k (P_k or re-latched robot pose)
    double progressHighWaterMark;         // Monotonic progress within this geometry generation
    long geometryGeneration;             // Prevent comparing old and rebuilt arc lengths
    double headingReferenceRad;           // Continuous unwrapped heading reference state theta_ref
    double headingReferenceVelocity;     // Angular reference velocity
    ChassisSpeeds lastCommand;            // Previous loop's emitted velocity vector
    double lastTimestamp;                 // Previous loop timestamp for dynamic dt
}
```

## 3. Tick Contract

- Run one update per control-loop tick, nominally every 20 ms (50 Hz) or 10 ms (100 Hz).
- Supply the plan's goals, heading plan, and zones to the pipeline.
- Capture one coherent `TickInput` with the injected `TimeSource` timestamp, measured field pose and speeds, enabled and localization status, resolved targets, and optional last-applied command.
- Use meters, seconds, radians, and counterclockwise-positive rotation.
- At activation, supply robot limits (`v_max`, `a_fric`, `a_brake`, and `tau_lag`), tracking policy, field bounds and keepouts, robot footprint, and module geometry.
- Run pure footprint checks over the route and permitted headings before activation or rebuild.
- Cache the `TickResult` with command, status, readiness, events, and diagnostics.
- Return the cached result from repeated getters without advancing execution.
- Send field-relative `ChassisSpeeds` to swerve.
- Log diagnostics and the active limiting factor via DogLog.
- Emit applicable runtime events such as `STALLED`, `RELATCH`, and `INFEASIBLE`.

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
- Fire each marker once per activation, in authored order.
- Preserve fired-marker state across rebuilds.

## 5. Lifecycle and Faults

- Start, update, and cancel explicitly.
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
- Select one command source per axis. During a snap, use driver translation and assist rotation.
- Combine translation and rotation each tick before applying the [drivetrain limits](02-velocity-profiler-and-limits.md).

---

## 8. Resolved Plan Inputs

- Evaluate each autonomous target supplier once per tick.
- Require timestamps and validity for dynamic targets. Pass resolved values to the calculators.
- Use a controlled stop when a target goes stale or is lost. Require explicit reactivation after reacquisition.

---

## 9. Teleop Limits & Watchdogs

- **Timeout.** Cancel unfinished `driveTo` and `snapAngle` operations at the configured timeout (default `3.0 s`). Report failure and return control through the seeded limiter.
- **Keepouts.** For full alignment, check the swept robot footprint along the entire path against field bounds and keepouts. Reject blocked paths with a diagnostic and leave alternate routing to the caller.
- **Wheel lock.** After successful `driveTo` completion (`atGoal() == true`), swerve may apply the configured X-Stop while the driver requests no manual motion. Release the lock on driver input through the normal teleop handoff.
