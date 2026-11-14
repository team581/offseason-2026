# Trailblazer 2 Design Specification: Runtime, Robot Integration, and Teleop Assist

**Document ID:** TB2-SPEC-04  
**Status:** Implementation design
**Author:** Team 581 Technical Staff  
**Applies To:** `com.team581.trailblazer2.runtime`, `TimeSource`, `RobotStateSource`, `TrailblazerDriveSource`, `TeleopAssist`, `Swerve`, `DriverOverride`  

---

## 1. Overview

Runtime owns state, executes one planner update per tick, and manages activation, completion, and faults. Robot integration supplies measurements and routes commands to the drivetrain. The same planner supports autonomous routines and teleop driver assistance:
1. **Single-Button Drive-to-Pose (`driveTo`):** Drive to a field pose (Reef branch, Subwoofer, Trench, Climb) on the fly without pre-generating paths.
2. **Dynamic Vision Tracking:** Align to camera-detected game pieces or AprilTags updated at 50 Hz.
3. **Instant Driver Override:** Touching the sticks beyond configured deadband immediately hands control back to the driver through a seeded motion limiter.
4. **Shared Physical Limits:** Teleop assists obey the exact same `DrivetrainLimits` and 2D vector acceleration bounds as auto.

---

## 2. Persistent State

- Runtime owns motion state and passes it to pure calculators.
- Runtime also owns activation and goal IDs, fired markers, applied-command history, target filters, and timeout and dwell timers.
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

Robot integration injects time and measured state; calculators receive explicit values and `dt`. Example adapter interfaces:

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

- **On Robot:** `TimeSource` binds to `RobotController.getFPGATime() / 1e6`.

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
    public void driveTo(Supplier<Pose2d> targetSupplier, LimitsProfile profile);
    public void driveThrough(List<Pose2d> waypoints, Pose2d finalStop, LimitsProfile profile);
    public void cancel();
}
```

When `driveTo(targetPose, profile)` is called, Trailblazer creates a 1-segment plan from the robot's current pose to a `Goal.stop` target (default finish tolerance: `0.03 m`, `2°`, `0.1 m/s`) with a `HARD` heading gate. Tolerances also include angular speed and dwell ([stop completion](#4-stops-and-markers)); `driveTo` starts once, and periodic updates do not restart it.

---

## 7. Driver Preemption & Continuous Velocity Handoff

### 7.1 Preemption Detection (Decoupled Translation & Rotation)
Translation and rotation stick deflections are checked separately in robot integration. The `0.15` below is an example controller deadband, not a TB2 constant. Cancelling an axis required by a full-path hard gate cancels that path; independent auto-aim may remain active when explicitly selected:

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
Seed the receiving limiter with the last applied field-relative velocity, or measured velocity if unavailable. Apply this in both directions: manual-to-assist and assist-to-manual. Compose translation and rotation before applying the combined limits in [02](02-velocity-profiler-and-limits.md); only one controller owns each axis.

---

## 8. Dynamic Vision Target Filtering

Runtime evaluates each target supplier once per tick and owns filter history; math receives resolved values. Dynamic targets also need timestamps and validity: stale/lost targets trigger controlled stopping, with explicit reactivation on reacquisition. To reduce steering jitter:
1. **Exponential Low-Pass Filter ($\tau = 0.10\text{ s}$):**
   $$\mathbf{p}_{target, filtered}(t) = \alpha \mathbf{p}_{target, vision}(t) + (1 - \alpha) \mathbf{p}_{target, filtered}(t - \Delta t), \quad \alpha = \frac{\Delta t}{\tau + \Delta t}$$
2. **Moving Target Feedforward:** If tracking a moving target, $\vec{v}_{des} = v_{profile} \hat{t}_{eval} + \vec{v}_{target\_ff} + v_{xt} \hat{n}(s)$, bounded by the final speed, module, and acceleration limits in [02](02-velocity-profiler-and-limits.md).

---

## 9. Semi-Autonomous Assisted Modes

Trailblazer 2 enables hybrid teleoperated control modes where one degree of freedom is automated while the other remains under driver manual control:

| Assist Mode | Translation Authority | Rotation Authority | Use Case |
| :--- | :--- | :--- | :--- |
| **Full Alignment** | Trailblazer 2 (`driveTo`) | Trailblazer 2 (`hard` gate) | Hands-free docking to Reef, Subwoofer, or Climb |
| **Driver Translation + Auto Aim** | Manual Driver Left Stick | Trailblazer 2 (`facePoint(Hub)`) | Shooting on the fly; driver evades defense while robot aims |
| **Auto Line + Driver Speed** | Trailblazer 2 (latched trench line) | Manual / Fixed Heading | Confined trench transit; driver controls throttle, robot steers |
| **Bump Crossing Assist** | Trailblazer 2 (robot-validated speed cap) | Trailblazer 2 (`hold` heading) | Season-specific terrain assist, subject to robot testing |

---

## 10. Teleop Limits & Watchdogs

1. **Maximum Execution Timeout:** Any `driveTo` teleop command has an internal timeout (default: $3.0$ seconds). If uncompleted, it automatically cancels to prevent runaway behavior.
2. **Field Keepout Rejection:** Check the swept robot footprint along the connecting path, not just the target pose, against supplied field bounds and keepouts. Reject blocked paths with a diagnostic; TB2 does not route around obstacles.
3. **Wheel Lock on Arrival:** Upon successfully completing a `driveTo` (`atGoal() == true`), the swerve subsystem can be configured to command an "X-Stop" module lock to resist defensive nudges.
