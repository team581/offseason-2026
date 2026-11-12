# Trailblazer 2 Design Specification: Teleop Integration & Dynamic Assist

**Document ID:** TB2-SPEC-04  
**Status:** Approved for Implementation  
**Author:** Team 581 Technical Staff  
**Applies To:** `TrailblazerDriveSource`, `TeleopAssist`, `Swerve`, `DriverOverride`  

---

## 1. Overview

Trailblazer 2 uses the same planner for both autonomous routines and teleop driver assistance:
1. **Single-Button Drive-to-Pose (`driveTo`):** Drive to a field pose (Reef branch, Subwoofer, Trench, Climb) on the fly without pre-generating paths.
2. **Dynamic Vision Tracking:** Align to camera-detected game pieces or AprilTags updated at 50 Hz.
3. **Instant Driver Override:** Touching the sticks beyond deadband (`0.15`) immediately hands control back to the driver with zero velocity jump.
4. **Shared Physical Limits:** Teleop assists obey the exact same `DrivetrainLimits` and 2D vector acceleration bounds as auto.

---

## 2. Teleop Assist API

```java
public class Trailblazer {
    public void driveTo(Pose2d targetPose, LimitsProfile profile);
    public void driveTo(Supplier<Pose2d> targetSupplier, LimitsProfile profile);
    public void driveThrough(List<Pose2d> waypoints, Pose2d finalStop, LimitsProfile profile);
    public void cancel();
}
```

When `driveTo(targetPose, profile)` is called, Trailblazer creates a 1-segment plan from the robot's current pose to a `Goal.stop` target (default finish tolerance: `0.03 m`, `2°`, `0.1 m/s`) with a `HARD` heading gate.

---

## 3. Driver Preemption & Continuous Velocity Handoff

### 3.1 Preemption Detection (Decoupled Translation & Rotation)
Translation and rotation stick deflections are checked separately so hybrid assist modes (e.g. manual driving + auto-aiming at the Hub) only cancel the axis the driver overrides:

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

### 3.2 Continuous Velocity Handoff
When the driver takes over, `teleopDriveSource.seedVelocity(trailblazer.getLastCommand())` seeds the manual slew-rate limiter with Trailblazer's last commanded velocity $\vec{v}_{last\_command}$ so the robot does not jerk or brake on handoff.

---

## 4. Dynamic Vision Target Filtering

To prevent camera noise from causing steering jitter when following a `Supplier<Pose2d>`:
1. **Exponential Low-Pass Filter ($\tau = 0.10\text{ s}$):**
   $$\mathbf{p}_{target, filtered}(t) = \alpha \mathbf{p}_{target, vision}(t) + (1 - \alpha) \mathbf{p}_{target, filtered}(t - \Delta t), \quad \alpha = \frac{\Delta t}{\tau + \Delta t}$$
2. **Moving Target Feedforward:** If tracking a moving target, $\vec{v}_{des} = v_{profile} \hat{t}_{eval} + \vec{v}_{target\_ff} + v_{xt} \hat{n}(s)$, clamped by `VectorLimiter`.

---

## 5. Semi-Autonomous Assisted Modes

Trailblazer 2 enables hybrid teleoperated control modes where one degree of freedom is automated while the other remains under driver manual control:

| Assist Mode | Translation Authority | Rotation Authority | Use Case |
| :--- | :--- | :--- | :--- |
| **Full Alignment** | Trailblazer 2 (`driveTo`) | Trailblazer 2 (`hard` gate) | Hands-free docking to Reef, Subwoofer, or Climb |
| **Driver Translation + Auto Aim** | Manual Driver Left Stick | Trailblazer 2 (`facePoint(Hub)`) | Shooting on the fly; driver evades defense while robot aims |
| **Auto Line + Driver Speed** | Trailblazer 2 (latched trench line) | Manual / Fixed Heading | Confined trench transit; driver controls throttle, robot steers |
| **Bump Crossing Assist** | Trailblazer 2 (constant 4 m/s zone) | Trailblazer 2 (`hold` heading) | Safe traverse across terrain without driver pitch/roll bounce |

---

## 6. Teleop Safety Guarantees & Watchdogs

1. **Maximum Execution Timeout:** Any `driveTo` teleop command has an internal timeout (default: $3.0$ seconds). If uncompleted, it automatically cancels to prevent runaway behavior.
2. **Field Keepout Rejection:** If a driver accidentally commands a `driveTo` whose target pose falls inside a defined keepout zone (e.g. opposing alliance stage or illegal field perimeter), Trailblazer rejects the command with an audible driver station alert.
3. **Wheel Lock on Arrival:** Upon successfully completing a `driveTo` (`atGoal() == true`), the swerve subsystem can be configured to command an "X-Stop" module lock to resist defensive nudges.
