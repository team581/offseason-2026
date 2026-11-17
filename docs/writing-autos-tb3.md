# Writing autonomous routines with Trailblazer 3

This guide explains how to write, register, and test a new autonomous routine in `comp-bot`.

## Three golden rules

1. **Always write coordinates in Blue alliance.**
   Every point, pose, heading, path, and keepout obstacle is authored from the Blue driver station point of view (`x = 0` at the Blue alliance wall, `+X` pointing downfield, `0°` facing `+X` away from the Blue wall, `180°` facing the Blue wall). `TrailblazerAuto` and `CompAutoField` automatically flip everything for Red alliance.
2. **Write each auto as a Team 581 `StateMachine<S>`.**
   Define an `enum State` for your auto phases and implement standard `afterTransition`, `getNextState`, and `whileInState` methods on `TrailblazerAuto<State>`, matching subsystems and in-season autos.
3. **You do not need ruler-perfect placement on the carpet.**
   If you place the robot 15 cm off the tape mark, Trailblazer 3 replans live from the robot's actual pose as soon as auto starts. Before enabling, `Autos/AtExpectedStartPose` on the dashboard gives drivers a friendly sanity check without ever blocking autonomous.

## Anatomy of a simple auto

Here is a complete real auto (`comp-bot/src/main/java/frc/robot/autos/LeftTrenchAuto.java`):

```java
package frc.robot.autos;

import com.team581.trailblazer3.model.AutoSegment;
import com.team581.trailblazer3.runtime.Trailblazer;
import frc.robot.config.CompAutoField;

/** Drives straight out of the left (depot-side) trench toward the alliance wall. */
final class LeftTrenchAuto extends TrailblazerAuto<LeftTrenchAuto.State> {
  enum State {
    LEAVE_TRENCH,
    DONE
  }

  private static final double TRAVEL_METERS = 2.0;

  static final AutoSegment LEAVE_TRENCH_SEGMENT =
      Trailblazer.segment("leave-left-trench")
          .stopAt(CompAutoField.offset(CompAutoField.LEFT_TRENCH_START, -TRAVEL_METERS, 0.0))
          .build();

  LeftTrenchAuto(AutoContext context) {
    super(State.LEAVE_TRENCH, CompAutoField.LEFT_TRENCH_START, context, LEAVE_TRENCH_SEGMENT);
  }

  @Override
  protected void afterTransition(State newState) {
    switch (newState) {
      case LEAVE_TRENCH -> {
        drive(LEAVE_TRENCH_SEGMENT);
        AutoActions.homeSubsystems(robot);
      }
      case DONE -> stopDriving();
    }
  }

  @Override
  protected State getNextState(State currentState) {
    return currentState == State.LEAVE_TRENCH && atGoal() ? State.DONE : currentState;
  }
}
```

How it works:

- `super(State.LEAVE_TRENCH, CompAutoField.LEFT_TRENCH_START, context, LEAVE_TRENCH_SEGMENT)` sets the initial state, tells the dashboard where to place the robot on the carpet, and registers preview segments for `Autos/PreviewPath` and disabled wheel pre-alignment. If an auto has no fixed start pose (like `DoNothingAuto`), call `super(State.IDLE, context);`.
- `drive(LEAVE_TRENCH_SEGMENT)` flips the Blue segment to the active alliance and activates Trailblazer with `CompAutoField.CLEARANCE` (use `driveWithoutClearance(segment)` to opt out).
- `atGoal()`, `passedMarker(marker)`, `timeout(seconds)`, and `stopDriving()` work inside standard `getNextState`, `whileInState`, and `afterTransition` hooks.
- `robot` (`RobotManager`) is guaranteed non-null on every `TrailblazerAuto`.

### Shared 2026 mechanism helpers (`AutoActions`)

Reuse common 2026 mechanism requests and completion conditions from `frc.robot.autos.AutoActions` across autos instead of duplicating subsystem calls:

```java
AutoActions.startFirstIntakeCycle(robot);
AutoActions.prepareForBumpShot(robot, SHOOT_POINT);
AutoActions.startScoring(robot);
AutoActions.finishedShooting(this);
AutoActions.startNextCycle(robot);
```

## Building paths with `Trailblazer.segment()`

Each path is a chain of waypoints starting from wherever the robot currently is:

### Waypoints

- `.through(point)` / `.through(x, y)`: Drive through a point using the default `0.25 m` corner blend radius.
- `.through(x, y, blendRadius)` / `.through(pose, blendRadius)`: Override how wide `PathBuilder` rounds that corner (`0.50`–`0.95 m` for fast neutral-zone sweeps). Trailblazer automatically clamps fillets against keepouts and ignores `within` on straight segments.
- `.finishThrough(point, speedMps)`: Drive straight through a point (such as picking up game pieces on the fly) at `speedMps`.
- `.stopAt(pose)` / `.stopAt(x, y, headingDeg)` / `.stopAt(x, y, Goal::faceHub)`: Come to a smooth stop at the target pose or point while facing a fixed angle or the alliance Hub.

### Headings & turn gates

Add a lambda `g -> g...` or method reference (such as `Goal::faceHub` or `Goal::counterClockwise`) as the last argument to any waypoint:

- `g.faceHub()` (or `Goal::faceHub`): Aims the rear-mounted shooter (`FieldUtil.BLUE_HUB_TRANSLATION` with `180°` offset) at the active alliance's Hub (use `g.faceHub(offset)` for a different mechanism angle).
- `g.faceDegrees(180)` / `g.face(rot)`: Turn smoothly toward that angle (SOFT gate — does not pause forward motion).
- `g.clockwise()` / `g.counterClockwise()` (or `Goal::clockwise` / `Goal::counterClockwise`): Force turn direction around the circle (Trailblazer also automatically delays initial rotation until clear of nearby walls/keepouts).
- `g.hardGate()` / `g.faceHardDegrees(90)`: Must finish turning within tolerance before passing this waypoint (HARD gate — great right before entering a tight slot).
- `g.facePoint(CompAutoField.point(4.6, 4.0))`: Aim the front of the robot toward a field point.
- `g.faceTravel()`: Rotate to point the front of the robot along the direction of travel.
- `g.mark(MyMarker.FOO)`: Marks a waypoint so `passedMarker(MyMarker.FOO)` returns `true` in `getNextState` / `whileInState`.

### Speed limits

- `.maxSpeed(2.5)`: Simple speed cap in m/s with comp-bot default acceleration/braking.
- `.withLimits(MotionLimits.maxSpeed(3.0).withMaxAcceleration(3.5).withMaxBraking(4.0))`: Full customization when you need gentler acceleration or braking.

### Useful globals & `CompAutoField` helpers

- `FieldUtil.BLUE_HUB_TRANSLATION` — Shared Blue-alliance Hub `Translation2d` (`HUB_POSE.bluePose().getTranslation()`)
- `CompAutoField.point(x, y)` — `Translation2d` shortcut
- `CompAutoField.pose(x, y, deg)` — `Pose2d` shortcut
- `CompAutoField.offset(basePose, dx, dy)` — shifts a known landmark pose by `(dx, dy)` meters
- `CompAutoField.offsetWithHeading(basePose, dx, dy, deg)` — shifts a pose and overrides heading
- Keepout clearance: `CompAutoField.CLEARANCE` (full-field default)

## Adding your auto to the dashboard

Register one entry in `comp-bot/src/main/java/frc/robot/autos/AutoSelection.java`:

```java
MY_NEW_AUTO("My New Auto", MyNewAuto::new),
```

The Red VS Code preview uses the same `ClampedAutoPoints` feature flag as the robot. With the flag enabled, Red shows the home-field route, including the shifted left-trench leave. With it disabled, Red shows the full-field route. Blue shows the full field. If you change the flag's default in `Point.java`, rebuild with `./gradlew :comp-bot:writePreviewClasspathManifest` and reload the VS Code window to restart the preview worker.

Everything else (`SendableChooser`, telemetry `Autos/Step`, pose preview, Red/Blue flipping, disabled swerve launch wheel pre-alignment, and pre-match diagnostics) works automatically. Built-in empirical calibration autos (`TB3 Calibrate Lag`, `TB3 Calibrate Traction`, and `TB3 Calibrate Braking`) are also registered in `AutoSelection`.

## Testing your auto

### Run headless unit tests

```powershell
.\gradlew.bat :comp-bot:test
```

### Run interactive WPILib / AdvantageScope simulation

```powershell
.\gradlew.bat :comp-bot:simulateJava
```

Watch these DogLog / SmartDashboard keys while testing:

- `Autos/SelectedAutoName` — Currently chosen auto
- `Autos/Step` — Current `State` name (`IDLE` before enable, `DONE` when finished)
- `Autos/AtExpectedStartPose` — `true` if the robot sits within `0.25 m` and `15°` of the starting mark
- `Autos/PreviewPath` & `Trailblazer3/PlannedPath` — Stitched preview path and active segment polyline
- `Trailblazer3/TargetPose` — Live moving target pose
- `Trailblazer3/Status` & `Trailblazer3/Diagnostic` — Live follower state and any flagged issue (`RELATCH`, `KEEPOUT_VIOLATION`, `HARD_GATE_BYPASSED`, etc.)
- `ResetPoseForAuto` (`DSOptions`) — Seeds pose in sim / pre-match auto before the first enable (never resets pose back after auto finishes)
