# Writing autonomous routines with Trailblazer 3

This guide explains how to write, register, and test a new autonomous routine in `comp-bot`.

## Three golden rules

1. **Always write coordinates in Blue alliance.**
   Every point, pose, heading, path, and keepout obstacle is authored from the Blue driver station point of view (`x = 0` at the Blue alliance wall, `+X` pointing downfield, `0°` facing `+X` away from the Blue wall, `180°` facing the Blue wall). `TrailblazerAuto` and `CompAutoField` automatically flip everything for Red alliance.
2. **Write steps top-to-bottom.**
   Inside your auto class constructor, you list steps in the exact order they happen: `action(...)`, `drive(...)`, `waitFor(...)`, `waitSeconds(...)`, and `branch(...)`. No boilerplate `enum State`, `switch`, or `getNextState` is needed.
3. **You do not need ruler-perfect placement on the carpet.**
   If you place the robot 15 cm off the tape mark, Trailblazer 3 replans live from the robot's actual pose as soon as auto starts. Before enabling, `Autos/AtExpectedStartPose` on the dashboard gives drivers a friendly sanity check without ever blocking autonomous.

## Anatomy of a simple auto

Here is a complete real auto (`comp-bot/src/main/java/frc/robot/autos/LeftTrenchAuto.java`):

```java
package frc.robot.autos;

import com.team581.trailblazer3.model.AutoSegment;
import com.team581.trailblazer3.model.MotionLimits;
import com.team581.trailblazer3.runtime.Trailblazer;
import frc.robot.config.CompAutoField;

/** Drives straight out of the left (depot-side) trench toward the alliance wall. */
final class LeftTrenchAuto extends TrailblazerAuto {
  private static final double TRAVEL_METERS = 2.0;

  static final AutoSegment LEAVE_TRENCH_SEGMENT =
      Trailblazer.segment("leave-left-trench")
          .withLimits(MotionLimits.robotMaximum())
          .stopAt(CompAutoField.offset(CompAutoField.LEFT_TRENCH_START, -TRAVEL_METERS, 0.0))
          .build();

  LeftTrenchAuto(AutoContext context) {
    super(CompAutoField.LEFT_TRENCH_START, context);
    drive("leave_left_trench", LEAVE_TRENCH_SEGMENT);
  }
}
```

How it works:

- `super(CompAutoField.LEFT_TRENCH_START, context)` tells the dashboard where the drivers should set the robot on the carpet (and lets simulation/pre-match pose reset seed telemetry). If an auto does not have a fixed starting pose (like `DoNothingAuto`), call `super(context);`.
- `drive("leave_left_trench", ...)` activates the path (automatically flipped to Red if needed), automatically checks field/trench bumper keepouts using `CompAutoField.CLEARANCE` (active by default on `CompTrailblazer`; use `.withoutClearance()` to opt out or `.withClearance(...)` to override), automatically pre-points the swerve wheels along the initial launch direction while disabled, and advances to `"DONE"` once Trailblazer reaches the goal. (For match autos that also home mechanisms right away, put an `action("home_mechanisms", () -> { ... })` before `drive(...)`.)

## Step cheat sheet

| Step | What it does | Common modifiers |
| --- | --- | --- |
| `action("name", runnable)` | Runs a one-shot action right away and moves immediately to the next step | — |
| `branch("name", cond, ifTrue)` / `branch("name", cond, ifTrue, ifFalse)` | Evaluates `cond` when reached and expands nested step sequences (`drive`, `waitFor`, `waitSeconds`, `action`, and nested `branch`) in place | — |
| `drive("name", blueSegment)` | Drives along a Blue-frame `AutoSegment` until finished | `.withoutClearance()`, `.withClearance(blueClearance)`, `.onStart(...)`, `.everyLoop(...)`, `.onMarker(marker, runnable)`, `.releaseWhen(cond)`, `.until(cond)`, `.withTimeout(sec)` |
| `waitFor("name", maxSec, condition)` | Stops the drivetrain and waits until `condition` becomes `true` or `maxSec` runs out | `.onStart(...)`, `.everyLoop(...)` |
| `waitSeconds("name", sec)` | Stops the drivetrain and waits a fixed number of seconds | `.onStart(...)`, `.everyLoop(...)` |

Example showing markers, waits, and continuous actions:

```java
enum Marker {
  START_INTAKING,
  DROP_DEPLOY
}

AutoSegment grabBall =
    Trailblazer.segment("grab-ball")
        .maxSpeed(3.0)
        .through(CompAutoField.point(5.5, 1.5), g -> g.within(0.35).mark(Marker.START_INTAKING).faceDegrees(0))
        .finishThrough(CompAutoField.point(7.2, 1.5), 1.8, g -> g.mark(Marker.DROP_DEPLOY))
        .build();

drive("grab_ball", grabBall)
    .onMarker(Marker.DROP_DEPLOY, robot::homeDeployInAutoRequest)
    .everyLoop(() -> robot.intake.intakeRequest())
    .until(() -> robot.hopperManager.hasGamePiece())
    .withTimeout(3.5);

waitFor("wait_for_intake", 1.0, () -> robot.hopperManager.hasGamePiece());
```

## Building paths with `Trailblazer.segment()`

Each path is a chain of waypoints starting from wherever the robot currently is:

### Waypoints

- `.through(point, blendRadius)`: Drive through `(x, y)` smoothly curving toward the next point. Larger `blendRadius` (e.g. `0.25`–`0.50` m) makes wider, faster corners.
- `.through(pose)` / `.through(pose, blendRadius)`: Drive through `pose` while targeting `pose.getRotation()`.
- `.finishThrough(point, speedMps)`: Drive straight through a point (such as when picking up a game piece on the fly) at `speedMps`.
- `.stopAt(pose)` / `.stopAt(x, y, headingDeg)`: Come to a smooth stop at the target pose.
- `.stopAndWait(pose)` (via `.addGoal(Goal.stopAndWait(pose))`): Pause inside a multi-goal segment until `.releaseWhen(...)` becomes true.

### Headings & turn gates

Add a lambda `g -> g...` or method reference (such as `Goal::counterClockwise`) as the last argument to any waypoint:

- `g.faceDegrees(180)` / `g.face(rot)`: Turn smoothly toward that angle (SOFT gate — does not pause forward motion).
- `g.clockwise()` / `g.counterClockwise()` (or `Goal::clockwise` / `Goal::counterClockwise`): Force turn direction around the circle (Trailblazer also automatically delays initial rotation until clear of nearby walls/keepouts).
- `g.hardGate()` / `g.faceHardDegrees(90)`: Must finish turning within tolerance before passing this waypoint (HARD gate — great right before entering a tight slot or shooting).
- `g.facePoint(CompAutoField.point(4.6, 4.0))`: Aim the front of the robot toward a field point (such as the Hub).
- `g.faceTravel()`: Rotate to point the front of the robot along the direction of travel.
- `g.mark(MyMarker.FOO)`: Trigger `.onMarker(MyMarker.FOO, ...)` as the robot passes this waypoint.

### Speed limits

- `.maxSpeed(2.5)`: Simple speed cap in m/s with comp-bot default acceleration/braking.
- `.withLimits(MotionLimits.maxSpeed(3.0).withMaxAcceleration(3.5).withMaxBraking(4.0))`: Full customization when you need gentler acceleration or braking.

### Useful `CompAutoField` helpers

- `CompAutoField.point(x, y)` — `Translation2d` shortcut
- `CompAutoField.pose(x, y, deg)` — `Pose2d` shortcut
- `CompAutoField.offset(basePose, dx, dy)` — shifts a known landmark pose by `(dx, dy)` meters
- `CompAutoField.offsetWithHeading(basePose, dx, dy, deg)` — shifts a pose and overrides heading
- Keepouts: `CompAutoField.CLEARANCE` (full-field default), `CompAutoField.RIGHT_TRENCH_CLEARANCE`, `CompAutoField.LEFT_TRENCH_CLEARANCE`

## Adding your auto to the dashboard

Register one entry in `comp-bot/src/main/java/frc/robot/autos/AutoSelection.java`:

```java
MY_NEW_AUTO("My New Auto", MyNewAuto::new),
```

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
- `Autos/Step` and `Autos/StepIndex` — Which step is running right now (`IDLE` before enable, `DONE` when finished)
- `Autos/AtExpectedStartPose` — `true` if the robot sits within `0.25 m` and `15°` of the starting mark
- `Autos/StartPoseDistanceMeters` / `Autos/StartPoseHeadingErrorDeg` — Exact offset from tape
- `Trailblazer3/Status` & `Trailblazer3/Diagnostic` — Live planner state
- `ResetPoseForAuto` (`DSOptions`) — Seeds pose in sim / pre-match auto before the first enable (never resets pose back after auto finishes)
