# Writing TB3 autos

## Coordinates

- Use `CompAutoField.fromBlue(x, y, degrees)` or `fromRed(x, y, degrees)` in any auto.
- Both return a pose in the blue authoring frame. `TrailblazerAuto` converts it to the active alliance.
- Distances are meters. Headings are counterclockwise from field +X.
- `CompAutoField.pose(x, y, degrees)` remains a blue-frame shortcut.
- For a position without heading, use `pose.getTranslation()`.

```java
import static frc.robot.config.CompAutoField.fromRed;

static final Pose2d START = fromRed(12.1, 7.6, -90);
static final AutoSegment SEGMENT =
    Trailblazer.segment("example")
        .through(fromRed(10.089, 7.0, -90), 0.25, 15)
        .mark(Marker.START_INTAKE)
        .stopAt(fromRed(9.0, 6.0, 0))
        .build();
```

## State machine

- Extend `TrailblazerAuto<State>` and define an `enum State`.
- Pass the initial state, start pose, context, and ordered preview segments to `super(...)`.
- For an auto without a fixed start, use `super(initialState, context)`.
- `collectInputs`: read sensors needed by the auto.
- `getNextState`: choose transitions with `atGoal()`, `passedMarker(...)`, and `timeout(seconds)`.
- `afterTransition`: start segments with `drive(segment)` and run one-time actions.
- `whileInState`: send repeated mechanism requests.
- `stopDriving()` cancels the follower.
- Register the auto in `AutoSelection` with `MY_AUTO("My Auto", MyAuto::new)`.

## Waypoints

| Call | Use |
|---|---|
| `through(x, y)` | Pass a position; default corner tolerance is 0.25 m |
| `through(pose, meters)` | Pass near a pose with that corner tolerance |
| `through(pose, meters, degrees)` | Also set heading passage tolerance |
| `stopAt(pose)` | Stop and settle at a pose |
| `stopAt(point, Goal::faceHub)` | Stop with the rear shooter facing the Hub |
| `finishThrough(point, speed)` | Finish while moving at that speed in m/s |
| `mark(Marker.NAME)` | Attach an event to the preceding waypoint |

- A pose sets heading. A translation sets position only.
- A corner tolerance allows rounding; it does not move the authored waypoint.
- A callback can set `faceDegrees`, `facePoint`, `faceTravel`, `clockwise`, or `counterClockwise`.
- Use `g -> g.faceDegrees(90).hardGate()` to require heading readiness before passage.
- Set segment caps with `maxSpeed`, `maxAcceleration`, `maxBraking`, or `withLimits`.
- Keep bump preparation, tracker calls, and shooting transitions in the auto.

## Tuning

Edit `shared/src/main/java/com/team581/trailblazer3/config/TrailblazerConfig.java`.

- `COMP`, `OFFSEASON`, `NEW_MEMBER`: traction in m/s², braking in m/s², response lag in seconds, heading P, and heading D.
- `headingP`, `headingD` belong to each robot's `EmpiricalDriveCalibration`; tune tracking and damping there.
- `HEADING_RESPONSE_MULTIPLIER`: lag-based gain used outside spatial heading tracking, capped by the robot's `headingP`.
- `TRANSLATION_RESPONSE_MULTIPLIER`, `MIN_TRANSLATION_RESPONSE_SECONDS`: translation correction response; effective P is `1 / max(minimum, multiplier * (lag + period / 2))`.
- Measure traction, braking, and lag on carpet first. If tracking errors remain, check heading gains and translation response there.
- `deriveLimits(...)` in the config computes speed, acceleration, rotation, and steering limits from the robot specs.
- See [parameter meanings and derivations](../shared/src/main/java/com/team581/trailblazer3/planning/07-empirical-calibration-routine.md).
- Robot dimensions and motor data stay in `CompTrailblazer`; field keepouts live in `trailblazer3/config/Field2026`.

## Compare changes

```sh
python3 tools/compare_tb3.py
python3 tools/compare_tb3.py --auto LeftTrenchAuto --auto RightTrenchAuto
python3 tools/compare_tb3.py --ref main --robot 581 --output /tmp/tb3-times.json
./gradlew :shared:test :comp-bot:test --tests 'frc.robot.autos.*'
```

- Requires Python 3.12+ and the project's Java 21 JDK.
- Default: compare every `TrailblazerAuto` subclass, on both alliances, against `HEAD` for robot 581.
- Repeat `--auto` or `--robot` to choose cases.
- Uses the VS Code preview worker with seed 581, one JVM per revision, and cached branch builds.
- Builds both revisions, so edits to shared follower/config code are included.
- Shows drive time and its delta. Shooting waits and bump sensor timing are not simulated by the preview.
- Also compares trajectories, paths, waypoints, markers, headings, and diagnostics; excludes source locations and build timings.
- Exit code is nonzero for changed output or a failed preview. Check the bump/state machine tests too.
- Cached branch copies are under `.gradle/trailblazer-compare`; delete that directory to clear them.

## On the robot

- The follower starts from measured pose. The expected start pose is advisory.
- `Autos/AtExpectedStartPose`: within 0.25 m and 15° of the expected start.
- `Autos/Step`: auto state; `Trailblazer3/Status` and `Diagnostic`: follower state and issues.
- `Autos/PreviewPath`, `Trailblazer3/PlannedPath`, `Trailblazer3/TargetPose`: route and target.
- Use `:comp-bot:simulateJava` for interactive simulation.
