# Integration testing

Integration tests check how robot code works together. The offseason bot currently supports
a straight-line diagnostic in full simulation or on the robot.
Comp and new member bots do not currently include this testing setup.

Use Java 21 and run commands from the repository's root folder.
On Windows, use `gradlew.bat` instead of `./gradlew`.

## Full sim

Start the offseason robot application with the simulator window:

```sh
./gradlew :offseason-bot:simulateJava
```

To run the straight-line diagnostic:

1. While disabled, select `STRAIGHT_LINE` in the connected dashboard's `Tests/SelectedTest` chooser.
2. Enable **Teleop** in the simulator, then set `/DSOptions/TestMode` to `true`.
   The option defaults to `false`; the selected test starts when enabled teleop and `TestMode` are both active.
3. Watch `Tests/Regression/Status` and `Tests/Reason` in the dashboard or logs for the result.
   Disable to select another test or run the same test again. Disabling or turning off `TestMode` during a run aborts it.

Like `AutoChooser`, the test chooser uses WPILib’s `SendableChooser`, published through
`SmartDashboard.putData`. Auto selection is at `Autos/SelectedAuto`; test selection is at
`Tests/SelectedTest`. Selection is read while disabled, matching `Autos.getNextState()`. Each activation creates a fresh test. Toggle `TestMode` off and on to rerun while enabled.
Completed results remain visible until the next run. Selecting `NONE` leaves normal teleop operation active.
Tests command the shared Trailblazer; Swerve uses its normal field-centric drive source.
Normal controller bindings remain active.
The diagnostic reports through dashboard and log values.
Straight-line distances, constraints, and tolerances are constants at the top of
`StraightLineTest.java`.

## On robot

Deploy the offseason application to the intended robot:

```sh
./gradlew :offseason-bot:deploy
```

1. With the robot disabled, allow room for the test’s driving distance and keep the path clear.
2. In the connected dashboard, select `STRAIGHT_LINE` as described under **Full sim**.
3. Enable **Teleop** in Driver Station, then set `/DSOptions/TestMode` to `true`.
   Watch the robot and disable or turn off `TestMode` to abort if needed.
4. Read `Tests/Regression/Status` and `Tests/Reason`, save the robot log, and measure the final
   position independently. Disable before another attempt; select `NONE` when finished.

The diagnostic writes to the normal robot WPILOG through DogLog, including its configuration,
motion samples, and result. The same analysis command accepts a downloaded robot log:

```sh
uv run tools/analyze_straight_line.py /path/to/FRC_robot_log.wpilog
```

A passing straight-line diagnostic checks reaching the goal and settling. It does not verify
acceleration limits or replace checking the robot's physical motion.

## Check test results

- **CI:** Open the branch's latest **CI** run in GitHub Actions. Check **Run tests** for unit tests.
  Open a failed job to read its test failures.
- **Local unit tests:** After `./gradlew test`, open `<project>/build/reports/tests/test/index.html`
  for each project you want to check.
- **Full sim or on robot:** Check `Tests/Regression/Status` for `PASSED`, `FAILED`, or `ABORTED`,
  and `Tests/Reason` for an explanation. These values are in the dashboard and robot logs.

## Add a robot test

Implement a state machine extending `RobotTest` in the `frc.robot.testing` package. Like the existing
autos, its public constructor takes `RobotManager` and `Trailblazer`. The test owns its mechanism
requests, sensor reads, DogLog telemetry, outcome checks, and abort cleanup. Return a `TestResult`
with `RUNNING`, `PASSED`, `FAILED`, or `ABORTED` and a reason. Checks belong in the test; the optional analysis script only reports
measurements and plots.

Add the test's selection and construction case in `TestSelection`. That enum is the only registration list
and the dashboard chooser enumerates it. Swerve uses its existing simulation thread and normal
drive requests. Add any new subsystem simulation support to the regular robot simulation.

```mermaid
flowchart LR
    Registry[TestSelection] --> Chooser[Dashboard chooser]
    Chooser --> Manager[TestManager]
    Manager --> Test[Test state machine]
    Test --> Trailblazer
    Trailblazer --> Drive[Drive source]
```

The normal `test` task runs `TestManagerTest` for lifecycle behavior and `StraightLineTestTest`
for controlled sensor and outcome checks.
