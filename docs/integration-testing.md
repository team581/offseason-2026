# Integration testing

Integration tests check how robot code works together. The offseason bot currently supports
headless simulation checks and a straight-line diagnostic in full simulation or on the robot.
Comp and new member bots do not currently include this testing setup.

Use Java 21 and run commands from the repository's root folder.
On Windows, use `gradlew.bat` instead of `./gradlew`.

## Headless sim

Run the regular robot application without a simulator window or physical robot. The headless
runner uses the normal robot lifecycle, scheduler, controller bindings, and subsystem simulations
to run the same registered tests used with the `TestMode` DS option. Test manager behavior and controlled
straight-line failure checks also run with the normal unit tests.

```sh
./gradlew headlessTest
```

Gradle prints the test results and ends with `BUILD SUCCESSFUL` if all tests pass.
A failed test makes the command fail.

To run only the straight-line tests:

```sh
./gradlew :offseason-bot:straightLineTest
```

Straight-line distances, constraints, and tolerances are constants at the top of
`StraightLineRoutine.java`. The robot and headless simulation use those same values.
To change the simulation environment:

```sh
./gradlew :offseason-bot:straightLineTest -Psim.batteryVoltage=10 -Psim.startHeading=90
```

After `headlessTest`, open these files:

- `offseason-bot/build/reports/tests/headlessTest/index.html` — test results and failure details.
- `offseason-bot/build/reports/headlessTest/motion/summary.md` — straight-line measurements.
- `offseason-bot/build/reports/headlessTest/motion/` — CSV motion data.

For `straightLineTest`, replace `headlessTest` with `straightLineTest` in those paths.
Reports are overwritten on each run.

CI runs the suite on every push in the **Headless tests** job. Download its **headless-results**
artifact for reports and logs. Normal `test`, `build`, and deployment commands do not run this suite.

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

Selection is read while disabled, matching auto selection. Each activation creates a fresh test. Toggle `TestMode` off and on to rerun while enabled.
`TestManager.runRequest()` can also restart the selected test while `TestMode` is active in enabled teleop.
Tests command the shared Trailblazer; Swerve uses its normal field-centric drive source.
Normal controller bindings remain active.
The interactive diagnostic reports through dashboard/log values rather than the headless HTML report.

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

A passing straight-line diagnostic checks reaching the goal and settling. It does not verify
acceleration limits or replace checking the robot's physical motion.

## Check test results

- **CI:** Open the branch's latest **CI** run in GitHub Actions. Check **Run tests** for unit tests
  and **Headless tests** for simulation tests. Open a failed job to read its test failures. The run's **Summary** includes straight-line
  measurements. Under **Artifacts**, download and unzip **headless-results**, then open
  `index.html` under `reports/tests/headlessTest/` for the test report.
- **Local headless tests:** Check the terminal for failed test names, then open
  `offseason-bot/build/reports/tests/headlessTest/index.html` in a browser for details.
  Read `offseason-bot/build/reports/headlessTest/motion/summary.md` for straight-line measurements.
  For `straightLineTest`, replace `headlessTest` with `straightLineTest` in these paths.
- **Local unit tests:** After `./gradlew test`, open `<project>/build/reports/tests/test/index.html`
  for each project you want to check.
- **Full sim or on robot:** Check `Tests/Regression/Status` for `PASSED`, `FAILED`, or `ABORTED`,
  and `Tests/Reason` for an explanation. These values are in the dashboard and robot logs;
  interactive runs do not produce the headless HTML report.

## Add a robot test

Implement a state machine extending `TestManager.Test`, then register its factory in
`TestSelection`, just like `AutoSelection`. That enum is the only registration list: the dashboard
chooser and parameterized headless runner both enumerate it. No separate simulation test is needed.
The runner constructs `Robot`, selects through the dashboard chooser, enables teleop and `TestMode`, and
checks the result returned by the test. Swerve uses its existing simulation thread and normal
drive requests. Add any new subsystem simulation support to the regular robot simulation so
interactive and headless runs share it.

```mermaid
flowchart LR
    Registry[TestSelection] --> Chooser[Dashboard chooser]
    Registry --> Runner[Headless runner]
    Chooser --> Manager[TestManager]
    Runner --> Robot[Robot lifecycle]
    Robot --> Chooser
    Manager --> Test[Test state machine]
    Test --> Trailblazer
    Trailblazer --> Drive[Drive source]
```

The normal `test` task runs the controlled checks. `headlessTest` uses the same test source set,
selecting the parameterized runner with a JUnit tag. `straightLineTest` filters that runner to
`STRAIGHT_LINE`.
