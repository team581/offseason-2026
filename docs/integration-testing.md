# Integration testing

Integration tests check how robot code works together. The offseason bot currently supports
headless simulation checks and a straight-line diagnostic in full simulation or on the robot.
Comp and new member bots do not currently include this testing setup.

Use Java 21 and run commands from the repository's root folder.
On Windows, use `gradlew.bat` instead of `./gradlew`.

## Headless sim

Run simulation tests without a simulator window or physical robot. They can cover straight-line
driving, controller bindings, test manager behavior, etc.

```sh
./gradlew headlessTest
```

Gradle prints the test results and ends with `BUILD SUCCESSFUL` if all tests pass.
A failed test makes the command fail.

To run only the straight-line tests:

```sh
./gradlew :offseason-bot:straightLineTest
```

To change the straight-line distance (meters) and maximum speed (meters per second):

```sh
./gradlew :offseason-bot:straightLineTest -Psim.distance=3 -Psim.maxVelocity=1.5
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
2. Set values under `Tests/StraightLine/Config/`. Defaults include `distance=2` meters,
   `maxVelocity=1` meter per second, and `maxAcceleration=0.75` meters per second squared.
3. Enable **Test** mode in the simulator. The diagnostic runs once per enable.
4. Watch `Tests/Regression/Status` and `Tests/Reason` in the dashboard or logs for the result.
   Disable before changing settings or running again. Disabling during a run aborts it.

Normal controller bindings are suppressed during Test mode. Configuration is captured when
Test mode is enabled; edits during a run apply the next time you enable.
The interactive diagnostic reports through dashboard/log values rather than the headless HTML report.

## On robot

Deploy the offseason application to the intended robot:

```sh
./gradlew :offseason-bot:deploy
```

1. With the robot disabled, allow room for the configured driving distance and keep the path clear.
2. In the connected dashboard, select `STRAIGHT_LINE` and configure it as described under **Full sim**.
3. Enable **Test** mode in Driver Station. Watch the robot and disable to abort if needed.
4. Read `Tests/Regression/Status` and `Tests/Reason`, save the robot log, and measure the final
   position independently. Disable before another attempt; select `NONE` when finished.

A passing straight-line diagnostic checks reaching the goal and settling. It does not verify
acceleration limits or replace checking the robot's physical motion.
