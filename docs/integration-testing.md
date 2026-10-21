# Integration testing

The offseason bot has a headless test suite and a straight-line diagnostic you can run in
simulation or on the robot. This setup is only available in the offseason application.

Use Java 21 and run commands from the repository's root folder.
On Windows PowerShell, use `.\gradlew.bat` instead of `./gradlew`.

## Headless sim

Run checks for straight-line driving, controller bindings, and test manager behavior without
a simulator window or physical robot:

```sh
./gradlew headlessTest
```

Gradle prints the test results and ends with `BUILD SUCCESSFUL` if all tests pass.

To run the straight-line and test manager checks:

```sh
./gradlew :offseason-bot:straightLineTest
```

To change the straight-line distance (meters) and maximum speed (meters per second):

```sh
./gradlew :offseason-bot:straightLineTest -Psim.distance=3 -Psim.maxVelocity=1.5
```

After `headlessTest`, find the results here:

- `offseason-bot/build/reports/tests/headlessTest/index.html` — test results and failure details.
- `offseason-bot/build/reports/headlessTest/motion/summary.md` — straight-line measurements.
- `offseason-bot/build/reports/headlessTest/motion/` — CSV motion data.

For `straightLineTest`, replace `headlessTest` with `straightLineTest` in those paths.
Reports are overwritten on each run.

CI runs the suite on every push in the **Headless tests** job. Download its **headless-results**
artifact for reports and logs. Run `headlessTest` explicitly; `test`, `build`, and deployment
commands do not run it.

## Full sim

Start the offseason robot application with the simulator window:

```sh
./gradlew :offseason-bot:simulateJava
```

To run the straight-line diagnostic:

1. Connect a dashboard to the simulator. While disabled, select `STRAIGHT_LINE` in
   `SmartDashboard/Tests/SelectedTest`.
2. Set values under `Tests/StraightLine/Config/`. Defaults include `distance=2` meters,
   `maxVelocity=1` meter per second, and `maxAcceleration=0.75` meters per second squared.
3. Select **Test** mode in the simulator and enable. The diagnostic runs once per enable.
4. Watch `Tests/Regression/Status` and `Tests/Reason` in the dashboard or logs for the result.
   Disable before changing settings or running again. Disabling during a run aborts it.

Normal controller buttons do not issue requests during Test mode. Settings are read when you
enable; changes made during a run apply on the next enable. Results appear in the dashboard
and robot logs. Full sim does not create a headless HTML report.

## On robot

Deploy the offseason application to the intended robot:

```sh
./gradlew :offseason-bot:deploy
```

1. With the robot disabled, clear enough space for the configured drive and stopping distance.
2. In the connected dashboard, select `STRAIGHT_LINE` and configure it as described under **Full sim**.
3. Enable **Test** mode in Driver Station. Watch the robot and disable to abort if needed.
4. Read `Tests/Regression/Status` and `Tests/Reason`, save the robot log, and measure the final
   position independently. Disable before another attempt; select `NONE` when finished.

The straight-line diagnostic passes when the robot reaches the goal and settles within
tolerance. Acceleration limits are not checked.
