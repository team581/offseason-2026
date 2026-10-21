# Headless testing

Headless tests run the offseason bot's simulation checks without a simulator window or a
physical robot. They cover straight-line driving, controller bindings, and test manager behavior.
Comp and new member bots do not currently have a headless suite.

## Run the tests

Use Java 21. Open a terminal in the repository's root folder and run:

```sh
./gradlew headlessTest
```

On Windows, use `gradlew.bat` instead of `./gradlew`.
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

## Find the results

After `headlessTest`, open these files:

- `offseason-bot/build/reports/tests/headlessTest/index.html` — test results and failure details.
- `offseason-bot/build/reports/headlessTest/motion/summary.md` — straight-line measurements.
- `offseason-bot/build/reports/headlessTest/motion/` — CSV motion data.

For `straightLineTest`, replace `headlessTest` with `straightLineTest` in those paths.
Reports are overwritten on each run. A passing straight-line test checks reaching the goal and
settling; it does not verify acceleration limits.

CI runs the suite on every push in the **Headless tests** job. Open that job for results or
download its **headless-results** artifact for reports and logs.

Run this command explicitly: normal `test`, `build`, and deployment commands do not run the
headless suite.
