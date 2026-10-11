# Test authoring for students and agents

Status: proposal, October 10, 2026. APIs and milestones described here are proposed unless explicitly identified as existing.

[Project overview](../simple-testing-plan.md)

Start with typed Java builders and three copyable templates: request/state, sensor/interlock,
and movement/settling. Avoid an initial YAML language, expression parser, or reflection-based
signal lookup. Java already supplies autocomplete, compiler errors, units/types, and debugging.
Parameter records can make distance, target, and tolerance variants cheap without a new language.

Every test supplies a name, prerequisites, action, observations/checks, deadline, cleanup, and
at most a few useful metrics. Discovery/registration should be handled once by the framework,
including robot selection; adding a scenario must not require editing Gradle or CI.

Proposed authoring shape, not a current API:

```java
scenario("hood-reaches-target")
    .requires(HOOD_HOMED)
    .action(robot::requestHoodTarget)
    .always("valid angle", robot::hoodAngleIsFiniteAndInRange)
    .eventually("settled", robot::hoodWithinTolerance,
        deadlineSeconds, holdSeconds)
    .metric("settleSeconds", SECONDS, robot::hoodSettleTime)
    .cleanup(robot::idleMechanisms);
```

The actual hood uses distance-based requests; its adapter should use the existing API and
observe the resulting target. Specify expected behavior independently wherever practical.
Checking only `atGoal()` or comparing a target to itself can hide the bug being tested.

Provide only a few temporal helpers at first:

- `eventually`: reach a condition before a deadline, optionally hold it continuously.
- `always`: a condition must hold throughout an explicitly named phase.
- `never`: forbidden behavior, such as feeding while not ready.
- `within`: numeric tolerance, with units, wrapped-angle handling, and a finite-value check.

Observe values after the normal sensor/read/control cycle. A missing/stale sample cannot satisfy
a check. Failure output includes phase, signal, measured value, expected limit, and first failure
time. Deadlines and hold times use the adapter clock, not test-method wall time.

Measure authoring success: after the helpers exist, have a student copy a template, add a test,
and run it within 15 minutes. This target assumes an existing action/observation adapter; adding
a brand-new sensor capability is separate framework work.

Include a short local setup/run guide and validate terminal and VS Code test execution on team
software laptops. Document the WPILib Java/toolchain and native
dependency setup, terminal unit/headless/focused commands, VS Code test discovery, and report
locations. A successful authoring trial includes finding and interpreting the result on a team
laptop. Diagnose editor failures separately from Gradle failures; record known fixes in the guide.

Add or update the [coverage register](manual-regression.md) when adding a test. For behavior that
cannot yet be automated, write a manual case with explicit observations and evidence instead.

### Repository-local agent skill

After the API stabilizes, add `.agents/skills/write-robot-test/SKILL.md` with templates and focused
run commands. Its workflow should:

1. Turn the reported bug into a short setup/action/expected-behavior statement.
2. Choose a unit test for pure math, or a scenario for temporal/integrated behavior.
3. Reuse existing actions, observations, and checks; list unsupported inputs explicitly.
4. Choose justified tolerances/deadlines; distinguish software limits from measured tuning limits.
5. Run the focused test and demonstrate it catches the bug, using the old code or a temporary
   intentional defect where practical. Merely weakening an expected value proves too little.
6. Report evidence, applicable environments, and coverage limits. Never loosen thresholds just
   to get a passing result or automatically enable physical hardware.

Humans and agents use the same examples and library. Keep HAL setup, logging, registration,
mode cleanup, and report formatting out of individual test definitions.

## Concrete authoring workflow

Place deployable definitions in `testing/scenarios/`, typed request/read adapters in
`testing/adapters/`, and JUnit wrappers in `src/simTest/java/frc/robot/sim/`. Put pure check and
metric tests in `src/test/java/frc/robot/testing/`. Each definition has a stable case ID, an integer
version, a description, and an immutable parameter record. Increment the version when acceptance
meaning changes; preserve ID across refactors. Parameter presets are separate comparison keys.

Implement these initial templates:

| Template | Phases | Required adapter inputs |
|---|---|---|
| Request/state | prerequisite → request → observe → cancel | Public request/cancel functions, state snapshot, output/request evidence |
| Sensor/interlock | baseline → inject/occlude → observe blocked behavior → restore | Capability-qualified sensor source, readiness gates, transport commands |
| Movement/settling | ready → target step → hold → idle | Target, measured position/speed, limits, source timestamps |

For `hood-reaches-target`, the action adapter calls `ShooterHood.scoreRequest(distanceMeters)`;
the observation adapter reads `getAngle()`. Bind an independently reviewed expected-angle preset
and tolerance to that distance. `ShooterHood/GoalAngle` is useful report evidence but must not be
the sole expected value. If homing or a required target observation is inaccessible, record the
capability gap before promising the scenario in an environment.

A student adding an existing-adapter case performs these steps:

1. Copy a template into `testing/scenarios/`, choose ID/version, and state the bug and expected
   response. Define parameters and validate their finite values/ranges.
2. Select named phases and attach checks to phases. Require each input's freshness budget and
   define deadlines from action time, excluding prerequisite waiting.
3. Add a catalog factory and headless wrapper. The wrapper invokes common fixture/session setup,
   writes evidence in `finally`, then asserts terminal PASS through JUnit.
4. Run the focused class, inspect its report, and demonstrate the intended failure with a known
   bad implementation or temporary defect. Restore temporary defects before finishing.
5. Link source, supported environments, and evidence from the coverage register. Record remaining
   manual checks separately from automated PASS.

Existing commands from the repository root:

```sh
./gradlew :offseason-bot:test --tests frc.robot.testing.DiagnosticSessionTest
./gradlew :offseason-bot:headlessTest --tests frc.robot.sim.StraightLineSimTest
./gradlew :offseason-bot:straightLineTest -Psim.distance=-2 -Psim.outputDir=/tmp/reverse-trial
./gradlew test headlessTest
```

Once a new wrapper exists, use its fully qualified class in the same `headlessTest --tests` form.
VS Code's normal test discovery may not include `simTest`; keep the terminal command as the
supported route until custom source-set discovery is verified. Record `./gradlew --version` and
its JVM in laptop setup checks, then confirm the native fixture starts and its report is readable.

## Exact temporal semantics

Evaluate checks once per accepted frame. `eventually` starts its deadline on phase entry and
starts a hold timer on the first true sample; false, stale, missing, or nonfinite input resets the
hold timer. It passes only when consecutive valid samples cover the configured hold duration.
Require both start and end samples; a long gap cannot count as continuous success. Define a
maximum gap per signal, initially based on expected sampling cadence, and record it in results.

`always` and `never` begin at the declared phase boundary and stop when that phase exits. Missing
required data fails the check's observability and prevents phase success; it is never interpreted
as false for `never`. `within` uses inclusive bounds and explicit units; angle error wraps only
when the signal represents a periodic angle, not a mechanically limited multi-turn position.
For new temporal helpers, a frame at the deadline is evaluated before declaring timeout.
The legacy straight-line routine currently checks timeout first; retain that behavior in the
compatibility bridge and require a case-version change if its deadline semantics change. Empty observation windows and
zero valid samples cannot pass. Test these semantics with an injected clock and synthetic frames,
without HAL or sleeping.


Related: [shared framework](architecture.md), [initial suite](initial-suite.md), and [implementation roadmap](roadmap.md).
