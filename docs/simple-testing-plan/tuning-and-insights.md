# Tuning experiments and software insights

Status: proposal, October 10, 2026. APIs and milestones described here are proposed unless explicitly identified as existing.

[Project overview](../simple-testing-plan.md)

## Repeatable tuning experiments

Reuse target-step scenarios at a few representative positions/speeds. Record settling time,
overshoot, steady-state/RMS error, current, and voltage. Change one gain/configuration at a time,
repeat a small number of runs under comparable conditions, and compare. Keep gains fixed during
each run. Automatic tuning and broad search are unnecessary initially.

Use [WPILib SysId](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/introduction.html)
when feedforward characterization would address a specific mechanism issue. It fits parameters
from measured voltage/motion data. Characterization and the repeatable target-step acceptance
test serve different purposes; validate fitted gains with the latter on hardware.

## Passive checks on ordinary runs

Evaluate existing logs for prolonged not-ready time, target tracking error, stale sensors,
unexpected state dwell, resets, voltage sag, and scheduler timing. Start with a short list of
explicit checks scoped to meaningful operating phases. These can reveal problems that students
did not think to script. Normal operation lacks controlled initial conditions, so use these as
diagnostic flags rather than treating every match as a controlled regression experiment.
Keep this review in the match/practice workflow; share analysis helpers and AdvantageScope views
with development tests, but do not mix uncontrolled observations into their acceptance history.

## Tiny boundary sweeps and captured bug cases

Parameterize checks just below/at/above readiness thresholds, debounce duration, angle wrapping,
and state cancellation boundaries. A handful of purposeful cases can expose software errors
without simulating slip, collisions, or projectiles. Start captured-data regression with a few
recorded inputs to pure shot/localization calculations; defer whole-robot closed-loop replay.
Old sensor recordings cannot reveal how the physical robot would respond to changed outputs.

## Metric algorithms and experimental use

Capture timestamped target, measured position/velocity, error, current, and voltage over a named
phase. Use one acquisition path for checks and metrics; do not reread a controller to calculate
a metric. Store metric algorithm versions so a formula change invalidates old comparisons.

| Metric | Initial calculation |
|---|---|
| Settle time | Time from target-step edge to the start of the first valid uninterrupted in-tolerance hold; also report hold completion time |
| Overshoot | Maximum signed excursion beyond final target in step direction; report units, and percentage only for a nonzero step |
| Tracking RMS | Square root of time-weighted mean squared error over the declared window; reject gaps exceeding the sample policy |
| Steady-state error | Time-weighted mean error over a fixed final hold window, with max absolute error beside it |
| Ready/recovery time | First readiness hold after request or restored gate; specify each gate and debounce policy |
| Current/voltage | Per-device peak and time-weighted mean; battery minimum and context; no simulated-current hardware comparison |

For a gain change, select a fixed case/preset and collect at least three baseline and three
candidate hardware runs as an initial repeatability trial. Alternate baseline/candidate order
when practical and record battery replacement, temperature/warmup, mechanical changes, and
operator setup. Rebuild/redeploy or apply the existing supported configuration path while disabled;
record the effective gains each time. Never change gains inside the acceptance window.

Open the run summary, compare individual values and median/range, then open a saved AdvantageScope
layout showing target/measured/error, current/voltage, and phase markers. Review a warning by
checking traces and repeating under comparable conditions. Promote a baseline only after a
reviewer accepts the change and all absolute acceptance checks pass. If values vary enough to
hide the candidate effect, improve setup or observation rather than broadening thresholds.

## Offline checks and boundary cases

Implement passive analysis as catalogued checks with required paths, units, operating-phase
predicate, minimum valid window, threshold, and explanation of the next action. Begin with
tracking-error dwell, stale-input age, and loop timing. Each flag includes log/time range,
measured value versus threshold, and links to the relevant view. Report unavailable fields as
unsupported. Align streams by timestamps and reject stale joins rather than blindly taking the
last value forever. Verify units from logged metadata/source before interpreting timing metrics.

For threshold tests, use parameter presets at limit-minus-epsilon, limit, and limit-plus-epsilon;
choose epsilon above sensor/representation resolution. Debounce cases use one valid sample
before the boundary, at it, and after it, including a false sample that resets the hold.
Use synthetic frames and an injected clock for helper semantics; use physical fixtures only for
behavior that depends on the mechanism. Captured pure-function cases store inputs, expected
outputs, source run/time range, units, and a case version, and run through ordinary unit tests.


Related: [initial suite](initial-suite.md), [results and baselines](results-and-baselines.md), and [implementation roadmap](roadmap.md).
