# Results, artifacts, and regression baselines

Status: proposal, October 10, 2026. APIs and milestones described here are proposed unless explicitly identified as existing.

[Project overview](../simple-testing-plan.md)

Write one unique run directory containing `results.json`, `summary.md`, and references to WPILOG
and optional CSV/video. Keep raw time series in WPILOG and small metrics in JSON. Include run/test
ID and version, Git SHA/dirty state, environment/composition, configuration, units, status, checks,
metrics, battery/setup context, and artifact paths. Missing context can invalidate a comparison.

Log test/run ID, case version, phase boundaries, checks, and terminal result in WPILOG so an
offline report can locate test-specific data in a longer robot log. Link manual endpoint or
shot observations to that same run ID. Case definitions describe expected behavior before
execution; run artifacts are receipts of what actually happened. Keep both accessible from
the coverage register described in [manual regression](manual-regression.md).

Results are PASS, FAIL, BLOCKED, or ABORTED. A failed prerequisite blocks dependent checks.
Disable, mode exit, exception, timeout, and completion all execute cleanup. Preserve evidence
on every outcome; reports must not be overwritten by the next run.

Freeze a known-good hardware baseline after several controlled repeats. Use absolute
requirements for hard failures; flag performance changes against comparable baseline runs as
warnings until repeatability is established. Compare matching test/configuration versions,
robot/calibration, environment, and battery/setup conditions. Do not compare simulated current
or settling to a hardware baseline. Near-zero metrics need absolute deltas rather than percentages.

Keep development-test review separate from match/practice review.
Use one development-test index linking case definitions, latest results, manual session receipts,
and comparable historical runs. Initially a comparison script and Markdown table are enough;
a database or dashboard is not a prerequisite. Link Fetch-hosted logs/video when convenient,
but do not require Fetch to manage cases, verdicts, or outliers. Any future integration needs a
specific gap and a scope decision. Evaluate reporting and case-management tools separately:
reporting summarizes executions; case management tracks planned coverage and procedures.
Flag missing or incomparable baselines explicitly rather than implying no regression.
Preserve known-good baselines and important failure receipts beyond CI's current 14-day artifact
retention, using an agreed development-artifact location; the index must link durable evidence.
Use saved AdvantageScope views for target/measured traces and state timelines; its
[line graphs](https://docs.advantagescope.org/tab-reference/line-graph/) support numeric and discrete
signals with a synchronized selected time across views.

## Versioned result contract

Implement a JSON schema and Java result records together; begin with `schemaVersion: 1`.
Use canonical SI units for calculations and an explicit unit string in each measurement.
Store no NaN/infinity in JSON: use null with `valid: false` and a reason. Required invalid
measurements prevent PASS. Stable check IDs support comparison across runs.

| Object | Required fields |
|---|---|
| Run | `schemaVersion`, `runId`, UTC start/end, Git SHA/dirty, environment, composition, robot identity, configuration/calibration hashes |
| Case | `caseId`, `caseVersion`, preset/parameters, status/reason, required capabilities, missing capabilities, start/end offsets |
| Check | ID, phase, status, signal/source, expected bounds, unit, measured value, first failure offset, valid sample count, freshness/gap policy |
| Metric | ID, value/unit, phase window, sample count, validity/reason, algorithm version |
| Artifact | kind, relative path or URL, log time range, availability, optional checksum |
| Context | battery identity and voltage range, setup description, operator observations, input-script version/seed when applicable |

Store `runs/<runId>/results.json` and `summary.md` under the supplied report root; maintain a
small `index.md` linking runs. Generate both from the same immutable result. Write through a temp
file and rename after serialization; reject duplicate run IDs rather than overwriting them.
The summary shows status first, first failure with measured/expected values and time, blocked
capabilities, metrics, and artifact links. Manual acceptance has its own case result or receipt
and does not overwrite the automatic diagnostic verdict.

Keep current stable log keys, including `Tests/Regression/RunId`, `Status`, and step reasons.
Add case version, preset, phase, loop sequence, relative time, and check/metric values under
`Tests/Regression/Active/`. Stable keys avoid allocating a new WPILOG entry for every run; run-ID
and phase transitions delimit segments. Log metadata at run start and terminal evidence at end.
Offline extraction requires both DS mode/enabled flags and run markers; a missing terminal
marker means incomplete evidence, not PASS.

## Export, compare, and CI integration

Add a desktop CLI under proposed `tools/testing/` with `export`, `compare`, and `index` operations.
Implement structured result writing in Java first. Add WPILOG extraction when hardware metrics
need it; resolve exact logged paths from the entry catalog rather than assuming a prefix.
The exporter accepts a log, run ID, and optional manual observations, and reports missing signals
explicitly. It checks timestamp monotonicity and uses source timestamps when available.

Define the CLI interface before implementing its launcher. Proposed invocations, unavailable today:

```sh
tools/testing/report export --log /path/robot.wpilog --run-id RUN_ID --observations /path/manual.json --out /path/results
tools/testing/report compare --candidate /path/results/RUN_ID/results.json --baseline /path/baseline.json --out /path/comparison
tools/testing/report index --runs /path/results --out /path/results/index.md
```

`export` supports an omitted observations file for purely automated cases. `compare` writes both
`comparison.json` and `comparison.md`, including baseline IDs, matched keys, exclusions, and metric
deltas. Use exit 0 for a completed passing operation, 1 for failed required checks, and 2 for invalid
input or incomplete/incomparable evidence. Performance warnings remain exit 0 until a declared
profile makes them blocking. Preserve output for every exit. Keep the launcher and offline parser
dependencies desktop-only; deployable scenario classes do not depend on them.

The comparator accepts a candidate result and an explicitly selected baseline manifest. Match
case/version/preset, environment/composition, robot/calibration, configuration fields relevant to
the metric, and setup/battery policy. List every mismatch and decline that comparison. Permit only
named experimental fields, such as one tuning gain, to differ for an experiment; record those
changes instead of relaxing all configuration matching.

A baseline manifest contains approved run IDs, comparison keys, metric algorithm versions,
absolute acceptance limits, warning deltas, and approval date/owner. Report median and range
across repeats; retain individual values and absolute failures. Percentage changes apply only
above a metric-specific nonzero floor; otherwise use absolute differences. FAIL takes precedence
over a favorable average. Warnings cannot change a failed check into PASS or promote a baseline.

For desktop runs, use the existing `sim.outputDir` as the report root and add run directories
beneath it. Keep legacy `summary.md`/CSV outputs during migration. Existing CI globs already upload
that directory; extend the summary collection to find nested run summaries once writers migrate.
Keep JUnit XML/HTML, fail the JUnit wrapper for FAIL/BLOCKED/ABORTED required cases, and write
artifacts in `finally` before asserting. A failure before fixture startup still needs a configuration
or setup failure result. Hardware-only exclusions are listed in the CI profile, not JUnit passes.

Current CI runs on push. To make checks required before merge, add a `pull_request` trigger,
verify the headless job runs on candidate PRs, and configure the exact check name in branch
protection. Add the headless job to Build's dependencies if Build is intended to summarize all
verification. Preserve artifact upload on failure and publish comparator warnings in the job
summary; enable warning-based blocking only after repeatability is established.


Related: [shared framework](architecture.md), [tuning and software insights](tuning-and-insights.md), and [implementation roadmap](roadmap.md).
