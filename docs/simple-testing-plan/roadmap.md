# Implementation roadmap and acceptance criteria

Status: proposal, October 10, 2026. APIs and milestones described here are proposed unless explicitly identified as existing.

[Project overview](../simple-testing-plan.md)

| Milestone | Deliverable | Done when |
|---|---|---|
| 0. Trial the process | Inventory existing unit/headless coverage; define a short manual protocol and save an offseason-bot session receipt | Coverage gaps and not-run cases are visible; executed cases have observations and logs; review produces follow-up actions |
| 1. Prove reuse | Shared observations/checks and result records around straight line | Same definition/checks run headless, interactive, and on hardware; failure/abort logs retained |
| 2. Prove authoring | Three templates, discovery/chooser registration, student guide and agent skill | Student adds/runs a regression in under 15 minutes; intentional defect is detected |
| 3. Cover likely bugs | 3–5 state/interlock/mechanism scenarios using the diagnostic entrypoint, plus manual cases for normal-robot paths | Supported CI checks pass; unsupported capabilities are visible; manual results identify the exercised composition; no private-field mutation |
| 4. Support tuning | Step-response metrics, controlled hardware repeats, baseline comparator | A gain change produces a reproducible comparison with target/measured evidence |
| 5. Observe everyday use | A few offline log checks and small boundary sweeps | Reports identify actionable problems with tolerable noise and maintenance cost |

For software changes, run units plus the small headless suite. Require the headless status for
merge once reliable, alongside existing checks. Make hardware routines easy to select/run during
bringup and after control/calibration changes. Add nightly runs only when extra cases justify
them; repeatedly running unchanged deterministic tests adds little evidence.

After the offseason trial, review authoring friction, laptop setup, reporting gaps, and manual
session effort. Fix gaps before expanding the suite. Establish a weekly hardware regression
window with an assigned lead/reviewer and reserved robot/field time; keep receipts and track
missed sessions. Preserve valuable existing math/unit tests alongside SIL;
do not replace them with slower integration tests.

Reporting/tool selection follows working non-trivial tests. Start with a coverage register and
one development-test index linking CI results and manual receipts. Consider a reporting or
case-management platform only if a specific workflow remains painful. Fetch scope expansion
and a new review platform are not milestone prerequisites.

Stop after the first authoring trial if a typical test still requires HAL setup, reflection,
custom timing/report code, or several infrastructure edits. Fix the shared helpers before
expanding coverage. Measure useful bugs caught, authoring/run time, flaky failures, and whether
someone can identify the next action from the report.
Include planned cases lacking coverage or execution evidence, and whether manual regression
sessions happen and produce follow-ups. More test definitions alone do not show adoption.

Defer game-piece physics, richer dynamics, HIL rigs, a general scenario language, broad random
fuzzing, whole-robot replay, database migrations, and autonomous self-improvement. Add each only
when a specific unresolved problem makes its expected value exceed its maintenance cost.

## Implementation batches and verification

| Batch | Concrete changes | Verification before expanding |
|---|---|---|
| 0 | Add `docs/testing/coverage.md`, two case files, session template, and development index; map current tests | Complete one hardware receipt with all planned cases accounted for; verify evidence links |
| 1 | Add result records/schema and drive bridge; retain current chooser/configuration and CSV reports | Existing focused drive/session tests still pass; invalid config, abort, check failure, and report failure retain evidence |
| 2 | Add frame/check helpers, explicit catalog, three templates, common JUnit wrapper | Clock-only checks cover holds/gaps/deadlines/nonfinite inputs; one catalog case appears in chooser and focused JUnit report |
| 3 | Add one minimal mechanism composition and opt-in diagnostic entrypoint/tasks | No duplicate hardware owners; one definition runs headless/interactive/hardware; mode exit neutralizes every owned output |
| 4 | Add remaining 3–5 supported cases and coverage profile; finish laptop guide and agent skill | Student authoring trial completes; each promoted case catches a deliberate defect and lists unsupported checks |
| 5 | Add metric calculations, baseline manifest, comparator, and saved analysis views | Controlled repeats expose a measurable change; mismatched versions/robots/configurations refuse comparison |
| 6 | Add offline checks, nested CI summaries, PR trigger and required-check configuration | Failed PR test retains artifacts and blocks merge through configured protection; flags include actionable trace windows |

Batches refine the milestone table rather than require a single large rollout. Merge result records
and drive compatibility first; mechanism automation remains gated on public observations and
single-owner composition. Run existing `DiagnosticSessionTest`, `DiagnosticReportTest`, manager,
and straight-line tests for lifecycle/report changes. Use fast unit tests for temporal and metric
algorithms, including missing data and mismatched baseline rejection. Run only supported physical
scenarios during hardware validation and record application restoration after a diagnostic session.

## Day-to-day use

For a software change, the author runs focused unit/scenario tests, adds a regression for a fixed
bug, updates the coverage entry, and reviews failure evidence before pushing. CI runs the declared
software profile. The reviewer checks whether the changed behavior has unit, simulated, or manual
evidence and whether uncovered paths are explicit; a green diagnostic cannot substitute for a
missing normal-robot case.

For weekly hardware regression, the lead selects versioned cases, records the deployed build and
setup, runs the protocol, attaches observations/logs, and assigns actions for failures or missing
coverage. For tuning, the engineer selects a baseline and fixed preset, collects controlled repeats,
compares metrics, and records a reviewed baseline update. At each coverage review, prioritize actual
failures and high-impact gaps; retire duplicate cases or obsolete presets with a recorded reason.


Related: [shared framework](architecture.md), [test authoring](authoring.md), [manual regression](manual-regression.md), and [results and baselines](results-and-baselines.md).
