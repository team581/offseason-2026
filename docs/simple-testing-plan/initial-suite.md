# Initial regression test suite

Status: proposal, October 10, 2026. APIs and milestones described here are proposed unless explicitly identified as existing.

[Project overview](../simple-testing-plan.md)

| Test | Primary evidence | Added value |
|---|---|---|
| Straight line, reverse, strafe | Pose, heading, cross-track error, settling | Reuse today's diagnostic and establish the authoring pattern |
| Hood/turret target changes | Measured angle/error, settle time, overshoot | Regression checks and repeatable tuning measurements |
| Shooter preparation | All four measured RPMs, expected ratios/directions, settle time | Detect follower problems that the single readiness observation may miss |
| Readiness lost while scoring | Manager/transport states and commanded outputs | Catch a high-impact interlock regression without ball physics |
| Intake cancel/unjam/stow | Request/state/output sequence | Cover common operator transitions cheaply |
| Test disable/exit | Abort status, cleared requests, neutral output commands | Verify shared runner cleanup; distinguish command stopping from mechanical coast-down |

Start with 3–5 scenarios and grow from real bugs. Track existing unit tests, automated scenarios,
and manual cases together in the [coverage register](manual-regression.md). Trial the manual
hardware protocol now, using normal controls for scoring/cancellation and Test mode for the
existing drive diagnostic; mechanism scenario automation need not precede hardware regression.

Validate simulated input availability before
promising a scenario in CI. Physical shot success and localization accuracy need independent
observations: RPM readiness does not prove a hit; odometry reaching its own target does not
prove field accuracy. Initially use a measured endpoint or operator shot count tied to the run.

## First implementation cases

Implement drive reuse and abort handling first, then choose one mechanism case with complete
observations. Promote interlocks only after the required input and output adapters work.

| ID/preset | Setup/action | Checks and measurements | Capability gate |
|---|---|---|---|
| D01 forward/reverse/strafe | Current `StraightLineConfig`; distance +2/-2 m or direction 90°; idle baseline before enable | Retain current endpoint/heading/cross-track/stopped checks; report duration, peak velocity/acceleration and endpoint error | Valid pose/speeds; external endpoint observation needed for field-accuracy claim |
| M01 hood-step | Homed hood; `scoreRequest(distance)` at two reviewed distance presets, then `idleRequest()` | Finite bounded angle, independent expected angle, continuous hold, settle/overshoot metrics | Measured angle and homing evidence; physical thresholds calibrated before acceptance |
| M02 shooter-prepare | `prepareScoreRequest(distance)` from idle at fixed distance, then idle | Four measured motor RPMs versus intended ratios/signs; first-ready and settle times; no unintended transport request | Device readings via test-owned hardware references or offline log extraction; `atGoal()` alone insufficient |
| I01 readiness-loss | Prepare scoring, make one supported gate false, restore, cancel | State/transport command matches documented gate and in-progress-shot policy; blocked duration and recovery time | Controllable gate plus observable transport commands; otherwise manual-only |
| O01 cancel/unjam/stow | `intakeAutoRequest()`, cancel, `unjamRequest()`, idle, stow/cancel-stow | Expected request/state/output sequence, bounded transition delay, no lingering request | Public manager composition plus request/output evidence |
| X01 disable/exit | Interrupt drive and one available mechanism case during action and hold | ABORTED, neutral commands, no restart while still disabled or on unrelated mode entry | Lifecycle adapter and actual output observer |

D01 uses current defaults: 10 s timeout, 0.10 m position tolerance, 3° heading tolerance,
0.20 m cross-track tolerance, 0.10 m/s stopped translation, and 0.25 s settle. Preserve the
existing rotation stopping rule from `StraightLineRoutine`: angular speed within 5°/s of zero.
Acceleration remains a metric, not an enforced acceptance limit. Mechanism tolerances/deadlines
must come from reviewed requirements or repeatable hardware trials; do not copy drive limits.

Keep signal access explicit. `Shooter` currently logs `Shooter/TopLeft/RPM`, `TopRight/RPM`,
`BottomLeft/RPM`, and `BottomRight/RPM` but exposes readiness rather than all RPMs as getters.
Use those traces for initial offline/manual results. For online checks, add a test-owned adapter
that reads the existing hardware devices once with timestamps and documented gearing/sign
conversion; do not add private-field reflection. If that observation cannot be made reliable,
leave the online case BLOCKED and retain the offline evidence.

For every promoted CI case, record one failure demonstration and one cleanup demonstration.
Keep normal binding tests and manual normal-robot cases linked beside diagnostic results;
a fixture state transition does not certify that an operator button reaches it.


Related: [test authoring](authoring.md), [shared framework](architecture.md), and [tuning and software insights](tuning-and-insights.md).
