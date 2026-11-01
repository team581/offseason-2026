# Offseason robot behavior verification

Use this procedure with the existing task, [behavior specs](README.md), [integration-testing guide](../integration-testing.md), and [bringup checklist](../subsystem-bringup-checklist.md). Record results in the task rather than maintaining a second completion list.

## Before a run

Agree with the software lead on the behavior being checked, conditions, expected outputs, and measurable pass criteria. Record these before running:

- Robot/application identity and deployed commit.
- Starting pose, mechanism configuration, operating mode, and relevant sensor conditions.
- Test locations, distances or paths, number of attempts, and ball count where applicable.
- Acceptable outcome, tolerances, and abort conditions.
- Person responsible for running and confirming the check.

An unresolved value is an open decision, not an assumed pass criterion. Select physical limits and tolerances from the approved behavior and measured robot; do not infer them from a successful simulation.

## Initial behavior checks

These are verification scenarios, not claims that tests exist or have passed. Confirm any unresolved expected behavior before implementing or executing a check.

| Behavior | Check | Record |
| --- | --- | --- |
| Score/feed transitions | Enter each mode, cross the relevant field-zone boundary, release the request, and repeat. Compare coordinated mechanism requests and outputs with the approved spec. | Robot/manager states, mechanism requests and outputs, current limits, and any stale request after a transition. |
| Unjam | Enter unjam, hold it, release it, and verify recovery to the agreed state. Settle the per-mechanism actions before the run. | Entry, sustained operation, recovery, and outputs for every commanded mechanism. |
| Vision-loss fallback | Exercise vision disabled/lost and restored under controlled conditions. Check approved fallback selection, hood behavior, and firing constraints. | Camera/localization health, selected fallback, hood and shooting requests, and recovery behavior. |
| Shot verification | Shoot from measured stationary locations, then run the agreed straight-back, side-to-side, and left/right circular motion patterns. | Starting locations, paths/speeds, attempted/scored ball counts, aiming error, and logs/video for each condition. |

For the straight-line diagnostic, follow the [on-robot instructions](../integration-testing.md#on-robot), record its result and reason, and independently measure final position. A passing diagnostic does not establish shooting performance or acceleration-limit compliance.

Run applicable simulation checks first. Add reusable checks to the existing integration framework when simulation can meaningfully represent the behavior; keep the physical acceptance check separate.

## Record the result

Add a short comment to the existing task:

```text
Code: <commit or PR, plus deployed commit if different>
Check: <conditions, repetitions, and acceptance criteria>
Result: <passed, failed, or aborted; measurements and reason>
Evidence: <log/video/report link>
Verified by: <person who ran or witnessed the check>
```

Keep the task open if a required check failed, was aborted, or has not run. Record the next action. Close it only after the agreed checks pass; a feature implemented in code may still need simulation, tuning, or robot verification.
