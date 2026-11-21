# Runtime integration

- Implementation: `runtime/Trailblazer`; robot wrapper: `frc.robot.autos.TrailblazerAuto`.
- Supply an injected seconds clock, robot spec, calibration, period, and optional field clearance.
- Supported nominal period: 4–20 ms.
- Measurements and output speeds are field-relative.

## Calls

| Call | Effect |
|---|---|
| `activate(segment)` | Start with default clearance |
| `activate(segment, clearance)` | Start with explicit clearance |
| `activateWithoutClearance(segment)` | Start without keepout checks |
| `update(time, pose, speeds)` | Compute and store the next command |
| `passedMarker(marker)` | Read recorded passage |
| `atGoal()` | Read terminal completion |
| `cancel()` | Clear the segment and command |

## Recovery

- Duplicate timestamps return the cached result.
- Nonfinite measurements return zero and reset command seeding.
- Backward time or a gap over 100 ms triggers missed-tick recovery.
- Off-track error recovers smoothly onto the fixed path using unified cross-track control and velocity re-seeding on sustained collision divergence.
- Cancellation clears markers.

## Robot state machine

- Collect inputs before state transitions.
- Use `getNextState` for transitions, `afterTransition` for entry actions, and `whileInState` for repeated requests.
- Use the injected follower clock for auto timeouts.
- Run the auto state machine before the follower update, as in `Autos`.
- Keep bump tracker arming, cancellation, and shooting readiness in the auto.
- The current runtime supports static auto segments; teleop assist and dynamic-target APIs are not implemented here.
