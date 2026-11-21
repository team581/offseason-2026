# Verification

```sh
./gradlew :shared:test :comp-bot:test --tests 'frc.robot.autos.*'
python3 tools/compare_tb3.py --output /tmp/tb3-times.json
```

## Existing regression coverage

- Rounded corners, adjacent fillets, keepouts, and exact reversals.
- Fixed headings, directed turns, hard gates, heading passage tolerances, and Hub aiming.
- Stops, moving finishes, markers, closed-loop routes, and cancellation.
- Offset starts, localization resets, nonfinite input, and missed ticks.
- Ideal and lagged plants.
- Preview compilation, source spans, cursor selection, schema parity, and both alliances.
- Auto transitions, timeouts, disabled wheel alignment, and bump tracker progression.

## Comparing a cleanup

- Require zero drive-time delta and matching preview output for each affected auto/alliance.
- Source locations and build timings may differ; trajectories and events must match.
- Run state machine and bump tests because previews simulate driving segments only.
- Keep regression tests that protect current behavior. Remove unused development wrappers and harnesses.

## Limits

- Preview durations omit shooting waits and bump sensor timing.
- Deterministic desktop parity checks a refactor; it does not measure carpet performance.
- Hardware timing, calibration, and target-controller allocation need separate measurement.
