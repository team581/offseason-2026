# Straight-line test

Result: **PASSED** — Reached goal and settled within tolerances

Configuration: `StraightLineConfig[distance=6.0, direction=0.0, maxVelocity=3.0, maxAcceleration=3.0, timeout=20.0, positionTolerance=0.1, headingTolerance=3.0, stoppedVelocity=0.1, settleSeconds=0.25, crossTrackTolerance=0.2]`

Battery: 12.00 V; initial heading: 0.00 degrees

| Measurement | Value |
|---|---:|
| Duration (s) | 3.075 |
| Peak speed (m/s) | 3.003 |
| Peak absolute acceleration (m/s²) | 58.587 |
| Maximum cross-track error (m) | 0.004 |
| Final position error (m) | 0.083 |

Acceleration is the finite difference of measured velocity and includes vendor timing noise.

This result checks endpoint and settling, not acceleration-limit compliance.
`drive-output.csv` records the stopped baseline and each actual manager command alongside
the measured field-relative velocity immediately before applying that command.
