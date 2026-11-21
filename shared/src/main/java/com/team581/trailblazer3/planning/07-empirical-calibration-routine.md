# Parameters

- `config/TrailblazerConfig`: calibration, feedback settings, and computed limits.
- `deriveLimits(robot, calibration, period, caps)` computes limits when a segment activates.
- Derived values follow the robot spec automatically. Do not copy calculated numbers into autos.
- Javadoc on `RobotDrivetrainSpec`, `EmpiricalDriveCalibration`, and `DrivetrainLimits` explains every field and its units.

## Robot inputs

| Input | Where the value comes from |
|---|---|
| Mass | Weigh the complete robot, including battery and bumpers |
| Yaw inertia | CAD mass properties about the robot's vertical center axis |
| Wheel radius | Loaded wheel travel divided by wheel rotation in radians |
| Drive/steer ratios | Multiply gear stages; motor turns per wheel/azimuth turn |
| Module positions | CAD X/Y offsets from robot center |
| Motor models | WPILib model matching each motor and control mode |
| Slip current | Current per drive motor when wheels start slipping on carpet |
| Static friction voltage | Slow voltage ramp; record when wheel motion starts |
| Loop period | Actual scheduled control period; comp uses 0.020 s |

## Carpet measurements

| Parameter | How to obtain it |
|---|---|
| Traction | Acceleration before slip: measured speed change / elapsed time |
| Braking | Deceleration magnitude during a braking test |
| Response lag | Fit measured speed-step response to the preview's first-order lag plant |

- `COMP`, `OFFSEASON`, and `NEW_MEMBER` hold those three measurements plus heading P/D for robots 581, 9991, and 9981.
- Current motion defaults are `(8.5, 6.0, 0.055)`, `(8.5, 6.0, 0.055)`, and `(8.0, 5.5, 0.060)`, in m/s², m/s², and seconds. Each robot currently uses heading P `8` and D `0.4`.
- Use the chooser's lag, traction, and braking calibration autos with the same downstream drive request as normal autos.

## Computed limits

Use `r` for wheel radius, `R` for largest module radius, `m` for mass, `I` for yaw inertia, and `dt` for loop period.

| Value | Calculation |
|---|---|
| Module radius `R` | Largest `hypot(moduleX, moduleY)` |
| Free speed | `(motor voltage - static friction voltage) * motor Kv * r / drive ratio` |
| Wheel-speed limit | `0.91 * free speed` |
| Translation speed | Wheel-speed limit, capped by the segment |
| Motor acceleration at speed | Current → motor torque → geared wheel force → force / mass; see `motorAcceleration` |
| Braking | Measured braking, capped by the segment |
| Traction | Measured traction |
| Lateral acceleration | `0.85 * traction` |
| Rotation speed | `0.85 * wheel-speed limit / R`, capped by the segment |
| Rotation acceleration | `0.85 * m * min(motor acceleration at zero / 0.90, traction) * R / I`, capped by the segment |
| Module turn rate | Smaller of half modeled steer free speed and `0.50 / (lag + dt / 2)` |
| Lag | Measured response lag |
| Inertia share | `min(1, 4 * I / (m * sum(module radius²)))` |
| Acceleration cap | Segment cap; infinity leaves the motor model in control |

- Steer free speed uses `max(0, steer voltage - min(static friction voltage, steer voltage / 2)) * steer Kv / steer ratio`.
- Motor acceleration includes back EMF, slip current, 0.018 Ω battery resistance, 0.95 inverter efficiency, 0.90 drivetrain efficiency, and a 0.90 acceleration reserve.
- Motor back EMF is `ks + (V - ks) * speed / freeSpeed`.
- Current is `min(slipCurrent, max(0, V - backEMF) / (motorResistance + 4 * 0.018 / 0.95))`.
- Acceleration is `0.90 * 4 * 0.90 * driveRatio * motorKt * max(0, current - freeCurrent) / (m * r)`.
- The margins and electrical assumptions are existing model choices; they are not derived from CAD.

## Feedback settings

| Parameter | Meaning and source |
|---|---|
| `calibration.headingP = 8` | Heading-error gain in 1/s; existing feedback setting; verify on carpet |
| `calibration.headingD = 0.4` | Angular-speed-error gain; existing damping setting; verify on carpet |
| `HEADING_RESPONSE_MULTIPLIER = 3` | Existing response-time choice; heading P is `min(calibration.headingP, 1 / (3 * (lag + dt / 2)))` |
| `TRANSLATION_RESPONSE_MULTIPLIER = 4` | Existing response-time choice; translation response is `max(minimum, 4 * (lag + dt / 2))` |
| `MIN_TRANSLATION_RESPONSE_SECONDS = 0.12` | Minimum correction response time; translation P is inverse response time |

- Heading P/D live in `EmpiricalDriveCalibration`; their values are copied into the calculated limits.
- Translation P is derived from that calibration's response lag. The follower has no integral term.
- These feedback choices are not determined by robot dimensions alone.
- Measure the physical limits and lag first; adjust feedback only if tracking or settling needs it.
- Segment `MotionLimits` are route caps chosen by the author, not extra robot calibration values.
