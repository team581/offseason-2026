# Speed and command limits

- Implementation: `math/DrivetrainLimits`, `VelocityProfiler`, `RotationBudgetEstimator`, `VectorLimiter`.
- Inputs: robot dimensions and motors, carpet calibration, segment caps, measured motion, and heading plan.
- Segment caps can lower modeled limits; positive infinity uses the robot limit.

## Speed profile

- Sample the path at fixed distance intervals, piece boundaries, goal anchors, and heading nodes.
- Limit speed using curvature, goal passage speed, heading demand, and available wheel speed.
- Run a backward braking pass, then a forward acceleration pass.
- Account for translation and rotation sharing wheel speed and traction.
- Evaluate ahead using drive response lag and loop period.
- Blend path tracking into terminal position correction near a stop or closed gate.

## Limits

- Braking magnitude comes from calibration, capped by the segment.
- Motor acceleration depends on speed, motor data, current, and the modeled electrical losses.
- Corner speed follows the lateral acceleration limit: `v² * |curvature| <= lateral`.
- Rotation consumes traction and module speed before translation is limited.
- `VectorLimiter` projects the requested velocity onto the acceleration, traction, module-speed, and braking constraints.
- If measured motion makes those constraints infeasible, report it and use the existing recovery command.

## Tuning

- Edit existing values in `config/TrailblazerConfig`; see the [authoring guide](../../../../../../../../docs/writing-autos-tb3.md#tuning).
- Translation P is derived from response time. Heading uses the configured P/D gains.
- Keep model constants in the math classes; change them only when changing the model intentionally.
