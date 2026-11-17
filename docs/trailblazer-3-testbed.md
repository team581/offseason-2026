# TB3 comp testbed

- Choose `TB3 Right Trench Leave`, `TB3 Left Trench Leave`, or `TB3 Trench Rotation Test`.
- The leave autos start centered under their trench, facing the alliance wall, and drive 2 m toward it.
- Output uses the dedicated Phoenix trajectory velocity request.
- Current defaults: 8.5 m/s² traction, 6.0 m/s² braking, 55 ms response lag.
- Clearance uses 35-inch square bumpers and a 4 cm margin.
- These defaults need carpet validation.

## Checks

- Run the commands in [Writing TB3 autos](writing-autos-tb3.md#compare-changes).
- Compare drive time, full preview output, and the bump/state machine tests before and after edits.
- Desktop plants do not establish hardware accuracy or controller timing.

## Calibration

- Chooser entries: `TB3 Calibrate Lag`, `TB3 Calibrate Traction`, `TB3 Calibrate Braking`.
- Use the same drive request mode as the autos when measuring response.
- Put measured values in `TrailblazerConfig`.
- For another robot, supply its mass, inertia, module locations, wheel radius, reductions, motors, and slip current.
