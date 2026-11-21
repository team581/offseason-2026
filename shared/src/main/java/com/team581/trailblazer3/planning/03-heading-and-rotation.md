# Heading and rotation

- Implementation: `math/HeadingSchedule`, `HeadingPlanner`, `RotationBudgetEstimator`.
- Heading is independent of travel direction.
- A pose waypoint supplies a fixed heading. A translation waypoint supplies position only.
- Other targets can face travel or a field point with a mechanism offset.

## Scheduling

- Fixed-heading through points with explicit angular tolerances use a continuous spatial schedule.
- The schedule is indexed by measured path progress.
- Other heading targets use the bounded heading reference in `HeadingPlanner.step`.
- Heading offsets move the anchor before or after the waypoint in meters.
- `clockwise` and `counterClockwise` choose the turn direction; the default is shortest.
- Clearance can delay the initial turn until the bumper sweep fits.
- Heading demand reserves module speed and traction in the translation profile.

## Gates and completion

- Soft headings allow translation to continue.
- Hard gates hold progress at the heading anchor until readiness or the existing bypass condition.
- Stop goals check heading error, angular speed, position, linear speed, and dwell.
- Heading passage diagnostics record error at the anchor.

## Feedback

- Spatial tracking uses heading P and angular-rate D, with lag compensation in feedforward.
- Other heading tracking uses a lag-derived P capped by the robot's calibrated `headingP`.
- Existing gains live in `config/TrailblazerConfig`.
- Verify both alliances, directed turns, clearance, and missed-loop recovery after changes.
