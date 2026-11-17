# Integration status

## Implemented

- Static auto segments, rounded geometry, heading plans, speed profiling, and command limiting.
- Comp auto state machines, previews, calibration routines, and regression tests.
- Shared carpet defaults and field keepouts for robot and preview.
- Batch working-tree versus branch comparisons.

## When moving to another robot

- Supply its physical drivetrain data and measured carpet calibration.
- Supply the field geometry, bumper dimensions, and keepouts.
- Use one clock domain and field-relative measured speeds.
- Send commands through the dedicated trajectory drive request.
- Verify both alliances, stops, corners, heading gates, displaced starts, and bump recovery.
- Measure control-loop timing on the target controller.

## Outside the current implementation

- Dynamic target policies and shared-axis teleop assists.
- Wheel-force feedforward output and live supply-limit inputs.
- Hardware qualification for a future robot.
