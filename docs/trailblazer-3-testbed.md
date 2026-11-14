# Trailblazer 3 comp testbed

Select `TRAILBLAZER_3_RIGHT_TRENCH` in the comp auto chooser. The robot starts centered under the alliance’s right trench with the intake facing the driver station wall. The segment uses full modeled limits and velocity-only Phoenix output. The testbed calibration uses 4.0 m/s² traction, 1.6 m/s² braking, and 70 ms response lag. The footprint uses 35-inch square bumpers and a 4 cm clearance margin. The calibration values are desktop estimates.

The one-dimensional constant-traction reference for 2 m predicts a 2.138 m/s peak and braking after 0.571 m in the absence of loop delay and settling. The reference movement takes 1.871 s before dwell. The deterministic 70 ms lag plant reached 2.0051 m on both alliances in 2.20 s with 0.177 m minimum clearance and a 2.088 m/s peak command. It peaked at 0.434 m, 0.137 m before the zero-delay braking point. The planner evaluates speed 90 ms ahead at 50 Hz, so raw observed progress at braking and the zero-delay reference braking position measure different points. The peak command differs by 2.33%, outside the 2% target. The model flagged five terminal recovery ticks per run between 1.82 and 1.90 s because the remaining stopping distance was below the command-history braking envelope. Braking commands remained within the configured bound and the final command held zero for one second.

Five Phoenix runs on each alliance settled in 2.12–2.16 s, traveled 1.9906–1.9929 m and maintained at least 0.175 m clearance. Peak commands ranged from 2.097 to 2.135 m/s. All ten passed the travel and clearance checks with zero infeasible ticks. These temporary harness runs exercised the trench state machine and the Phoenix field-centric velocity request directly. Full chooser and robot scheduler verification remains pending. The temporary harness was removed after the runs.

## 2027 integration

Supply the new robot's module centers, wheel radius, reductions, mass, yaw inertia, motors, slip current, and carpet measurements through `RobotDrivetrainSpec` and `EmpiricalDriveCalibration`. Verify the azimuth-drive odometry compensation against the module ring-to-pinion coupling ratio. Provide season-owned field geometry, bumper dimensions, and hard keepouts through `FieldGeometry` and `FieldClearance`, and author scenarios from the robot's actual motion plans.

Normalize pose timestamps to the injected `TimeSource` epoch. Supply field-relative measured velocity, localization validity and reset indicators, and last applied velocity for command handoffs. Supply live power limits only from an instantaneous per-drive supply ceiling and resting or sag-compensated battery voltage.

Consume the command in a dedicated zero-deadband drivetrain path. Record the command, feedforwards, measured pose and velocity, saturation, limiting factors, and events. For rotation snaps, supply pre-mapped Blue-origin driver field velocity and keep translation ownership with the driver. Seed manual control from the last applied velocity on assist release. Calibrate the response lag with the same downstream feedforward mode used on carpet.

## Remaining acceptance evidence

The comp hardware, target-controller timing and allocation, and 2027 carpet calibration are unmeasured. The automated suite covers the trench path in two plants and representative shared-library cases. The broader scenario matrix and log-replay acceptance work are pending.
