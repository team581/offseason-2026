# Simulation

How to run and debug the comp-bot robot code in the WPILib simulator.

## Launch

```bash
./gradlew comp-bot:simulateJava
```

This starts the robot code plus the WPILib Sim GUI and a simulated DriverStation. The GUI is
enabled unless `SIM_HEADLESS` is set. The configuration is in `comp-bot/build.gradle`.

## Controls

`comp-bot/simgui-ds.json` persists the keyboard-joystick mappings. Keyboard 0 currently maps:

- Axis 0 (LeftX): A/D
- Axis 1 (LeftY): W/S — robot translation on the Xbox left stick is axes 0/1
- Axis 2 (LeftTrigger): E/R (slow ramp, no decay)
- Buttons 1–4: Z/X/C/V

In the Sim GUI DriverStation window, assign Keyboard 0 to joystick port 0, tick the "Xbox"
checkbox, and use "Keyboard 0 Settings" to add more axes/buttons.

Xbox axis indices: 0 LeftX, 1 LeftY, 2 LeftTrigger, 3 RightTrigger, 4 RightX, 5 RightY.
Xbox buttons: 1 A, 2 B, 3 X, 4 Y, 5 LB, 6 RB, 7 Back, 8 Start.

To fully drive and score, add axis 3 (RightTrigger = score/feed) and axis 4 (RightX = rotation),
e.g. on T/G and J/L (J/L also appear on Keyboard 1, which is harmless while only Keyboard 0 is
assigned to port 0).

Robot bindings (from `Robot.configureBindings`):

- Driver RT: score-or-feed (press = prepare, release = idle)
- Driver LT: intake (hold)
- Driver LB: eject (hold)
- Driver RB: idle
- Driver Back: zero gyro
- Operator RT: warmup score

## Alliance and auto selection

The DriverStation window sets alliance/station. The auto chooser is published to NT at
`/SmartDashboard/Autos/SelectedAuto` (default `DO_NOTHING`; `LEFT`/`RIGHT` run the bump-crossing
autos).

## AdvantageScope

Open AdvantageScope and use "Connect to Simulator" (NT4 to localhost). 3D robot assets live in
`advantagescope_assets/Robot_comp`. DogLog publishes all `DogLog.log` topics to NT (development
mode), and `DogLog.tunable` values are editable over NT — search for `FeatureFlags/UnbeachScore`,
`Swerve/StuckOnBallBackoffSpeed`, `Sim/StuckOnBall/*`.

## Verification scenario A (teleop, score while stuck on a ball)

1. Enable Teleop.
2. Drive across the bump toward the alliance zone. When the robot reaches the virtual ball
   (default just past the outpost bump landing zone), watch `Imu/Main/Pitch` / `Imu/Main/Roll`
   rise (tilt up to ~10°) and `Imu/BeachedRecovery/StuckOnBall` go true.
3. Hold right trigger: `RobotManager/State` goes `PREPARE_SCORE` then `SCORE_STUCK_ON_BALL`.
4. The robot creeps backward away from the ball at `Swerve/StuckOnBallBackoffSpeed` (0.3 m/s)
   while still aiming at the hub (`Swerve/ScoringAngle`) and running the shooter/hopper.
5. As distance to the ball grows, tilt decays; once tilt stays below ~5° for 0.25s the state
   returns to `SCORE`.

## Verification scenario B (auto)

Select the `RIGHT` auto and enable Autonomous. During `CROSS_BUMP_TO_SHOOT_1` the existing tilt
pulses play; after landing on the ball the sustained tilt begins. In `SHOOT_1` the RobotManager
enters `SCORE_STUCK_ON_BALL`, backs off while scoring, then resumes the normal `SCORE` path.

## Tilt compensation

While beached (`SCORE_STUCK_ON_BALL`, and `PREPARE_SCORE`-while-stuck), the hood angle is offset by
`TiltCompensation.getHoodCompensationDegrees(pitch, roll)` = `-pitch` for the rear-facing shooter,
scaled by tunable `TiltCompensation/HoodGain` (default 1.0). Watch
`ShooterHood/ScoreAngleOffset` and `RobotManager/Scoring/TiltCompensationDegrees`.

## Headless self-test

```bash
SIM_SELF_TEST=1 ./gradlew comp-bot:simulateJava
```

Selects the `RIGHT` auto, enables autonomous via `DriverStationSim`, drives over the bump onto the
virtual ball, and prints `SIM_SELF_TEST PASS`/`FAIL` with assertion details to the console (no GUI
interaction needed). It also nudges the ball 0.09 m ahead of the shoot point via the
`Sim/StuckOnBall/BallXOffset` tunable so the auto (which settles ~0.19 m short of its target)
lands deep enough on the ball to exercise the full backoff.

## Key NT topics to watch

- `RobotManager/State`
- `Swerve/State`
- `RobotManager/Scoring/StuckOnBall`
- `Imu/Main/Pitch`, `Imu/Main/Roll`
- `Imu/BeachedRecovery/RecoveryPose`
- `Imu/RobotPoseWithTilt`
- `Sim/StuckOnBall/*`
- `Localization/EstimatedPose`

## Agent control

AI agents can drive the robot in teleop or auto without any GUI interaction. A sim-only shim
(`com.team581.sim.AgentSimShim`) listens on the `/Agent` NT table and re-asserts the driver
joystick and DriverStation state every robot loop via `DriverStationSim`, so agents exercise the
exact same input path as a human driver. The `sim_agent` Python package
(`shared/src/main/python/sim_agent`) provides the client:

```bash
# Launch a headless sim, drive/rotate/score in teleop, assert state transitions
uv run --package sim-agent sim-agent drive_and_score

# Select and run the RIGHT auto
uv run --package sim-agent sim-agent right_auto

# Attach to an already-running sim (e.g. one with the GUI open)
uv run --package sim-agent sim-agent drive_and_score --attach
```

Headless runs set `SIM_HEADLESS=1`, which skips the Sim GUI (`addGui().defaultEnabled` gate in
`comp-bot/build.gradle`); AdvantageScope can still connect to the robot's NT4 server, and every
run writes a `.wpilog` to `comp-bot/logs/` plus JSON recordings to `comp-bot/logs/agent-runs/`.

Write custom scenarios as Python files with a `run(session)` function and pass the path to the
`sim-agent` CLI. The full `/Agent` topic contract, the `SimSession` API, and troubleshooting
steps are documented in the `simulate-and-debug` agent skill (`.agents/skills/simulate-and-debug/SKILL.md`).
