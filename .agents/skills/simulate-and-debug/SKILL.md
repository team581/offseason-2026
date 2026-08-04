---
name: simulate-and-debug
description: Drive and debug this repo's robot code in the WPILib simulator — launch the sim headless, control teleop/auto through the /Agent NetworkTables gamepad, assert on /Robot topics, and analyze results with AdvantageScope or wpilog.
license: MIT
---

# Simulate and debug

Closed-loop workflow for AI agents: make a code change, launch the WPILib simulator headless,
drive the robot in teleop or auto through NetworkTables, assert on live robot state, and keep a
recorded artifact (JSON samples + `.wpilog`) for debugging. No GUI interaction is ever needed.

Use this skill whenever a task requires verifying robot behavior: driving, state machine
transitions, mechanism actions, autos, tunables, or reproducing a bug seen on the field.

## How it works

- `com.team581.sim.AgentSimShim` (shared module, sim-only) listens on the `/Agent` NT table. When
  `/Agent/Enabled` is true it re-asserts the driver joystick + DriverStation state every robot
  loop via `DriverStationSim`, so the agent drives the **real input path** (`XboxController` →
  `ControllerBindings` + `XboxControllerDriveSource` → swerve/RobotManager). When false it is
  inert and a human's Sim GUI keyboard controls work as before.
- The robot process hosts the NT4 server on localhost; the Python client (`sim_agent`, using
  pyntcore) connects as a plain NT4 client — exactly like AdvantageScope does.
- Everything observable is published by DogLog under `/Robot/**` (dev mode), tunables are
  writable under `/Tunable/**`, and every run writes a `.wpilog` to `comp-bot/logs/`.
- Headless launch skips the Sim GUI (`SIM_HEADLESS=1` gates `wpi.sim.addGui().defaultEnabled` in
  `comp-bot/build.gradle`). This is also more reliable: `libhalsim_gui` has SIGSEGV'd the robot
  JVM in-process before (see `comp-bot/hs_err_pid*.log`).

## Quick start

Run everything from the repo root. The first launch compiles the robot, so allow a few minutes.

```bash
# Teleop smoke test: enable teleop, drive forward, rotate, hold RT to score, release to IDLE
uv run --package sim-agent sim-agent drive_and_score

# Auto smoke test: select the RIGHT auto, enable autonomous, verify the robot drives and acts
uv run --package sim-agent sim-agent right_auto

# Attach to a sim you (or a human) already started, e.g. to watch in the Sim GUI / AdvantageScope
uv run --package sim-agent sim-agent drive_and_score --attach
```

Exit code 0 = PASS, 1 = assertion failure, 2 = unexpected error. Artifacts land in
`comp-bot/logs/agent-runs/` (gitignored): sim stdout (`sim-stdout-*.log`) and 50 Hz topic
recordings (`<scenario>-*.json`).

To watch live as a human: open AdvantageScope → "Connect to Simulator" (works headless), or open
the run's `.wpilog` afterwards. 3D assets are in `advantagescope_assets/Robot_comp`.

## Writing a scenario

A scenario is any Python file with a `run(session)` function; pass its path to the CLI:

```python
import time
from sim_agent import SimSession

def run(session: SimSession) -> None:
    session.set_alliance("Red1")          # optional; shim defaults to whatever DS had
    session.enable_teleop()               # or session.enable_auto("RIGHT")
    session.wait_state("IDLE", timeout=5.0)

    session.set_sticks(left_y=-0.6)       # WPILib Xbox conventions: -1 = full forward
    session.wait_pose_moved(1.0, timeout=10.0)
    session.set_sticks(left_y=0.0)

    session.set_sticks(right_trigger=1.0) # LT/RT are AXES; bindings fire at >= 0.5
    session.wait_state("PREPARE_SCORE", "SCORE", "PREPARE_FEED", "FEED", timeout=5.0)
    session.set_sticks(right_trigger=0.0)

    session.set_buttons(back=True)        # zero gyro; buttons only fire while ENABLED
    session.set_buttons(back=False)
    session.disable()
```

Run it: `uv run --package sim-agent sim-agent path/to/my_scenario.py`

`SimSession` API (see `shared/src/main/python/sim_agent/src/sim_agent/session.py`):

| Method | Purpose |
| --- | --- |
| `enable_teleop()` / `enable_test()` / `enable_auto(name)` / `disable()` | DriverStation control. `enable_auto` also selects the auto on the chooser. |
| `select_auto(name)` / `set_alliance("Red1")` | Chooser + alliance station. |
| `set_sticks(left_x=, left_y=, right_x=, left_trigger=, right_trigger=)` | Raw axis values −1..1; `None` = unchanged. **Forward is `left_y=-1`** (stick-up convention). |
| `set_buttons(a=, b=, x=, y=, lb=, rb=, back=, start=, left_stick=, right_stick=)` | Buttons fire only while enabled. Driver bindings: RT score/feed, LT intake, LB eject, RB idle, Back zero gyro. |
| `zero_inputs()` | Center sticks, release buttons. |
| `get(path, kind)` / `getter(path, kind)` | Read a topic once / build a poller. Kinds: `bool`, `double`, `string`, `double_array`, `pose2d`, `chassis_speeds`. |
| `wait_for(path, kind, predicate, timeout=, description=)` | 50 Hz poll until predicate true; raises `AssertionError` on timeout. |
| `wait_state(*states, timeout=)` | Wait for `RobotManager/State`. |
| `wait_pose_moved(meters, timeout=)` | Wait until `Localization/EstimatedPose` moved N meters. |
| `wait_pose_settled(tolerance_m=, window_s=, timeout=)` | Wait until the pose holds still — call before `wait_pose_moved` so the disabled boot/auto-start pose reset teleport isn't mistaken for driving. |
| `wait_robot_enabled(timeout=)` | Wait for the shim's `/Agent/Status/RobotEnabled` echo (the enable write has actually landed robot-side). |
| `start_recording({path: kind}, hz=50)` / `stop_recording(path)` | Background sampler → JSON artifact. |
| `time()` / `runs_dir()` | Seconds since handshake / artifact directory. |

Scenario tips:

- Always `zero_inputs()` before changing DS mode; stale triggers surprise you.
- On boot the disabled pose reset teleports `EstimatedPose` to the selected auto's start pose up to
  ~1s after the handshake. Before asserting on robot-driven movement, call `wait_robot_enabled()` +
  `wait_pose_settled()` (or use `enable_auto`, which does both) so the teleport isn't counted.
- Give mechanisms time: state machines transition on later loops (20 ms loop, debouncers up to
  0.25 s). Prefer `wait_*` predicates over fixed sleeps.
- Tunables are writable NT entries: publish to `/Tunable/<key>` (e.g.
  `/Tunable/Swerve/StuckOnBallBackoffSpeed`) via raw pyntcore if a scenario needs non-defaults.
- `pose2d` values decode to `(x_m, y_m, heading_deg)`; `chassis_speeds` to `(vx, vy, omega_rad)`.

## The /Agent NT contract

Published by the agent (consumed by `AgentSimShim`):

| Topic | Type | Notes |
| --- | --- | --- |
| `/Agent/Enabled` | bool | Master switch. False → shim inert; on falling edge the shim forces the robot disabled and zeroes the joystick once. |
| `/Agent/Joystick/Axes` | double[6] | 0 LeftX, 1 LeftY, 2 LT, 3 RT, 4 RightX, 5 RightY. Clamped to −1..1. |
| `/Agent/Joystick/Buttons` | bool[10] | 0 A, 1 B, 2 X, 3 Y, 4 LB, 5 RB, 6 Back, 7 Start, 8 LStick, 9 RStick. |
| `/Agent/DriverStation/Enabled` | bool | |
| `/Agent/DriverStation/Mode` | string | `"teleop"`, `"autonomous"`, or `"test"`. |
| `/Agent/DriverStation/AllianceStation` | string | `"Red1"`…`"Blue3"`; empty = leave unchanged. |

Published by the shim: `/Agent/Status/Active` (bool) — the session handshake waits for this —
and `/Agent/Status/RobotEnabled` (bool) — the robot's real `DriverStation.isEnabled()` echo, so the
agent can tell when its enable writes have landed (robot-side NT setup lags a fresh connection by
~0.5-1s).

The auto chooser is robot-side: write `/SmartDashboard/Autos/SelectedAuto/selected` (string,
e.g. `"DO_NOTHING"`, `"LEFT"`, `"RIGHT"`). `SimSession.select_auto` does this for you.

## Key robot topics to watch

All under `/Robot` (DogLog NT publishing is on in dev mode):

- `RobotManager/State`, `Swerve/State`, `HopperManager/State`, `Autos/State` (strings)
- `Localization/EstimatedPose` (struct:Pose2d), `Swerve/FieldRelativeSpeeds`,
  `Swerve/RobotRelativeSpeeds` (struct:ChassisSpeeds)
- `Imu/Main/Pitch`, `Imu/Main/Roll`, `Imu/BeachedRecovery/StuckOnBall`,
  `Imu/BeachedRecovery/RecoveryPose`
- `SuperstructureVisualization/Components` (Pose3d[]) — what AdvantageScope's 3D view renders
- Faults: `DogLog.logFault` entries; DS data (enable, joysticks) is captured into the wpilog

## Debugging workflow

1. Reproduce with a scenario; keep the failing recording JSON and note the `.wpilog` timestamp in
   `comp-bot/logs/` (newest file; the sim writes `FRC_YYYYMMDD_HHMMSS.wpilog`).
2. For deep analysis of the `.wpilog` (entries, struct payloads, DS flags like `/DS:enabled`),
   use the **analyze-wpilog** skill.
3. For visual debugging, a human can open the `.wpilog` in AdvantageScope
   (`open -a AdvantageScope <file>`) or watch live via "Connect to Simulator" — the agent's
   inputs show up exactly like a human driver's.
4. Iterate: fix code → `./gradlew spotlessApply comp-bot:build` → re-run the scenario.

## Troubleshooting

- **NT connect timeout**: sim didn't start or crashed — read the newest
  `comp-bot/logs/agent-runs/sim-stdout-*.log`. First run compiles; give it minutes.
- **Handshake timeout** (`/Agent/Status/Active` never true): robot code predates `AgentSimShim`,
  or the sim JVM crashed after NT came up (check stdout log, `hs_err_pid*` files).
- **Stale sim processes**: `pkill -f 'comp-bot-.*\.jar'` (the session already tries killpg +
  pkill; double-check if a run was interrupted).
- **Buttons do nothing**: robot must be *enabled* first (bindings are gated on enable), and
  LT/RT are axes — use `set_sticks(left_trigger=...)`, not buttons.
- **Robot won't move**: sticks are raw Xbox values — forward is `left_y=-1`, not `+1`.
- **Human wants the GUI**: launch `./gradlew comp-bot:simulateJava` without `SIM_HEADLESS` and
  use `--attach`; the agent wins over the Sim GUI DriverStation while `/Agent/Enabled` is true,
  and control returns to the human when the session ends. See `docs/simulation.md` for keyboard
  mappings.
