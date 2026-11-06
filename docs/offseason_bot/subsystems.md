# Subsystems

## Intake Deploy

- 2 Kraken X44s on CANivore (`581CANivore`): leader 15, follower 30; ratio `36.57:1`
- Arm angle: 0° fully down, initially 88.4° collapsed; tune during bringup
- Control: Motion Magic Torque Current FOC with Position Torque Current FOC for side-to-side sync
- All PID/feedforward gains start at zero, including the desync override
- Coast when both sides are within 5° of down. Recover after either side stays above 10° for 3 seconds
- Recovery timeout (5s) or desync (>15°) latches coast until another command
- Stow, compaction, homing, and desync override act immediately; disabling resets the recovery delay
- Motion limits: 90°/s, 180°/s²; tune targets and gains on the robot

```mermaid
flowchart LR
    INTAKE -->|Down| INTAKE_COAST
    INTAKE_COAST -->|Raised| INTAKE_COLLAPSED
    INTAKE_COLLAPSED -->|Falls back| INTAKE_COAST
    INTAKE_COLLAPSED -->|3 seconds raised| INTAKE_RECOVERING
    INTAKE_RECOVERING -->|Down| INTAKE_COAST
    INTAKE_RECOVERING -->|Timeout or desync| INTAKE_RECOVERY_FAILED
```

- Other states: `UNHOMED`, `STOW`, `SCORE_COMPACTION_WAITING`, `SCORE_COMPACTION`,
  `FEED_COMPACTION`, `BEAST_MODE_COMPACTION`, `FIX_DIFFERENTIAL_DESYNC`
- Homing: `HOME_INWARD` in auto, `HOME_OUTWARD` in teleop; both use current detection

## Swerve

- CANivore Bus (`581CANivore`)
- Kraken X60 drive motors, Kraken X44 steer motors
- Pigeon 2 IMU (ID 2)
- Swerve X2c X1 ratio with 10T pinion ([ratio cheatsheet](https://docs.wcproducts.com/welcome/frc-build-system/gearboxes/swerve#drive-ratios))
- Closed-loop voltage control on drive and steer
- Motion Magic Expo for slow steer movement when locking X-swerve

## Intake

- Rio CAN Bus
- 2 Kraken X60 motors (Left 16, Right 17)
- Gear Ratio: `12:20` (0.6)
- Rollers: 2" diameter + cat tongue tape
- Open-loop voltage control with FOC
- States:
  - `IDLE`: 0V
  - `INTAKE`: 12V
  - `SCORE`: 5V (lower voltage while shooting)
  - `FEED`: 5V (lower voltage while feeding)
  - `EJECT`: -12V

## Conveyor/Floor

- CANivore Bus (`581CANivore`): conveyor ID 18, tunnel top ID 20, tunnel bottom ID 21
- Open-loop voltage control with FOC
- Telemetry is grouped under `Floor/` and `Tunnel/`; shared state and voltage requests remain under `Conveyor/`.
- Tunnel surface speeds are directly under `Tunnel/TopSurfaceSpeedMetersPerSecond` and `Tunnel/BottomSurfaceSpeedMetersPerSecond`. Set `Tunnel/MetersPerPulleyRevolution` to measured belt travel per drive-pulley revolution; until configured, speeds are NaN and `Tunnel/SurfaceSpeedCalibrated` is false.
- Mechanically pivot upward when deploy retracts
- Mechanisms:
  - **Floor**: 1 Kraken X60, 1.5" rollers + cat tongue, Ratio `(14/60)*(16/18)*(16/20)*(17/20)^2` (~8.35:1 reduction)
  - **Tunnel (Belts)**: 2 Kraken X60s (opposed spin), Ratio `12:30` (2.5:1)
- States:
  - `IDLE`: 0V
  - `SCORE` / `FEED` / `INITIAL_SHOT`: 10V
  - `BALL_FILLING`: 1V
  - `INTAKE`: 2V
  - `EJECT`: -12V

## Funneler

- CANivore Bus (`581CANivore`), 1 Kraken X60 (ID 27)
- Gear Ratio: `16:30` (1.875:1)
- 2" Fairlane wheels
- Open-loop voltage control with FOC
- Move balls sideways from hopper into tunnel
- States: `IDLE`, `SCORE` (10V), `FEEDING`, `INTAKE`, `BALL_FILLING`

## Feeder

- CANivore Bus (`581CANivore`), 1 Kraken X60 (ID 19)
- Gear Ratio: `18:28` (1.556:1)
- 2" flex wheels
- Open-loop voltage control with FOC
- Roll balls up from tunnel into shooter
- States:
  - `IDLE`: 0V
  - `SCORE` / `FEED`: 10V
  - `BALL_FILLING`: 1V
  - `INTAKING`: -1V
  - `EJECT`: -6V

## Hopper Manager

- Coordinator subsystem managing intake deploy, intake rollers, floor & tunnel conveyors, funneler, and tower feeder
- Sensors:
  - DigitalInput tower beam-break sensor (Rio DIO 9) to detect fuel presence at tower entrance
  - CANrange removed (ball presence in hopper inferred from intake run duration: >3s intaking indicates full hopper)
- States:
  - `IDLE_DEPLOYED`: Deploy extended, intake idle; fills ball into tower if intake has been running and tower is empty
  - `IDLE_STOWED`: Deploy stowed; all transport mechanisms idle
  - `INTAKING`: Deploy extended, intake spinning forward (12V); runs conveyor and funneler into tower, backs feeder slightly (-1V) until filling conditions met
  - `EJECTING`: Deploy extended, intake (-12V), conveyor (-12V), and feeder (-6V) reversed to dump balls out the front; funneler idle
  - `UNJAMMING`: Deploy extended; intake and conveyor reverse using their outtake requests (-12V). Feeder and funneler use their shooting requests (10V); shooter uses its normal scoring request at a fixed 3m distance, with the normal RPM limit and pit-mode override. These requests stay active until unjam is released, then normal idle/intake requests resume. The tunnel belts are part of Conveyor/Floor and have no separate control in the current code.
  - `SCORE`: Deploy extended; intake scores (5V); conveyor runs initial shot (10V) then score (10V); feeder (10V) and funneler run forward. Compaction runs after 0.25s delay with 0.25s shuffle interval, unless beast mode is active
  - `SCORE_AND_INTAKE`: Simultaneous scoring and intaking; compaction disabled while intaking
  - `FEED`: Deploy extended; intake feeds (5V); conveyor runs initial shot (10V) then feed (10V); feeder (10V) and funneler run forward. Feed compaction activates after 0.25s delay
  - `FEED_AND_INTAKE`: Simultaneous feeding and intaking

## Shooter Hood

- CANivore Bus (`581CANivore`), 1 Talon FX (ID 22)
- Control exit angle relative to horizontal (~11° base, 13° to 45° travel)
- Position Voltage control loop
- States:
  - `UNHOMED`
  - `HOMING`: Current-based homing against hardstop
  - `IDLE`: Stow low to clear trench
  - `SCORING` / `FEEDING`: Interpolation table or regression model from distance

## Turret

- CANivore Bus (`581CANivore`), 1 Kraken X44 (ID 28)
- Gear Ratio: `60:1`
- Position Voltage control loop with velocity feedforward
- Travel range: -270° to -90° (offset 4.75" back, 8.025" left)
- Before the first enable after boot, hand-set the turret to -180° within ±3° (half a motor turn at 60:1).
  Calibrate `MOTOR_ROTOR_CAL_OFFSET` from `Turret/Motor/RotorPosition` at exactly -180°.
  While disabled before first enable, homing uses the wrapped rotor delta to refine the hand-set position.
  Errors above 0.01° for 100 ms trigger a motor-position update; checking continues until enable.
  Power-cycle to home again after repositioning by hand. Later enables retain the position.
- States:
  - `HOMING`: Hold neutral and reject aiming requests until the motor position is seeded
  - `SCORE` / `FEED`: Aim at target using shortest path
  - `IDLE_SCORE` / `IDLE_FEED`: Aim at target while smartly unwrapping to avoid hardstops
  - `STUCK`: Fault state when obstructed; inhibit firing

## Shooter

- CANivore Bus (`581CANivore`), 2 Talon FXs (left 23, right 24); right leads, left follows opposed. IDs 25 and 26 are unassigned.
- Velocity Voltage control on hardware (FOC disabled), using beta competition-bot gains.
- Simulation retains its Torque Current FOC gains and timed initial feedforward boost. The current-based boost is not applied to the voltage request on hardware.
- States:
  - `IDLE`: ~400 RPM
  - `PREPARE_SCORE` / `SCORE`: Distance-based RPM setpoint
  - `PREPARE_FEED` / `FEED`: Distance-based RPM setpoint

## Robot Manager

- Top-level coordinator for swerve, hopper, turret, shooter, hood, and power
- States:
  - `IDLE`
  - `WARMUP_SCORE` / `PREPARE_SCORE` / `SCORE`
  - `PREPARE_FORCE_SCORE` / `FORCE_SCORE` (testing override)
  - `WARMUP_FEED` / `PREPARE_FEED` / `FEED`
  - `PREPARE_FALLBACK_SCORE` / `FALLBACK_SCORE`
  - `PREPARE_FALLBACK_FEED` / `FALLBACK_FEED`
  - `UNJAM`

```mermaid
stateDiagram-v2
    [*] --> IDLE

    IDLE --> PREPARE_SCORE: Shoot (In Alliance Zone)
    IDLE --> PREPARE_FEED: Shoot (Outside Alliance Zone)
    IDLE --> PREPARE_FALLBACK_SCORE: Shoot (Vision Lost / Disabled)
    IDLE --> PREPARE_FALLBACK_FEED: Feed (Vision Lost / Disabled)
    IDLE --> UNJAM: Unjam Pressed

    state Scoring {
        PREPARE_SCORE --> SCORE: Aiming & Flywheel At Goal, Hub Active
        SCORE --> PREPARE_SCORE: Conditions Drop / Tolerances Exceeded
    }

    state Feeding {
        PREPARE_FEED --> FEED: Aiming & Flywheel At Goal, Safe Feed Location
        FEED --> PREPARE_FEED: Conditions Drop / Tolerances Exceeded
    }

    state Fallbacks {
        PREPARE_FALLBACK_SCORE --> FALLBACK_SCORE: Fallback Aim & Flywheel At Goal
        FALLBACK_SCORE --> PREPARE_FALLBACK_SCORE: Conditions Drop
        PREPARE_FALLBACK_FEED --> FALLBACK_FEED: Fallback Aim & Flywheel At Goal
        FALLBACK_FEED --> PREPARE_FALLBACK_FEED: Conditions Drop
    }

    PREPARE_SCORE --> PREPARE_FEED: Leaves Alliance Zone
    PREPARE_FEED --> PREPARE_SCORE: Enters Alliance Zone

    Scoring --> IDLE: Shoot Released
    Feeding --> IDLE: Shoot Released
    Fallbacks --> IDLE: Shoot Released
    UNJAM --> IDLE: Unjam Released
```

- Dynamic Feeding:
  - Auto-select closest unblocked feeding spot (`LEFT`, `RIGHT`, `BACKUP_LEFT`, `BACKUP_RIGHT`)
  - Switch to backup corner target if Hub blocks shot path
- Shoot-on-the-Move:
  - Lead shots using robot pose, swerve velocity, and ball time-of-flight (see [SOTM math audit and log-based calibration](../sotm-tuning.md))
  - Gate firing on turret angle, shooter RPM, hood angle, trench safety, and localization health
- Fallbacks:
  - Default to hardcoded trench shots based on Driver Station position if vision fails
- Hub Timing:
  - Spin up and shoot before Hub activates using ball time-of-flight

## Localization & Vision

- Use Spot vision coprocessor (replaces Limelights)
- 4 static chassis-mounted cameras (`left`, `right`, `front`, `back`)
- Tag Tracking:
  - Broad mode: Track all field tags for pose estimation
  - Hub mode: Filter only active alliance Hub tags for fast, accurate target tracking

```mermaid
stateDiagram-v2
    [*] --> TAGS
    TAGS --> WAITING_FOR_HUB_TAGS: Hub Target Requested
    WAITING_FOR_HUB_TAGS --> HUB_TAGS: Fresh Hub Tags Acquired
    HUB_TAGS --> WAITING_FOR_HUB_TAGS: Hub Tags Lost / Stale
    WAITING_FOR_HUB_TAGS --> TAGS: Broad Mode / Alliance Changed
    HUB_TAGS --> TAGS: Broad Mode Requested
```

- Rejection Filters:
  - Reject on high robot tilt, field boundaries, high ambiguity, or large pose jumps
  - Compensate for capture latency using odometry history
- Odometry Fusion:
  - Fuse vision poses with distance- and ambiguity-scaled standard deviations
- Trust Factor:
  - Increase uncertainty over distance driven, time elapsed, and IMU collisions
  - Reduce uncertainty with valid tag measurements
  - Trigger fallback shot logic when uncertainty threshold is exceeded ("lost")
- Driver Station:
  - Toggle vision on/off (forces fallback shots when off)
  - Fault indicators for camera and localization health


## Bringup tuning

Deploy average gains remain under `Deploy/Left` and `Deploy/Right`. Difference
axis gains are exposed under `Deploy/Left/Slot1`, `Deploy/Right/Slot1` (normal
synchronization) and `Deploy/Left/Slot2`, `Deploy/Right/Slot2` (desync override).
Each exposes kP, kI, kD, kS, kV, kA, and kG; tune both motors consistently.

## Beta tuning baseline

Copied the competition configuration from `beta-bot` at `6645bd8d^` (the last
revision before that project was removed): shooter voltage PID/feedforward,
scoring and feeding RPM/hood tables, time of flight, idle RPM, preparation/feeding
tolerances, turret voltage PID/feedforward and voltage limits, hood PID/current
limits and tolerance, and intake stator current limits. Shooter current limits
and reverse-output limits also follow beta; PowerManager still applies its
state-dependent supply limits.

Offseason gear ratios, motor directions, CAN mapping, travel limits, homing,
geometry, swerve settings, simulation gains, and opposed shooter follower remain
unchanged. The active-scoring RPM tolerance remains 300 RPM. Beta's linear deploy
used different feedback units and voltage control, so its gains and motion limits
were not copied to the offseason rotating arm. Beta's dye rotor and climber have
no corresponding offseason mechanisms. These copied values are a hardware tuning
baseline; the simulation tests do not validate physical tuning on the new robot.
