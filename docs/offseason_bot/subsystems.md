# Subsystems

## Intake Deploy

- 1 Kraken X44 on CANivore CAN **Needs update.**
- MotionMagicVoltage control loop
- Rotational movement
  - Folds out
- Gear ratio 1:1 **Needs update.**
- States
  - Unhomed
  - Homing
    - Current based, homes inward during auto
    - In teleop homes outward
  - Stowed
  - Intaking

## Swerve

- CANivore Bus **Needs update.**
- Ratio - Swerve X2c X1 ([ratio cheatsheet](https://docs.wcproducts.com/welcome/frc-build-system/gearboxes/swerve#drive-ratios)) **Needs update.**
  - 23.5” width between wheels **Needs update.**
  - 22” length between wheels **Needs update.**
  - comp bot has 10T pinion, practice has 11T pinion **Needs update.**
- Kraken X60 drive motors, Kraken X44 steer motors **Needs update.**

## Intake

- Rio CAN Bus
- Uses rollers to grab fuel, extended via intake deploy
- 2 Kraken X60 motor - Open loop control for roller **Needs update.**
- Gear ratio: 1:1 **Needs update.**
- States
  - Intaking
  - Stopped/idle
  - Shooting
    - This is basically the same as intaking but lower voltage
    - Shoot and then intake (sets state to intaking after stopped shooting)
  - Eject

## Conveyor/Floor

- Rio CAN Bus
- 2x Kraken x44 - Open loop control **Needs update.**
- Gear ratio: 1:1 **Needs update.**
- Mechanically pivots upward when deploy is brought in **Needs update.**
- Needs to roll when we’re ready to shoot
- States
  - Shooting
  - Ball filling
  - Eject
  - Idle/Stopped

## Funneler

- Rio CAN Bus
- ? Kraken x? - Open loop control **Needs update.**
- Ratio: ?:? **Needs update.**
- Funnels balls toward the feeder
- Needs to roll when we’re ready to shoot
- States
  - Shooting
  - Ball filling? **Needs update.**
  - Idle/Stopped

## Feeder

- Rio CAN Bus
- ? Kraken x? - Open loop control **Needs update.**
- Ratio: ?:? **Needs update.**
- Rolls balls up when we’re ready to shoot
- States
  - Shooting
  - Ball filling **Needs update.**
  - Idle/Stopped

## Shooter Hood

- Rio CAN Bus
- Pivot mechanism for controlling fuel exit angle
- ? Kraken X? - PositionVoltage control loop **Needs update.**
- Angle should represent exit angle (pivot to top of hood)
- Gear ratio: ?:? **Needs update.**
- States
  - Unhomed
  - Homing
    - Current based, runs in auto
  - Feeding
    - Interpolation table of distance -> angle
  - Scoring
    - Interpolation table of distance -> angle
  - Stowed (0°)

## Turret

- Rio CAN bus
- PositionVoltage loop

**Multiple states:**

- Unhomed
- Idle Score (aiming at hub) **Needs update.**
- Idle Feed (aiming at feed spot) **Needs update.**
- Feeding (aims at feed spot)
- Scoring (aims at hub)
- Stuck? **Needs update.**

Mechanism: **Needs update.**

- ? Kraken X? **Needs update.**
  - Gearing is ?:? **Needs update.**

## Shooter

- Rio CAN bus
- Only really cares about velocity (RPM)
- Velocity voltage control loop **Needs update.**

**Multiple states:**

- Idle (no parameters)
- Hub scoring (needs to know distance, which is converted to an RPM)
- Feeding (needs to know distance, same as scoring)

Mechanism: **Needs update.**

- ? Kraken X? **Needs update.**
  - Gearing is ?:? **Needs update.**

## Robot Manager

- Scoring states + fallbacks
- Feeding states + fallbacks
  - Support feeding to multiple poses on field **Needs update.**
  - Operator selects feed location during teleop **Needs update.**
- Idle state
- Handles some requests for PowerManager
- Intake controlled independently from rest of robot
- Unjam state
- Active hub state tracking

## Localization & Vision

- ? Limelights **Needs update.**
