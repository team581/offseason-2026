# Writing subsystems

This file contains guidelines and best practices for writing subsystems.

## Code organization

### Mechanism subsystems

- Subsystem code should be stored in a directory under `src/main/java/frc/robot/`
  - ex. `src/main/java/frc/robot/shooter/`
- Subsystems should be named `<mechanism>.java`
  - ex. `Shooter.java`
- State enums should be named `<mechanism>State.java`
  - ex. `ShooterState.java`

### Managers

- Manager code should be stored in a directory under `src/main/java/frc/robot/`
  - ex. `src/main/java/frc/robot/robot_manager/`
- Manager subsystems should be named `<scope>Manager.java`
  - ex. `RobotManager.java`
- State enums should be named `<scope>State.java`
  - ex. `RobotState.java`

## State machines

- Subsystems should extend `StateMachineSubsystem` for determinstic logic and standardized code structure
  - ex. `public class ArmSubsystem extends StateMachineSubsystem<ArmState>`
- State machines should almost never include additional state outside of the state enum
  - ex. `RobotManager` needs to track when the robot is climbing to ignore state requests when hanging
    - Whether the robot is climbing should be derived from `RobotState`
    - There should not be a separate field within `RobotManager` to track whether we are climbing

### State enums

- State enums should follow naming convention when possible
  - ex. `STOWED` for the idle state of a mechanism using position control
  - ex. `IDLE` for the stopped state of an roller/flywheel mechanism
- States in a known sequence should include a number of their position
  - ex. A climbing sequence that uses the states `CLIMBING_1_LINEUP`, `CLIMBING_2_GRAB`, and `CLIMBING_3_HANGING`

### State transitions

- Subsystems should not automatically transition from one state to another
  - With the exception of subsystems requiring homing
    - ex. `HOMING` -> automatic transition -> `STOWED`

## Passing in hardware

- Subsystems should accept all necessary hardware as parameters to their constructor
  - ex. `public Serializer(TalonFX leftMotor, TalonFX rightMotor)`
  - ex. `public Intake(TalonFX motor, DigitalInput sensor)`
- Subsystems should keep all instances of hardware private
  - You almost never want to expose the underlying hardware instance itself, this breaks the ownership pattern that subsystems have over their hardware

## Logging

- Logged values should usually be recorded in `whileInState()`
- Subsystems should always log important values for logic
  - ex. An intake subsystem would log whether it detects a game piece since that's a critical part of its functionality
    - The intake subsystem likely doesn't need to log its velocity, since that's not relevant to intaking most of the time
- Prior to bringup, subsystems should log helpful values for first-time testing
  - ex. An intake subsystem has never been tested before, so it should include logs for applied voltage and stator current draw
    - These values introduce some overhead to retrieve and log, but are useful during first-time tuning
    - You can use `GlobalConfig.IS_DEVELOPMENT` to selectively log values depending on whether you're at home or competing

## Homing routines

- Mechanisms that require homing should start in an unhomed state
- Mechanisms in a unhomed or homing state should reject state requests to exit the homing states
  - ex. An elevator subsystem that is in the middle of its homing routine shouldn't cancel it because it received a `stowRequest()`

### Current based

- Subsystems using current draw to home will apply a low amount of voltage until a spike in stator current is detected
  - An increase in stator current typically indicates you have hit a hardstop
- Once a sustained, elevated stator current draw is detected, the subsystem should reset its internal encoder position to a known value
  - ex. A climber subsystem applies 2V to the motor until stator current exceeds 20A
    - Once the current draw is recorded, it automatically transitions out of the homing state to an idle state
- When necessary, a moving average filter should be used to reduce the noise of the stator current measurement
  - Alternatively, increasing the minimum current can act as a way to filter out false positives
