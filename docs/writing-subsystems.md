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

- Subsystems should extend `StateMachineSubsystem` for deterministic logic and standardized code structure
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
  - ex. A climbing sequence that uses the states `CLIMB_1_LINEUP`, `CLIMB_2_RAISING`, and `CLIMB_3_HANGING`
- State enums can have associated values for configuration
  - ex. `IntakeState` with voltage values: `INTAKING(6), OUTTAKING(-6)`
  - ex. `RobotState` with boolean flags: `IDLE(false, false, false)` for `(climbingOrRehoming, intaking, shooting)`

### State transitions

- Subsystems should not automatically transition from one state to another
  - With the exception of subsystems requiring homing
    - ex. `HOMING` -> automatic transition -> `STOWED`
- Managers should use `getNextState()` for automatic transitions based on conditions
  - ex. `PREPARE_SHOOT` -> wait for `shooter.atGoal()` -> `SHOOT`

### State request methods

TODO(@jonahsnider): Document state requests for subsystems

### atGoal() methods

- Mechanisms that need to wait for a condition expose an `atGoal()` method
  - ex. `shooter.atGoal()` which returns whether the shooter is at the target velocity for its current state
- Implementation should check if the mechanism is at its target within a tolerance
  - ex. `MathUtil.isNear(goalAngle, currentAngle, TOLERANCE)`
- Managers use `atGoal()` to trigger automatic state transitions
  - ex. `PREPARE_SHOOT` -> wait for `shooter.atGoal()` -> `SHOOT`

## Passing in hardware

- Subsystems should accept all necessary hardware as parameters to their constructor
  - ex. `public Serializer(TalonFX leftMotor, TalonFX rightMotor)`
  - ex. `public Intake(TalonFX motor, DigitalInput sensor)`
- Subsystems should keep all instances of hardware private
  - You almost never want to expose the underlying hardware instance itself, this breaks the ownership pattern that subsystems have over their hardware
- Subsystems can accept other subsystems as dependencies
  - ex. `public Localization(Swerve swerve, TunerSwerveDrivetrain drivetrain, Vision vision)`

## Logging

- Logged values should usually be recorded in `whileInState()`
- Subsystems should always log important values for logic
  - ex. An intake subsystem would log whether it detects a game piece since that's a critical part of its functionality
    - The intake subsystem likely doesn't need to log its velocity, since that's not relevant to intaking most of the time
- Prior to bringup, subsystems should log helpful values for first-time testing
  - ex. An intake subsystem has never been tested before, so it should include logs for applied voltage and stator current draw
    - These values introduce some overhead to retrieve and log, but are useful during first-time tuning
    - You can use `GlobalConfig.IS_DEVELOPMENT` to selectively log values depending on whether you're at home or competing

### Fault logging

- Use `DogLog.logFault()` for reporting errors or warnings for the drivers to see
- Use `DogLog.clearFault()` when the condition is resolved
- State-dependent faults should be logged in `whileInState()`
  - ex. Log a fault when turret is `UNHOMED`, clear it in other states
    ```java
    switch (getState()) {
      case UNHOMED -> DogLog.logFault("Turret is not homed", AlertType.kError);
      default -> DogLog.clearFault("Turret is not homed");
    }
    ```

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
