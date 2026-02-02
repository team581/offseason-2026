package frc.robot.deploy;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.math.MathHelpers;
import com.team581.simkit.SimKit;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.util.tuning.TunablePid;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.util.scheduling.SubsystemPriority;

public class Deploy extends StateMachineSubsystem<DeployState> {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final PositionVoltage positionVoltageRequest =
      new PositionVoltage(0).withEnableFOC(false);
  private DeployState storedState = DeployState.UNHOMED;
  private double leftMotorPosition = 0.0;
  private double rightMotorPosition = 0.0;

  public Deploy(TalonFX leftMotor, TalonFX rightMotor) {
    super(SubsystemPriority.DEPLOY, DeployState.UNHOMED);
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    leftMotor.getConfigurator().apply(DeployConfig.LEFT_MOTOR_CONFIG);
    rightMotor.getConfigurator().apply(DeployConfig.RIGHT_MOTOR_CONFIG);

    TunablePid.register("Deploy/Left", leftMotor, DeployConfig.LEFT_MOTOR_CONFIG);
    TunablePid.register("Deploy/Right", rightMotor, DeployConfig.RIGHT_MOTOR_CONFIG);
  }

  public void intakeRequest() {
    setStateFromRequest(DeployState.INTAKE);
    switch (getState()) {
      case UNHOMED, HOMING, CATCHUP_TO_LEFT, CATCHUP_TO_RIGHT -> {
        // Do nothing, we aren't homed or need to catchup
      }
      default -> setStateFromRequest(DeployState.INTAKE);
    }
  }

  public void stowRequest() {
    setStateFromRequest(DeployState.STOWED);
    switch (getState()) {
      case UNHOMED, HOMING, CATCHUP_TO_LEFT, CATCHUP_TO_RIGHT -> {
        // Do nothing, we aren't homed or need to catchup
      }
      default -> setStateFromRequest(DeployState.STOWED);
    }
  }

  public void homingRequest() {
    setStateFromRequest(DeployState.HOMING);
  }

  @Override
  protected void beforeTransition(DeployState oldState, DeployState newState) {
    if (newState == DeployState.CATCHUP_TO_LEFT || newState == DeployState.CATCHUP_TO_RIGHT) {
      storedState = getState();
    }
  }

  @Override
  protected DeployState getNextState(DeployState currentState) {
    return switch (currentState) {
      case HOMING -> {
        if (leftMotor.getStatorCurrent().getValueAsDouble() > DeployConfig.HOMING_CURRENT
            && rightMotor.getStatorCurrent().getValueAsDouble() > DeployConfig.HOMING_CURRENT) {
          leftMotor.setPosition(DeployConfig.HOMING_END_POSITION);
          rightMotor.setPosition(DeployConfig.HOMING_END_POSITION);
          yield DeployState.STOWED;
        } else {
          yield currentState;
        }
      }
      case CATCHUP_TO_LEFT, CATCHUP_TO_RIGHT -> {
        if (MathUtil.isNear(leftMotorPosition, rightMotorPosition, 1)) {
          yield storedState;
        }
        yield currentState;
      }

      default -> currentState;
    };
  }

  private static double clamp(double deployLength) {
    return MathUtil.clamp(deployLength, DeployConfig.MIN_LENGTH, DeployConfig.MAX_LENGTH);
  }

  @Override
  protected void afterTransition(DeployState newState) {
    switch (newState) {
      case UNHOMED -> {
        leftMotor.disable();
        rightMotor.disable();
      }
      case HOMING -> {
        leftMotor.setVoltage(DeployConfig.HOMING_VOLTAGE);
        leftMotor.setVoltage(DeployConfig.HOMING_VOLTAGE);
      }
      case CATCHUP_TO_LEFT -> {
        leftMotor.disable();
        rightMotor.setControl(positionVoltageRequest.withPosition(leftMotorPosition));
      }
      case CATCHUP_TO_RIGHT -> {
        leftMotor.setControl(positionVoltageRequest.withPosition(rightMotorPosition));
        rightMotor.disable();
      }
      default -> {
        leftMotor.setControl(positionVoltageRequest.withPosition((clamp(newState.getLength()))));
        rightMotor.setControl(positionVoltageRequest.withPosition((clamp(newState.getLength()))));
      }
    }
  }

  @Override
  protected void whileInState(DeployState state) {
    if (!MathUtil.isNear(leftMotorPosition, rightMotorPosition, 1)) {
      DogLog.logFault("DEPLOY MOTORS NOT ALIGNED", AlertType.kError);
      if (leftMotorPosition > rightMotorPosition) {
        setStateFromRequest(DeployState.CATCHUP_TO_LEFT);
      } else {
        setStateFromRequest(DeployState.CATCHUP_TO_RIGHT);
      }
    }
    DogLog.clearFault("DEPLOY MOTORS NOT ALIGNED");
  }

  public double getPosition() {
    return MathHelpers.average(leftMotorPosition, rightMotorPosition);
  }

  @Override
  protected void collectInputs() {
    leftMotorPosition = leftMotor.getPosition().getValueAsDouble();
    rightMotorPosition = rightMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void simulationPeriodic() {
    var leftDeploySimulation =
        SimKit.positionMechanism(
            "Deploy/Left",
            mechanism ->
                mechanism
                    .addMotor(leftMotor)
                    .withMinPosition(DeployConfig.MIN_LENGTH)
                    .withMaxPosition(DeployConfig.MAX_LENGTH));
    var rightDeploySimulation =
        SimKit.positionMechanism(
            "Deploy/Right",
            mechanism ->
                mechanism
                    .addMotor(rightMotor)
                    .withMinPosition(DeployConfig.MIN_LENGTH)
                    .withMaxPosition(DeployConfig.MAX_LENGTH));

    if (getState() == DeployState.HOMING) {
      leftMotor.setPosition(DeployConfig.HOMING_END_POSITION);
      rightMotor.setPosition(DeployConfig.HOMING_END_POSITION);
      setStateFromRequest(DeployState.STOWED);
    }

    leftDeploySimulation.update();
    rightDeploySimulation.update();
  }
}
