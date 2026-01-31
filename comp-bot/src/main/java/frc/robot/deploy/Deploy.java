package frc.robot.deploy;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.simkit.SimKit;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.util.tuning.TunablePid;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.util.scheduling.SubsystemPriority;

public class Deploy extends StateMachineSubsystem<DeployState> {
  // TODO: These are just place holders we should update these when we do find out these position
  private final TalonFX motor;
  private final PositionVoltage positionVoltageRequest =
      new PositionVoltage(0).withEnableFOC(false);

  // TODO: Find position eventually

  public Deploy(TalonFX motor) {
    super(SubsystemPriority.DEPLOY, DeployState.UNHOMED);
    this.motor = motor;

    motor.getConfigurator().apply(DeployConfig.MOTOR_CONFIG);

    TunablePid.register("Deploy", motor, DeployConfig.MOTOR_CONFIG);
  }

  public void intakeRequest() {
    setStateFromRequest(DeployState.INTAKE);
    switch (getState()) {
      case UNHOMED, HOMING -> {
        // Do nothing, we aren't homed
      }
      default -> setStateFromRequest(DeployState.INTAKE);
    }
  }

  public void stowRequest() {
    setStateFromRequest(DeployState.STOWED);
    switch (getState()) {
      case UNHOMED, HOMING -> {
        // Do nothing, we aren't homed
      }
      default -> setStateFromRequest(DeployState.STOWED);
    }
  }

  public void homingRequest() {
    setStateFromRequest(DeployState.HOMING);
  }

  @Override
  protected DeployState getNextState(DeployState currentState) {
    return switch (currentState) {
      case HOMING -> {
        if (motor.getStatorCurrent().getValueAsDouble() > 20) {
          motor.setPosition(
              0); // TODO: reset the encoder to a homed position (this is some position we dont know
          // yet)
          yield DeployState.STOWED;
        } else {
          yield currentState;
        }
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
      case UNHOMED -> motor.disable();
      case HOMING -> motor.setVoltage(DeployConfig.HOMING_VOLTAGE);
      default ->
          motor.setControl(
              positionVoltageRequest.withPosition(
                  Units.degreesToRotations(clamp(DeployConfig.HOMING_END_POSITION))));
    }
  }

  public double getPosition() {
    return Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    var deploySimulation =
        SimKit.positionMechanism(
            "Deploy",
            mechanism ->
                mechanism
                    .addMotor(motor)
                    .withMinPosition(DeployConfig.MIN_LENGTH)
                    .withMaxPosition(DeployConfig.MAX_LENGTH));

    if (getState() == DeployState.HOMING) {
      motor.setPosition(0);
      setStateFromRequest(DeployState.STOWED);
    }

    deploySimulation.update();
  }
}
