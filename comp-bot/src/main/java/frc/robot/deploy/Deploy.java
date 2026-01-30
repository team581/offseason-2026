package frc.robot.deploy;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.util.tuning.TunablePid;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.util.scheduling.SubsystemPriority;

public class Deploy extends StateMachineSubsystem<DeployState> {
  private static final double MAX_ANGLE = 90;
  private static final double MIN_ANGLE = 0;
  // TODO: These are just place holders we should update these when we do find out these angle
  private final TalonFX motor;
  private final PositionVoltage positionVoltageRequest =
      new PositionVoltage(0).withEnableFOC(false);
  private final double homingEndAngle = 0;
  private final double homingVoltage = 0;

  // TODO: Find angle eventually

  public Deploy(TalonFX motor) {
    super(SubsystemPriority.DEPLOY, DeployState.UNHOMED);
    this.motor = motor;

    var configs =
        new TalonFXConfiguration()
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1))
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs().withStatorCurrentLimit(1).withStatorCurrentLimit(1))
            .withSlot0(new Slot0Configs().withKP(0).withKV(0).withKG(0));
    motor.getConfigurator().apply(configs);

    TunablePid.register("Deploy", motor, configs);
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
              0); // TODO: reset the encoder to a homed position (this is some angle we dont know
          // yet)
          yield DeployState.STOWED;
        } else {
          yield currentState;
        }
      }
      default -> currentState;
    };
  }

  private static double clamp(double deployAngle) {
    return MathUtil.clamp(deployAngle, MIN_ANGLE, MAX_ANGLE);
  }

  @Override
  protected void afterTransition(DeployState newState) {
    switch (newState) {
      case UNHOMED -> motor.disable();
      case HOMING -> motor.setVoltage(homingVoltage);
      default ->
          motor.setControl(
              positionVoltageRequest.withPosition(Units.degreesToRotations(clamp(homingEndAngle))));
    }
  }
}
