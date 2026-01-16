package frc.robot.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team581.util.state_machines.StateMachineSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class Intake extends StateMachineSubsystem<IntakeState> {
  private final TalonFX motor;

  public Intake(TalonFX motor) {
    super(SubsystemPriority.INTAKE, IntakeState.IDLE);
    motor
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                // TODO:Get sensor to mechanism ratio
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(0))
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(100)
                        .withSupplyCurrentLimit(100))
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast)));
    this.motor = motor;
  }

  @Override
  protected void afterTransition(IntakeState newState) {
    switch (newState) {
      case UNTUNED, STOPPED -> motor.disable();
      default -> motor.setVoltage(getState().volts);
    }
  }

  public void intakeRequest() {
    setStateFromRequest(IntakeState.INTAKING);
  }

  public void idleRequest() {
    setStateFromRequest(IntakeState.IDLE);
  }
}
