package frc.robot.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import frc.robot.util.scheduling.SubsystemPriority;

public class Intake extends StateMachineSubsystem<IntakeState> {
  private final TalonFX motor;

  public Intake(TalonFX motor) {
    super(SubsystemPriority.INTAKE, IntakeState.IDLE);
    motor
        .getConfigurator()
        .apply(
            new TalonFXConfiguration()
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(0))
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(100)
                        .withSupplyCurrentLimit(100))
                .withMotorOutput(
                    new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Coast)
                        .withInverted(InvertedValue.Clockwise_Positive)));
    this.motor = motor;
  }

  @Override
  protected void afterTransition(IntakeState newState) {
    switch (newState) {
      case UNTUNED, IDLE -> motor.disable();
      default -> motor.setVoltage(getState().volts);
    }
  }

  @Override
  protected void whileInState(IntakeState state) {
    DogLog.log("Intake/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Intake/SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Intake/Velocity", motor.getVelocity().getValueAsDouble());
  }

  public void intakeRequest() {
    setStateFromRequest(IntakeState.INTAKING);
  }

  public void shootingRequest() {
    setStateFromRequest(IntakeState.SHOOTING);
  }

  public void idleRequest() {
    setStateFromRequest(IntakeState.IDLE);
  }
}
