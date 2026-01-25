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
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;

  public Intake(TalonFX leftMotor, TalonFX rightMotor) {
    super(SubsystemPriority.INTAKE, IntakeState.IDLE);
    leftMotor
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
    rightMotor
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
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
  }

  @Override
  protected void afterTransition(IntakeState newState) {
    switch (newState) {
      case UNTUNED, IDLE -> {
        leftMotor.disable();
        rightMotor.disable();
      }
      default -> {
        leftMotor.setVoltage(getState().volts);
        rightMotor.setVoltage(getState().volts);
      }
    }
  }

  @Override
  protected void whileInState(IntakeState state) {
    DogLog.log("Intake/LeftMotor/StatorCurrent", leftMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Intake/RightMotor/StatorCurrent", rightMotor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Intake/LeftMotor/SupplyCurrent", leftMotor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Intake/RightMotor/SupplyCurrent", rightMotor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Intake/LeftMotor/Velocity", leftMotor.getVelocity().getValueAsDouble());
    DogLog.log("Intake/RightMotor/Velocity", rightMotor.getVelocity().getValueAsDouble());
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
