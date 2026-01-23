package frc.robot.intake;

import static edu.wpi.first.units.Units.Volts;

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

    var config =
        new TalonFXConfiguration()
            // TODO: Fill in the real ratio here
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1))
            .withCurrentLimits(
                new CurrentLimitsConfigs().withStatorCurrentLimit(20).withSupplyCurrentLimit(20))
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));

    motor.getConfigurator().apply(config);
    this.motor = motor;
  }

  public void shootingRequest() {
    setStateFromRequest(IntakeState.SHOOTING);
  }

  public void intakeRequest() {
    setStateFromRequest(IntakeState.INTAKING);
  }

  public void idleRequestgRequest() {
    setStateFromRequest(IntakeState.IDLE);
  }

  @Override
  protected void afterTransition(IntakeState newState) {
    switch (newState) {
      case IDLE -> motor.disable();
      case INTAKING -> motor.setVoltage(12);
      case SHOOTING -> motor.setVoltage(12);
    }
  }
}
