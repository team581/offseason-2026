package frc.robot.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.DoubleSubscriber;
import frc.robot.util.scheduling.SubsystemPriority;

public class Intake extends StateMachineSubsystem<IntakeState> {
  private final TalonFX intakeMotor;
  private final TalonFX hopperMotor;

  private final LinearFilter currentFilterHopper = LinearFilter.movingAverage(5);
  private double rawCurrentHopper = 0.0;

  private double filteredCurrentHopper = 0.0;

  private static final DoubleSubscriber JAM_CURRENT_THRESHOLD =
      DogLog.tunable("HOPPER/JamCurrentThreshold", 75.0);

  public Intake(TalonFX intakeMotor, TalonFX hopperMotor) {
    super(SubsystemPriority.INTAKE, IntakeState.IDLE);
    intakeMotor
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
    hopperMotor
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
    this.hopperMotor = hopperMotor;
    this.intakeMotor = intakeMotor;
  }

  @Override
  protected void afterTransition(IntakeState newState) {
    switch (newState) {
      case UNTUNED, STOPPED -> {
        intakeMotor.disable();
        hopperMotor.disable();
      }
      default -> {
        intakeMotor.setVoltage(getState().volts);
        hopperMotor.setVoltage(getState().volts);
      }
    }
  }

  @Override
  protected void collectInputs() {
    rawCurrentHopper = hopperMotor.getStatorCurrent().getValueAsDouble();
    filteredCurrentHopper = currentFilterHopper.calculate(rawCurrentHopper);
  }

  public boolean isJammed() {
    return filteredCurrentHopper > JAM_CURRENT_THRESHOLD.getAsDouble();
  }

  public void intakeRequest() {
    setStateFromRequest(IntakeState.INTAKING);
  }

  public void idleRequest() {
    setStateFromRequest(IntakeState.IDLE);
  }
}
