package frc.robot.hopper;

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

public class Hopper extends StateMachineSubsystem<HopperState> {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;

  private final LinearFilter currentFilter = LinearFilter.movingAverage(5);
  private double rawCurrentLeft = 0.0;
  private double rawCurrentRight = 0.0;

  private double filteredCurrent = 0.0;

  private static final DoubleSubscriber JAM_CURRENT_THRESHOLD =
      DogLog.tunable("HOPPER/JamCurrentThreshold", 75.0);

  public Hopper(TalonFX leftMotor, TalonFX rightMotor) {
    super(SubsystemPriority.HOPPER, HopperState.IDLE);
    leftMotor
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
    rightMotor
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
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
  }

  @Override
  protected void afterTransition(HopperState newState) {
    switch (newState) {
      case UNTUNED, STOPPED -> {
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
  protected void collectInputs() {
    rawCurrentLeft = leftMotor.getStatorCurrent().getValueAsDouble();
    rawCurrentRight = rightMotor.getStatorCurrent().getValueAsDouble();
    filteredCurrent = currentFilter.calculate((rawCurrentLeft + rawCurrentRight) / 2);
  }

  public boolean isJammed() {
    return filteredCurrent > JAM_CURRENT_THRESHOLD.getAsDouble();
  }

  public void intakeRequest() {
    setStateFromRequest(HopperState.INTAKING);
  }

  public void outtakeRequest() {
    setStateFromRequest(HopperState.OUTTAKING);
  }

  public void idleRequest() {
    setStateFromRequest(HopperState.IDLE);
  }
}
