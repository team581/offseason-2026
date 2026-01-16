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
  private final TalonFX motor;

  private final LinearFilter currentFilter = LinearFilter.movingAverage(5);
  private double rawCurrent = 0.0;

  private double filteredCurrent = 0.0;

  private static final DoubleSubscriber JAM_CURRENT_THRESHOLD =
      DogLog.tunable("HOPPER/JamCurrentThreshold", 75.0);

  public Hopper(TalonFX motor) {
    super(SubsystemPriority.HOPPER, HopperState.IDLE);
    var config =  new TalonFXConfiguration()
                // TODO:Get sensor to mechanism ratio
                .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(0))
                .withCurrentLimits(
                    new CurrentLimitsConfigs()
                        .withStatorCurrentLimit(100)
                        .withSupplyCurrentLimit(100))
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));

    motor
        .getConfigurator()
        .apply(
            config);


    this.motor = motor;
  }

  @Override
  protected void afterTransition(HopperState newState) {
    switch (newState) {
      case UNTUNED, STOPPED -> {
        motor.disable();
      }
      default -> {
        motor.setVoltage(getState().volts);
      }
    }
  }

  @Override
  protected void collectInputs() {
    rawCurrent = motor.getStatorCurrent().getValueAsDouble();
    filteredCurrent = currentFilter.calculate(rawCurrent);
  }

  public boolean isJammed() {
    return filteredCurrent > JAM_CURRENT_THRESHOLD.getAsDouble();
  }

  public void setState(HopperState newState) {
    setStateFromRequest(newState);
  }
}
