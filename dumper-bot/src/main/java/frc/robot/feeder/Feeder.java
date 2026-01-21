package frc.robot.feeder;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.DoubleSubscriber;
import frc.robot.util.scheduling.SubsystemPriority;

public class Feeder extends StateMachineSubsystem<FeederState> {
  private final TalonFX feederMotor;
  private final LinearFilter currentFilterHopper = LinearFilter.movingAverage(5);
  private double rawCurrent = 0.0;

  private double filteredCurrent = 0.0;
  private static final DoubleSubscriber JAM_CURRENT_THRESHOLD =
      DogLog.tunable("Feeder/JamCurrentThreshold", 75.0);

  public Feeder(TalonFX feederMotor) {
    super(SubsystemPriority.FEEDER, FeederState.IDLE);
    feederMotor
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

    this.feederMotor = feederMotor;
  }

  @Override
  protected void afterTransition(FeederState newState) {
    switch (newState) {
      case UNTUNED -> {
        feederMotor.disable();
      }
      default -> {
        feederMotor.setVoltage(getState().volts);
        feederMotor.setVoltage(getState().volts);
      }
    }
  }

  @Override
  protected void collectInputs() {
    rawCurrent = feederMotor.getStatorCurrent().getValueAsDouble();
    filteredCurrent = currentFilterHopper.calculate(rawCurrent);
  }

  public boolean isJammed() {
    return filteredCurrent > JAM_CURRENT_THRESHOLD.getAsDouble();
  }

  public void feedRequest() {
    setStateFromRequest(FeederState.FEED);
  }

  public void idleRequest() {
    setStateFromRequest(FeederState.IDLE);
  }
}
