package frc.robot.feeder;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;

public class FeederConfig {
  // Tune above unloaded current and below current while fuel passes through the feeder.
  public static final DoubleSubscriber FUEL_CURRENT_THRESHOLD_AMPS =
      DogLog.tunable("Feeder/FuelCurrentThresholdAmps", 10.0);
  public static final DoubleSubscriber EMPTY_DEBOUNCE_SECONDS =
      DogLog.tunable("Feeder/EmptyDebounceSeconds", 0.5);

  // Motor rotations per roller rotation, from the offseason subsystem hardware specs.
  public static final double MOTOR_TO_ROLLER_RATIO = 28.0 / 18.0;

  public static final double ROLLER_DIAMETER_METERS = Units.inchesToMeters(2.0);

  public static final TalonFXConfiguration MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Brake))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1.0 / 1.0))
          .withCurrentLimits(
              new CurrentLimitsConfigs().withStatorCurrentLimit(50).withSupplyCurrentLimit(10));
}
