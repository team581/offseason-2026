package frc.robot.conveyor;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;

public class ConveyorConfig {
  public static final double TUNNEL_MOTOR_TO_PULLEY_RATIO = 30.0 / 12.0;

  // Measure belt travel per drive-pulley revolution. Zero means not yet calibrated.
  public static final DoubleSubscriber TUNNEL_METERS_PER_PULLEY_REVOLUTION =
      DogLog.tunable("Tunnel/MetersPerPulleyRevolution", 0.0);

  public static final double MOTOR_TO_ROLLER_RATIO =
      1.0 / ((14.0 / 60.0) * (16.0 / 18.0) * (16.0 / 20.0) * (17.0 / 20.0) * (17.0 / 20.0));

  public static final double ROLLER_DIAMETER_METERS = Units.inchesToMeters(1.5);

  public static final TalonFXConfiguration MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1.0 / 1.0))
          .withCurrentLimits(
              new CurrentLimitsConfigs().withStatorCurrentLimit(50).withSupplyCurrentLimit(10));
  public static final TalonFXConfiguration TUNNEL_TOP_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Brake))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1.0 / 1.0))
          .withCurrentLimits(
              new CurrentLimitsConfigs().withStatorCurrentLimit(50).withSupplyCurrentLimit(10));
  public static final TalonFXConfiguration TUNNEL_BOTTOM_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Brake))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1.0 / 1.0))
          .withCurrentLimits(
              new CurrentLimitsConfigs().withStatorCurrentLimit(50).withSupplyCurrentLimit(10));
}
