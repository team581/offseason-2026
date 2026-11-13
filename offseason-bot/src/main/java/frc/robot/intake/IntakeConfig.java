package frc.robot.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class IntakeConfig {
  // Motor rotations per roller rotation, from the offseason subsystem hardware specs.
  public static final double MOTOR_TO_ROLLER_RATIO = 20.0 / 12.0;

  public static final double LEFT_ROLLER_DIAMETER_METERS = Units.inchesToMeters(2.0);
  public static final double RIGHT_ROLLER_DIAMETER_METERS = Units.inchesToMeters(2.0);

  public static final TalonFXConfiguration LEFT_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1.0 / 1.0))
          .withCurrentLimits(
              new CurrentLimitsConfigs().withStatorCurrentLimit(100).withSupplyCurrentLimit(50));
  public static final TalonFXConfiguration RIGHT_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1.0 / 1.0))
          .withCurrentLimits(
              new CurrentLimitsConfigs().withStatorCurrentLimit(100).withSupplyCurrentLimit(50));
}
