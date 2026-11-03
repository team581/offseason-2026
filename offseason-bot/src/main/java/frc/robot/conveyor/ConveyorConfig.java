package frc.robot.conveyor;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ConveyorConfig {
  // TODO: map motors 18/19 to the floor/tunnel before entering motor rotations per roller rotation.
  public static final double TOP_MOTOR_TO_ROLLER_RATIO = 1.0;
  public static final double BOTTOM_MOTOR_TO_ROLLER_RATIO = 1.0;

  // TODO: confirm which mechanism this motor drives and its roller/pulley diameter.
  public static final double TOP_ROLLER_DIAMETER_METERS = 1.0;
  // TODO: confirm which mechanism this motor drives and its roller/pulley diameter.
  public static final double BOTTOM_ROLLER_DIAMETER_METERS = 1.0;

  public static final TalonFXConfiguration TOP_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1.0 / 1.0))
          .withCurrentLimits(
              new CurrentLimitsConfigs().withStatorCurrentLimit(50).withSupplyCurrentLimit(10));
  public static final TalonFXConfiguration BOTTOM_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1.0 / 1.0))
          .withCurrentLimits(
              new CurrentLimitsConfigs().withStatorCurrentLimit(50).withSupplyCurrentLimit(10));
}
