package frc.robot.intake.Conveyor;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ConveyorConfig {
  public static final TalonFXConfiguration TOP_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(0.0 / 0.0)) // TBD
          .withCurrentLimits(
              new CurrentLimitsConfigs().withStatorCurrentLimit(50).withSupplyCurrentLimit(10));
  public static final TalonFXConfiguration BOTTOM_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(0.0 / 0.0)) // TBD
          .withCurrentLimits(
              new CurrentLimitsConfigs().withStatorCurrentLimit(50).withSupplyCurrentLimit(10));
}
