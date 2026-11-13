package frc.robot.feeder;

import static frc.robot.Hardware.SUPPLY_CURRENT_LIMIT_SCALE;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class FeederConfig {
  public static final TalonFXConfiguration TOP_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Brake))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1.0 / 1.0))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withSupplyCurrentLowerLimit(40 * SUPPLY_CURRENT_LIMIT_SCALE)
                  .withStatorCurrentLimit(50)
                  .withSupplyCurrentLimit(10 * SUPPLY_CURRENT_LIMIT_SCALE));
  public static final TalonFXConfiguration BOTTOM_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Brake))
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1.0 / 1.0))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withSupplyCurrentLowerLimit(40 * SUPPLY_CURRENT_LIMIT_SCALE)
                  .withStatorCurrentLimit(50)
                  .withSupplyCurrentLimit(10 * SUPPLY_CURRENT_LIMIT_SCALE));
}
