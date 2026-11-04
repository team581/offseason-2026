package frc.robot.intake;

import static frc.robot.Hardware.SUPPLY_CURRENT_LIMIT_SCALE;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class IntakeConfig {
  public static final TalonFXConfiguration LEFT_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.CounterClockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          // TODO: UPDATE RATIO
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1.0 / 1.0))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withSupplyCurrentLowerLimit(40 * SUPPLY_CURRENT_LIMIT_SCALE)
                  .withStatorCurrentLimit(75)
                  .withSupplyCurrentLimit(50 * SUPPLY_CURRENT_LIMIT_SCALE));
  public static final TalonFXConfiguration RIGHT_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withInverted(InvertedValue.Clockwise_Positive)
                  .withNeutralMode(NeutralModeValue.Coast))
          // TODO: UPDATE RATIO
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1.0 / 1.0))
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withSupplyCurrentLowerLimit(40 * SUPPLY_CURRENT_LIMIT_SCALE)
                  .withStatorCurrentLimit(75)
                  .withSupplyCurrentLimit(50 * SUPPLY_CURRENT_LIMIT_SCALE));
}
