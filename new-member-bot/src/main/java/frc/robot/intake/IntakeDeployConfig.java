package frc.robot.deploy;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.config.RobotKind;

public class DeployConfig {
  public static final double MAX_LENGTH = 11.8;
  public static final double MIN_LENGTH = 5;
  public static final double HOMING_END_POSITION_INWARD = 0;
  public static final double HOMING_END_POSITION_OUTWARD = 11.9;
  public static final double HOMING_VOLTAGE_INWARD = -2;
  public static final double HOMING_VOLTAGE_OUTWARD = 4;
  public static final double HOMING_CURRENT = 20.0;
  public static final double HOMING_CURRENT_OUTWARD = 35.0;
  public static final double POSITION_TOLERANCE = 0.25;

  private static final double RATIO = 1.59155;

  public static final TalonFXConfiguration LEFT_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(RATIO))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(
                      RobotKind.IS_COMP_BOT ? NeutralModeValue.Brake : NeutralModeValue.Coast)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withCurrentLimits(
              new CurrentLimitsConfigs().withStatorCurrentLimit(50).withSupplyCurrentLimit(18))
          .withTorqueCurrent(
              new TorqueCurrentConfigs()
                  .withPeakForwardTorqueCurrent(100)
                  .withPeakReverseTorqueCurrent(-35))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(200.0)
                  .withMotionMagicAcceleration(50.0))
          .withSlot0(new Slot0Configs().withKP(8.0))
          .withSlot1(new Slot1Configs().withKP(8.0))
          .withSlot2(new Slot2Configs().withKP(200.0));

  public static final TalonFXConfiguration RIGHT_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(RATIO))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(
                      RobotKind.IS_COMP_BOT ? NeutralModeValue.Brake : NeutralModeValue.Coast)
                  .withInverted(InvertedValue.CounterClockwise_Positive))
          .withCurrentLimits(
              new CurrentLimitsConfigs().withStatorCurrentLimit(50).withSupplyCurrentLimit(18))
          .withTorqueCurrent(
              new TorqueCurrentConfigs()
                  .withPeakForwardTorqueCurrent(100)
                  .withPeakReverseTorqueCurrent(-35))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(200.0)
                  .withMotionMagicAcceleration(50.0))
          .withSlot0(new Slot0Configs().withKP(8.0))
          .withSlot1(new Slot1Configs().withKP(8.0))
          .withSlot2(new Slot2Configs().withKP(200.0));

  private DeployConfig() {}
}
// VALUES ARE STOLEN FROM COMPBOT
