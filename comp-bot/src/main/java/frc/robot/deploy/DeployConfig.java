package frc.robot.deploy;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class DeployConfig {

  public static final double MAX_LENGTH = 11.8;
  public static final double MIN_LENGTH = 0;
  public static final double HOMING_END_POSITION_INWARD = 0;
  public static final double HOMING_END_POSITION_OUTWARD = 11.9;
  public static final double HOMING_VOLTAGE_INWARD = -2;
  public static final double HOMING_VOLTAGE_OUTWARD = 2;
  public static final double HOMING_CURRENT = 15.0;
  public static final double POSITION_TOLERANCE = 0.25;

  public static final TalonFXConfiguration MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withTorqueCurrent(
              new TorqueCurrentConfigs()
                  .withPeakForwardTorqueCurrent(70)
                  .withPeakReverseTorqueCurrent(-70))
          .withFeedback(
              new FeedbackConfigs()
                  .withSensorToMechanismRatio((40.0 / 8.0) * (1 / (Math.PI * (2 * 0.5)))))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Coast)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withCurrentLimits(
              new CurrentLimitsConfigs().withStatorCurrentLimit(80).withSupplyCurrentLimit(15))
          // TODO: TUNE MOTION MAGIC
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(200.0)
                  .withMotionMagicAcceleration(300.0))
          .withSlot0(
              new Slot0Configs()
                  .withKP(5.0)
                  .withKI(0)
                  .withKD(0.0)
                  .withKG(0.0)
                  .withKS(0.0)
                  .withKV(0.0)
                  .withKA(0.0));
}
