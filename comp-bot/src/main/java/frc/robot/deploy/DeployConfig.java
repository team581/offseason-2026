package frc.robot.deploy;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class DeployConfig {
  // TODO: These are just placeholders - we should update these when we find out these angles
  public static final double MAX_ANGLE = 90;
  public static final double MIN_ANGLE = 0;

  public static final double HOMING_END_ANGLE = 0;
  public static final double HOMING_VOLTAGE = 0;

  public static final TalonFXConfiguration MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1))
          .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
          .withCurrentLimits(
              new CurrentLimitsConfigs().withStatorCurrentLimit(1).withStatorCurrentLimit(1))
          .withSlot0(new Slot0Configs().withKP(0).withKV(0).withKG(0));
}
