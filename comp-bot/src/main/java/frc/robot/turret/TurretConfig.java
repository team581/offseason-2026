package frc.robot.turret;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;

public class TurretConfig {
  public static final double MIN_ANGLE = -270;
  public static final double MAX_ANGLE = 270;
  public static final double OUT_OF_BOUNDS_THRESHOLD = 1.0;
  public static final double HOMING_END_POSITION = MIN_ANGLE;
  public static final double TOLERANCE = 1.0;

  // Turret 2d transform relative to robot center
  public static final Transform2d TURRET_TO_ROBOT =
      new Transform2d(Units.inchesToMeters(0.0), Units.inchesToMeters(0.0), Rotation2d.kZero);

  public static final TalonFXConfiguration MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withFeedback(
              new FeedbackConfigs().withSensorToMechanismRatio((280.0 / 12.0) * (40.0 / 12.0)))
          .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
          .withCurrentLimits(
              new CurrentLimitsConfigs().withStatorCurrentLimit(30).withStatorCurrentLimit(30))
          .withSlot0(new Slot0Configs().withKP(150.0).withKV(0.0).withKG(0.0));

  private TurretConfig() {}
}
