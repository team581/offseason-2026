package frc.robot.turret;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

public class TurretConfig {
  public static final double MIN_ANGLE = -270.0;
  public static final double MAX_ANGLE = -90.0;
  public static final double FAUX_DUMPER_ANGLE = -180;

  public static final double OUT_OF_BOUNDS_THRESHOLD = 1.0;
  // Hand-set the turret at the center of travel before the first enable after boot.
  public static final double HOMING_END_POSITION = -180.0;
  public static final double HOMING_CORRECTION_THRESHOLD_DEGREES = 0.01;
  public static final double HOMING_DEBOUNCE_SECONDS = 0.1;

  public static final double MOTOR_TO_TURRET = 60.0;

  // Calibrate the fractional rotor position with the turret exactly at HOMING_END_POSITION.
  public static final double MOTOR_ROTOR_CAL_OFFSET = 0.0;

  public static final double MAX_VELOCITY_RAD_PER_SEC = 10.0;
  public static final double MAX_ACCELERATION_RAD_PER_SEC_SQUARED = 35.0;

  // Turret axis relative to robot center: 4.750 inches back and 8.025 inches left.
  public static final Transform2d TURRET_TO_ROBOT =
      new Transform2d(Units.inchesToMeters(-4.750), Units.inchesToMeters(8.025), Rotation2d.kZero);

  // Voltage output per turret rotation; SimKit has no gravity or friction model.
  private static final Slot0Configs SIM_GAINS = new Slot0Configs().withKP(60.0).withKD(2.0);

  // Real gains from beta-bot competition configuration (6645bd8d^).
  public static final TalonFXConfiguration MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(MOTOR_TO_TURRET))
          .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
          .withCurrentLimits(
              new CurrentLimitsConfigs().withStatorCurrentLimit(40).withSupplyCurrentLimit(20))
          .withVoltage(new VoltageConfigs().withPeakForwardVoltage(10).withPeakReverseVoltage(-10))
          .withSlot0(
              RobotBase.isSimulation()
                  ? SIM_GAINS.clone()
                  : new Slot0Configs().withKP(200).withKV(5.0).withKG(0.0).withKD(1.7).withKS(0.35))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(Units.radiansToRotations(MAX_VELOCITY_RAD_PER_SEC))
                  .withMotionMagicAcceleration(
                      Units.radiansToRotations(MAX_ACCELERATION_RAD_PER_SEC_SQUARED)));

  private TurretConfig() {}
}
