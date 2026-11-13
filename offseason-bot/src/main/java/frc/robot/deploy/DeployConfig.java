package frc.robot.deploy;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.config.RobotKind;

public class DeployConfig {
  // Arm angle: fully down is zero, positive angles raise/collapse the intake.
  public static final double FULL_SWEEP_DEGREES = 75.585;
  public static final double MOTOR_TO_ARM_RATIO = 36.57;
  // Starting estimate for bringup; tune the collapsed endpoint on the robot.
  public static final double COLLAPSED_ANGLE_DEGREES = 75.585;
  public static final double HOMING_END_POSITION_INWARD = COLLAPSED_ANGLE_DEGREES;
  public static final double HOMING_END_POSITION_OUTWARD = 0.0;
  public static final double MIN_ANGLE = -5.0;
  public static final double MAX_ANGLE = COLLAPSED_ANGLE_DEGREES - 20.0;
  // Motor feedback remains positive toward deployment. Account for the down pose's physical
  // angle from horizontal separately from the zero-based public arm angle.
  private static final double DOWN_GRAVITY_OFFSET_ROTATIONS = -0.028;

  public static final double INTAKE_TOLERANCE_DEGREES = 5.0;

  public static final double HOMING_VOLTAGE_INWARD = -2;

  public static final double HOMING_VOLTAGE_OUTWARD = 0.5;
  public static final double AUTO_INTAKE_DELAY_SECONDS = 0.5;
  public static final double HOMING_CURRENT = 20.0;
  public static final double HOMING_CURRENT_OUTWARD = 8.0;
  public static final double POSITION_TOLERANCE = 2.0;

  // Torque-current output: A/arm rotation and A/(arm rotation/s).
  // No simulated gravity/friction load, so kG/kS/kV remain zero.
  private static final Slot0Configs SIM_GAINS = new Slot0Configs().withKP(400.0).withKD(20.0);

  private static final Slot0Configs GAINS =
      new Slot0Configs()
          .withKP(500.0)
          .withKI(0)
          .withKD(20.0)
          // Keep gravity compensation at zero until measured on the physical arm.
          .withGravityType(GravityTypeValue.Arm_Cosine)
          .withGravityArmPositionOffset(DOWN_GRAVITY_OFFSET_ROTATIONS)
          .withKG(-18.0)
          .withKS(0.0)
          .withKV(0.0)
          .withKA(0);

  public static final TalonFXConfiguration LEFT_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(MOTOR_TO_ARM_RATIO))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(
                      RobotKind.IS_COMP_BOT ? NeutralModeValue.Brake : NeutralModeValue.Coast)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withCurrentLimits(
              new CurrentLimitsConfigs().withStatorCurrentLimit(50).withSupplyCurrentLimit(20))
          .withTorqueCurrent(
              new TorqueCurrentConfigs()
                  .withPeakForwardTorqueCurrent(100)
                  .withPeakReverseTorqueCurrent(-100))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(15.0)
                  .withMotionMagicAcceleration(100000.0))
          .withSlot0(RobotBase.isSimulation() ? SIM_GAINS.clone() : GAINS);

  public static final TalonFXConfiguration RIGHT_MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(MOTOR_TO_ARM_RATIO))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(
                      RobotKind.IS_COMP_BOT ? NeutralModeValue.Brake : NeutralModeValue.Coast)
                  .withInverted(InvertedValue.CounterClockwise_Positive))
          .withCurrentLimits(
              new CurrentLimitsConfigs().withStatorCurrentLimit(50).withSupplyCurrentLimit(20))
          .withTorqueCurrent(
              new TorqueCurrentConfigs()
                  .withPeakForwardTorqueCurrent(100)
                  .withPeakReverseTorqueCurrent(-100))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicCruiseVelocity(15.0)
                  .withMotionMagicAcceleration(100000.0))
          .withSlot0(RobotBase.isSimulation() ? SIM_GAINS.clone() : GAINS);

  private DeployConfig() {}
}
