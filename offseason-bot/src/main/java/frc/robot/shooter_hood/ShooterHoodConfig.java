package frc.robot.shooter_hood;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team581.math.PolynomialRegression;
import com.team581.util.tuning.TunableInterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.Map;

public class ShooterHoodConfig {
  /**
   * This is the effective angle of the shooter hood relative to the floor when the shooter hood is
   * fully retracted.
   */
  public static final double ANGLE_FROM_HORIZONTAL = 11;

  // 33 is max without top rollers touching
  public static final double MAX_ANGLE = ANGLE_FROM_HORIZONTAL + 34.0;
  public static final double MIN_ANGLE = ANGLE_FROM_HORIZONTAL + 2.0;
  public static final double IDLE_ANGLE = ANGLE_FROM_HORIZONTAL + 2.0;

  // TODO: Update Homing numbers
  public static final double HOMING_VOLTAGE = -1;
  public static final double HOMING_CURRENT_THRESHOLD = 10;
  public static final double HOMING_END_POSITION = ANGLE_FROM_HORIZONTAL;

  public static final double TOLERANCE = 3;
  public static final double FEEDING_TOLERANCE = 5;

  // Voltage output, measured in mechanism rotations: V/rotation and V/(rotation/s).
  private static final Slot0Configs SIM_GAINS = new Slot0Configs().withKP(100.0).withKD(2.0);

  public static final TalonFXConfiguration MOTOR_CONFIG =
      new TalonFXConfiguration()
          .withFeedback(
              new FeedbackConfigs().withSensorToMechanismRatio(1 / ((8.0 / 62.0) * (10.0 / 154.0))))
          .withCurrentLimits(
              new CurrentLimitsConfigs().withStatorCurrentLimit(40).withSupplyCurrentLimit(40))
          .withVoltage(new VoltageConfigs().withPeakForwardVoltage(10).withPeakReverseVoltage(-10))
          .withMotorOutput(
              new MotorOutputConfigs()
                  .withNeutralMode(NeutralModeValue.Coast)
                  .withInverted(InvertedValue.Clockwise_Positive))
          .withSlot0(
              RobotBase.isSimulation()
                  ? SIM_GAINS.clone()
                  : new Slot0Configs().withKP(0).withKV(0).withKS(0))
          // SimKit uses these limits even for PositionVoltage; hardware keeps its defaults.
          .withMotionMagic(
              RobotBase.isSimulation()
                  ? new MotionMagicConfigs()
                      .withMotionMagicCruiseVelocity(60.0 / 360.0)
                      .withMotionMagicAcceleration(180.0 / 360.0)
                  : new MotionMagicConfigs());

  // TODO: Update interpolating map numbers later
  public static final InterpolatingDoubleTreeMap DISTANCE_TO_SCORE =
      TunableInterpolatingDoubleTreeMap.ofEntries(
          "ShooterHood/DistanceToScore",
          Map.entry(4.92, 33.0),
          Map.entry(3.46, 29.0),
          Map.entry(2.38, 20.0),
          Map.entry(1.42, 12.5));
  public static final InterpolatingDoubleTreeMap DISTANCE_TO_FEED =
      TunableInterpolatingDoubleTreeMap.ofEntries(
          "ShooterHood/DistanceToFeed",
          Map.entry(6.0, 35.0),
          Map.entry(8.71, 40.0),
          Map.entry(13.6, 40.0));
  public static final PolynomialRegression SCORING_REGRESSION_MODEL =
      PolynomialRegression.quadratic("ShooterHood/ScoringRegression", DISTANCE_TO_SCORE);
  public static final PolynomialRegression FEEDING_REGRESSION_MODEL =
      PolynomialRegression.quadratic("ShooterHood/FeedingRegression", DISTANCE_TO_FEED);

  private ShooterHoodConfig() {}
}
