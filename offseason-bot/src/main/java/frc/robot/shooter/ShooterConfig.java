package frc.robot.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.team581.math.PolynomialRegression;
import com.team581.util.tuning.TunableInterpolatingDoubleTreeMap;
import dev.doglog.DogLog;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.config.FeatureFlags;
import java.util.Map;

public class ShooterConfig {
  public static final double ROLLER_DIAMETER_METERS = Units.inchesToMeters(4.0);
  public static final double MOTOR_TO_ROLLER_RATIO = 1.0;

  public static final double RPM_TOLERANCE = 150.0;
  public static final double RPM_TOLERANCE_ACTIVELY_SHOOTING = 300.0;

  public static final double RPM_TOLERANCE_FEEDING = 1000.0;

  public static final double IDLE_RPM = 400;

  public static final double PIT_FUNCTIONALITY_RPM = 800;

  public static final double MAX_SAFE_RPM = 6000;

  public static final InterpolatingDoubleTreeMap DISTANCE_TO_SCORE_RPM =
      InterpolatingDoubleTreeMap.ofEntries(Map.entry(2.2, 1500.0), Map.entry(2.77, 1800.0));
  public static final InterpolatingDoubleTreeMap DISTANCE_TO_FEEDING_RPM =
      TunableInterpolatingDoubleTreeMap.ofEntries(
          "Shooter/DistanceToFeedingRPM",
          Map.entry(6.0, 1860.0),
          Map.entry(8.71, 2139.0),
          Map.entry(13.6, 4650.0));
  public static final PolynomialRegression SCORING_REGRESSION_MODEL =
      PolynomialRegression.quadratic("Shooter/ScoringRegression", DISTANCE_TO_SCORE_RPM);
  public static final PolynomialRegression FEEDING_REGRESSION_MODEL =
      PolynomialRegression.quadratic("Shooter/FeedingRegression", DISTANCE_TO_FEEDING_RPM);

  public static final InterpolatingDoubleTreeMap DISTANCE_TO_SCORE_TOF =
      TunableInterpolatingDoubleTreeMap.ofEntries(
          "Shooter/DistanceToScoreToF",
          // near
          Map.entry(1.36, 0.8916666667),
          // tower
          Map.entry(2.42, 1.063636364),
          // trench
          Map.entry(3.54, 1.161904762),
          // corner
          Map.entry(5.5, 1.248484848));

  public static final InterpolatingDoubleTreeMap DISTANCE_TO_FEED_TOF =
      TunableInterpolatingDoubleTreeMap.ofEntries(
          "Shooter/DistanceToFeedToF",
          Map.entry(6.0, 1.305555556),
          Map.entry(8.71, 1.275555556),
          Map.entry(13.6, 1.530952381));

  public static final PolynomialRegression SCORING_TOF_REGRESSION_MODEL =
      PolynomialRegression.quadratic("Shooter/ScoringToFRegression", DISTANCE_TO_SCORE_TOF);
  public static final PolynomialRegression FEEDING_TOF_REGRESSION_MODEL =
      PolynomialRegression.quadratic("Shooter/FeedingToFRegression", DISTANCE_TO_FEED_TOF);
  // Torque-current output: A per motor RPS. SimKit has no friction/load to feed forward.
  private static final Slot0Configs SIM_GAINS = new Slot0Configs().withKP(5.0);

  public static final TalonFXConfiguration LEFT_MOTOR_CONFIG =
      createMotorConfig()
          .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive))
          .withSlot0(
              RobotBase.isSimulation()
                  ? SIM_GAINS.clone()
                  : new Slot0Configs().withKP(0.4).withKV(0.12));
  public static final TalonFXConfiguration RIGHT_MOTOR_CONFIG =
      createMotorConfig()
          .withMotorOutput(
              new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive))
          .withSlot0(
              RobotBase.isSimulation()
                  ? SIM_GAINS.clone()
                  : new Slot0Configs().withKP(0.55).withKV(0.127));
  public static DoubleSubscriber FULL_HOPPER_INITIAL_FF =
      DogLog.tunable("Shooter/FullHopperInitialFF", 0.0);
  public static DoubleSubscriber LOW_HOPPER_INITIAL_FFF =
      DogLog.tunable("Shooter/LowHopperInitialFF", 0.0);

  public static double getFeedTimeOfFlight(double distance) {
    return FeatureFlags.TOF_REGRESSION_MODEL.getAsBoolean()
        ? FEEDING_TOF_REGRESSION_MODEL.calculate(distance)
        : DISTANCE_TO_FEED_TOF.get(distance);
  }

  public static double getScoreTimeOfFlight(double distance) {
    return FeatureFlags.TOF_REGRESSION_MODEL.getAsBoolean()
        ? SCORING_TOF_REGRESSION_MODEL.calculate(distance)
        : DISTANCE_TO_SCORE_TOF.get(distance);
  }

  private static TalonFXConfiguration createMotorConfig() {
    return new TalonFXConfiguration()
        .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(1.0 / 1.0))
        .withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(MAX_SAFE_RPM / 60.0)
                .withMotionMagicAcceleration(4000.0 / 60.0))
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimitEnable(true)
                .withStatorCurrentLimit(100)
                .withSupplyCurrentLimit(100))
        .withVoltage(
            RobotBase.isSimulation()
                ? new VoltageConfigs()
                : new VoltageConfigs().withPeakReverseVoltage(0))
        .withTorqueCurrent(
            new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(200)
                .withPeakReverseTorqueCurrent(RobotBase.isSimulation() ? -200 : 0));
  }

  private ShooterConfig() {}
}
