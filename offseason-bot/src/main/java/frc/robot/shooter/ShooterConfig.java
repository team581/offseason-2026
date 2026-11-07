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
import com.team581.math.MathHelpers;
import com.team581.math.PolynomialRegression;
import com.team581.util.tuning.TunableInterpolatingDoubleTreeMap;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.config.FeatureFlags;
import java.util.Map;

public class ShooterConfig {
  // Beta competition configuration (6645bd8d^); retained for hardware self-tests.
  public static final double SELF_TEST_LEFT_MOTOR_EXPECTED_RPM = 2500;
  public static final double SELF_TEST_LEFT_MOTOR_RPM_TOLERANCE = 250;
  public static final double SELF_TEST_LEFT_MOTOR_EXPECTED_CURRENT = 10.0;
  public static final double SELF_TEST_LEFT_MOTOR_CURRENT_TOLERANCE = 5;

  public static final double SELF_TEST_RIGHT_MOTOR_EXPECTED_RPM = 2500;
  public static final double SELF_TEST_RIGHT_MOTOR_RPM_TOLERANCE = 250;
  public static final double SELF_TEST_RIGHT_MOTOR_EXPECTED_CURRENT = 10.0;
  public static final double SELF_TEST_RIGHT_MOTOR_CURRENT_TOLERANCE = 5;

  public static final double ROLLER_DIAMETER_METERS = Units.inchesToMeters(4.0);
  public static final double MOTOR_TO_ROLLER_RATIO = 1.0;

  public static final double RPM_TOLERANCE = 150.0;
  public static final double RPM_TOLERANCE_ACTIVELY_SHOOTING = 300.0;

  public static final double RPM_TOLERANCE_FEEDING = 1000.0;

  public static final double GP_DETECT_CURRENT_THRESHOLD = 70.0;

  public static final double IDLE_RPM = 400;

  public static final double PIT_FUNCTIONALITY_RPM = 800;

  public static final double TEST_VOLTAGE = 6.0;

  public static final double MAX_SAFE_RPM = 6000;

  public static final Transform2d SHOOTER_TO_ROBOT = new Transform2d(0.0, 0.0, Rotation2d.k180deg);

  // Shot tables and real voltage gains from beta-bot competition configuration (6645bd8d^).
  public static final InterpolatingDoubleTreeMap DISTANCE_TO_SCORE_RPM =
      TunableInterpolatingDoubleTreeMap.ofEntries(
          "Shooter/DistanceToScoreRPM",
          Map.entry(5.9, 2350.0),
          Map.entry(3.94, 1950.0),
          Map.entry(2.82, 1800.0),
          Map.entry(1.76, 1550.0));
  public static final InterpolatingDoubleTreeMap DISTANCE_TO_FEEDING_RPM =
      TunableInterpolatingDoubleTreeMap.ofEntries(
          "Shooter/DistanceToFeedingRPM",
          Map.entry(6.0, 2000.0),
          Map.entry(8.71, 2300.0),
          Map.entry(13.6, 5000.0));
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

  // Added to the hub distance before the scoring RPM and hood lookups to shift every shot longer.
  public static DoubleSubscriber SCORING_DISTANCE_OFFSET_METERS =
      DogLog.tunable("Shooter/ScoringDistanceOffsetMeters", Units.feetToMeters(1.0));

  public static DoubleSubscriber TURBO_MODE_FF_CURRENT =
      DogLog.tunable("Shooter/TurboShotFFCurrent", 0.0);
  // Calculated from average of CAPIN logs
  public static DoubleSubscriber FEEDER_TO_SHOOTER_TRAVEL_TIME =
      DogLog.tunable("Shooter/FeederToShooterTravelTime", 0.25);

  public static Rotation2d calculateAimingAngle(
      Translation2d shooterTranslation, Translation2d goalTranslation) {
    return MathHelpers.getDriveDirection(shooterTranslation, goalTranslation)
        .plus(SHOOTER_TO_ROBOT.getRotation());
  }

  public static double getFeedTimeOfFlight(double distance) {
    return FeatureFlags.TOF_REGRESSION_MODEL.getAsBoolean()
        ? FEEDING_TOF_REGRESSION_MODEL.calculate(distance)
        : DISTANCE_TO_FEED_TOF.get(distance);
  }

  public static double getGoalCentricTolerance(
      Translation2d goalTranslation, Pose2d shooterPose, double goalCentricToleranceMeters) {

    double distanceToGoal = shooterPose.getTranslation().getDistance(goalTranslation);
    return Math.toDegrees(Math.atan2(goalCentricToleranceMeters, distanceToGoal));
  }

  public static double getScoreTimeOfFlight(double distance) {
    return FeatureFlags.TOF_REGRESSION_MODEL.getAsBoolean()
        ? SCORING_TOF_REGRESSION_MODEL.calculate(distance)
        : DISTANCE_TO_SCORE_TOF.get(distance);
  }

  public static Pose2d getShooterPose(Pose2d robot) {
    return robot.plus(SHOOTER_TO_ROBOT);
  }

  public static ChassisSpeeds getShooterSpeeds(ChassisSpeeds robotSpeeds, double robotHeading) {
    var angularVelocity = robotSpeeds.omegaRadiansPerSecond;
    Translation2d fieldRelativeOffset =
        SHOOTER_TO_ROBOT.getTranslation().rotateBy(Rotation2d.fromDegrees(robotHeading));
    var shooterSwingX = -angularVelocity * fieldRelativeOffset.getY();
    var shooterSwingY = angularVelocity * fieldRelativeOffset.getX();
    var shooterTotalVelocityX = robotSpeeds.vxMetersPerSecond + shooterSwingX;
    var shooterTotalVelocityY = robotSpeeds.vyMetersPerSecond + shooterSwingY;
    return new ChassisSpeeds(shooterTotalVelocityX, shooterTotalVelocityY, angularVelocity);
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
