package frc.robot.util;

import com.team581.math.ShootOnTheMove;
import com.team581.util.FeedLocation;
import com.team581.util.FieldUtil;
import com.team581.util.FmsUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.shooter.ShooterConfig;
import frc.robot.turret.TurretCalculator;

public class AimParameterUtil {

  public record AimingParameters(
      double turretAngle,
      double distance,
      double turretTolerance,
      double turretFeedForwardRadians,
      double upcomingTurretAngle) {}

  private static final ShootOnTheMove SCORING_SOTM =
      new ShootOnTheMove(ShooterConfig.DISTANCE_TO_SCORE_TOF);

  private static final ShootOnTheMove FEEDING_SOTM =
      new ShootOnTheMove(ShooterConfig.DISTANCE_TO_FEED_TOF);

  private static final double FEEDING_GOAL_CENTRIC_TOLERANCE = Units.inchesToMeters(20);

  private static final double SCORING_GOAL_CENTRIC_TOLERANCE = Units.inchesToMeters(5);
  private static final double FEEDING_FALLBACK_DISTANCE_TO_GOAL = 8.0;

  private static final double FEEDING_FALLBACK_TOLERANCE = 5.0;

  public static AimingParameters getFallbackFeedingParameters(Rotation2d robotRotation) {
    var fieldGoal = FmsUtil.isRedAlliance() ? Rotation2d.kZero : Rotation2d.k180deg;
    var turretAngle = fieldGoal.minus(robotRotation).getDegrees();

    return new AimingParameters(
        turretAngle,
        FEEDING_FALLBACK_DISTANCE_TO_GOAL,
        FEEDING_FALLBACK_TOLERANCE,
        0.0,
        turretAngle);
  }

  public static AimingParameters getFeedingParameters(
      FeedLocation feedLocation, Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
    return getAimingParameters(
        FEEDING_SOTM,
        feedLocation.getTranslation(),
        FEEDING_GOAL_CENTRIC_TOLERANCE,
        robotPose,
        fieldRelativeSpeeds);
  }

  public static AimingParameters getScoringParameters(
      Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
    return getAimingParameters(
        SCORING_SOTM,
        FieldUtil.HUB_POSE.getTranslation(),
        SCORING_GOAL_CENTRIC_TOLERANCE,
        robotPose,
        fieldRelativeSpeeds);
  }

  public static AimingParameters getStaticFeedingParameters(
      FeedLocation feedLocation, Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
    return getStaticAimingParameters(
        FEEDING_SOTM,
        feedLocation.getTranslation(),
        FEEDING_GOAL_CENTRIC_TOLERANCE,
        robotPose,
        fieldRelativeSpeeds);
  }

  public static AimingParameters getStaticScoringParameters(
      Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
    return getStaticAimingParameters(
        SCORING_SOTM,
        FieldUtil.HUB_POSE.getTranslation(),
        SCORING_GOAL_CENTRIC_TOLERANCE,
        robotPose,
        fieldRelativeSpeeds);
  }

  private static AimingParameters getAimingParameters(
      ShootOnTheMove sotm,
      Translation2d goalTranslation,
      double tolerance,
      Pose2d robotPose,
      ChassisSpeeds fieldRelativeSpeeds) {
    var turretPose = TurretCalculator.getTurretPose(robotPose);
    var turretTranslation = turretPose.getTranslation();
    var turretFieldRelativeSpeeds =
        TurretCalculator.getTurretChassisSpeeds(
            fieldRelativeSpeeds, robotPose.getRotation().getDegrees());

    var separatedVelocityCompensatedGoal =
        sotm.getSeparatedVelocityCompensatedGoalWithEffectiveTof(
            turretTranslation, goalTranslation, turretFieldRelativeSpeeds);

    var compensatedGoal = separatedVelocityCompensatedGoal.fullyCompensatedGoal();
    double distance = turretTranslation.getDistance(compensatedGoal);
    double turretAngle = TurretCalculator.calculateTurretAimingAngle(robotPose, compensatedGoal);
    double turretTolerance =
        TurretCalculator.getGoalCentricTurretTolerance(compensatedGoal, robotPose, tolerance);

    double tangentialVelocity = separatedVelocityCompensatedGoal.tangentialVelocity();
    double realDistanceToGoal = turretTranslation.getDistance(goalTranslation);
    double translationalFF =
        realDistanceToGoal > 1e-6 ? tangentialVelocity / realDistanceToGoal : 0.0;
    double turretFeedForwardRadians =
        -translationalFF - turretFieldRelativeSpeeds.omegaRadiansPerSecond;
    Pose2d futureRobotPose =
        new Pose2d(
            robotPose
                .getTranslation()
                .plus(
                    new Translation2d(
                        fieldRelativeSpeeds.vxMetersPerSecond * 0.5,
                        fieldRelativeSpeeds.vyMetersPerSecond * 0.5)),
            robotPose
                .getRotation()
                .plus(Rotation2d.fromRadians(fieldRelativeSpeeds.omegaRadiansPerSecond * 0.5)));
    double upcomingTurretAngle =
        TurretCalculator.calculateTurretAimingAngle(futureRobotPose, compensatedGoal);

    return new AimingParameters(
        turretAngle, distance, turretTolerance, turretFeedForwardRadians, upcomingTurretAngle);
  }

  private static AimingParameters getStaticAimingParameters(
      ShootOnTheMove sotm,
      Translation2d goalTranslation,
      double tolerance,
      Pose2d robotPose,
      ChassisSpeeds fieldRelativeSpeeds) {
    var turretPose = TurretCalculator.getTurretPose(robotPose);
    var turretTranslation = turretPose.getTranslation();
    var turretFieldRelativeSpeeds =
        TurretCalculator.getTurretChassisSpeeds(
            fieldRelativeSpeeds, robotPose.getRotation().getDegrees());

    var separatedVelocityCompensatedGoal =
        sotm.getSeparatedVelocityCompensatedGoalWithEffectiveTof(
            turretTranslation, goalTranslation, turretFieldRelativeSpeeds);

    double distance = turretTranslation.getDistance(goalTranslation);
    double turretAngle = TurretCalculator.calculateTurretAimingAngle(robotPose, goalTranslation);
    double turretTolerance =
        TurretCalculator.getGoalCentricTurretTolerance(goalTranslation, robotPose, tolerance);

    double tangentialVelocity = separatedVelocityCompensatedGoal.tangentialVelocity();
    double turretFeedForwardRadians =
        -(distance > 1e-6 ? tangentialVelocity / distance : 0.0)
            - turretFieldRelativeSpeeds.omegaRadiansPerSecond;

    return new AimingParameters(
        turretAngle, distance, turretTolerance, turretFeedForwardRadians, turretAngle);
  }
}
