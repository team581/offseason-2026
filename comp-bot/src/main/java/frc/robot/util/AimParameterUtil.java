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

public class AimParameterUtil {

  public record AimingParameters(
      double goalAngle, double distance, double swerveTolerance, double swerveFeedForwardRadians) {}

  private static final ShootOnTheMove SCORING_SOTM =
      new ShootOnTheMove(ShooterConfig.DISTANCE_TO_SCORE_TOF);

  private static final ShootOnTheMove FEEDING_SOTM =
      new ShootOnTheMove(ShooterConfig.DISTANCE_TO_FEED_TOF);

  private static final double FEEDING_GOAL_CENTRIC_TOLERANCE = Units.inchesToMeters(20);

  private static final double SCORING_GOAL_CENTRIC_TOLERANCE = Units.inchesToMeters(5);
  private static final double FEEDING_FALLBACK_DISTANCE_TO_GOAL = 8.0;

  private static final double FEEDING_FALLBACK_TOLERANCE = 5.0;

  public static AimingParameters getFallbackFeedingParameters(Rotation2d robotRotation) {
    var swerveAngle =
        FmsUtil.isRedAlliance() ? Rotation2d.k180deg.getDegrees() : Rotation2d.kZero.getDegrees();

    return new AimingParameters(
        swerveAngle, FEEDING_FALLBACK_DISTANCE_TO_GOAL, FEEDING_FALLBACK_TOLERANCE, 0.0);
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
    var shooterPose = ShooterConfig.getShooterPose(robotPose);
    var shooterTranslation = shooterPose.getTranslation();
    var shooterFieldRelativeSpeeds =
        ShooterConfig.getShooterSpeeds(fieldRelativeSpeeds, robotPose.getRotation().getDegrees());

    var separatedVelocityCompensatedGoal =
        sotm.getSeparatedVelocityCompensatedGoalWithEffectiveTof(
            shooterTranslation, goalTranslation, shooterFieldRelativeSpeeds);

    var compensatedGoal = separatedVelocityCompensatedGoal.fullyCompensatedGoal();
    double distance = shooterTranslation.getDistance(compensatedGoal);
    double swerveAngle =
        ShooterConfig.calculateAimingAngle(shooterTranslation, compensatedGoal).getDegrees();
    double swerveTolerance =
        ShooterConfig.getGoalCentricTolerance(compensatedGoal, shooterPose, tolerance);

    double tangentialVelocity = separatedVelocityCompensatedGoal.tangentialVelocity();
    double realDistanceToGoal = shooterTranslation.getDistance(goalTranslation);
    double swerveFeedForwardRadians = -(tangentialVelocity / realDistanceToGoal);

    return new AimingParameters(swerveAngle, distance, swerveTolerance, swerveFeedForwardRadians);
  }

  private static AimingParameters getStaticAimingParameters(
      ShootOnTheMove sotm,
      Translation2d goalTranslation,
      double tolerance,
      Pose2d robotPose,
      ChassisSpeeds fieldRelativeSpeeds) {
    var shooterPose = ShooterConfig.getShooterPose(robotPose);
    var shooterTranslation = shooterPose.getTranslation();
    var shooterFieldRelativeSpeeds =
        ShooterConfig.getShooterSpeeds(fieldRelativeSpeeds, robotPose.getRotation().getDegrees());

    var separatedVelocityCompensatedGoal =
        sotm.getSeparatedVelocityCompensatedGoalWithEffectiveTof(
            shooterTranslation, goalTranslation, shooterFieldRelativeSpeeds);

    double distance = shooterTranslation.getDistance(goalTranslation);
    double swerveAngle =
        ShooterConfig.calculateAimingAngle(shooterTranslation, goalTranslation).getDegrees();
    double swerveTolerance =
        ShooterConfig.getGoalCentricTolerance(goalTranslation, shooterPose, tolerance);

    double tangentialVelocity = separatedVelocityCompensatedGoal.tangentialVelocity();
    double swerveFeedForwardRadians = -(tangentialVelocity / distance);

    return new AimingParameters(swerveAngle, distance, swerveTolerance, swerveFeedForwardRadians);
  }
}
