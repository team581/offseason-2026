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
        FmsUtil.isRedAlliance() ? Rotation2d.kZero.getDegrees() : Rotation2d.k180deg.getDegrees();

    return new AimingParameters(
        swerveAngle, FEEDING_FALLBACK_DISTANCE_TO_GOAL, FEEDING_FALLBACK_TOLERANCE, 0.0);
  }

  public static AimingParameters getFeedingParameters(
      FeedLocation feedLocation, Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
    // Calculate translation of turret on the field
    var shooterPose = ShooterConfig.getShooterPose(robotPose);

    var shooterTranslation = shooterPose.getTranslation();

    // Calculate speeds of turret (x, y, omega)
    var shooterFieldRelativeSpeeds =
        ShooterConfig.getShooterSpeeds(fieldRelativeSpeeds, robotPose.getRotation().getDegrees());

    Translation2d feedTranslation = feedLocation.getTranslation(shooterPose);

    // Get velocity compensated goals
    var separatedVelocityCompensatedGoal =
        FEEDING_SOTM.getSeparatedVelocityCompensatedGoalWithEffectiveTof(
            shooterTranslation, feedTranslation, shooterFieldRelativeSpeeds);

    // Calculate fully compensated distance to goal for shooter, hood, and turret
    var compensatedGoal = separatedVelocityCompensatedGoal.fullyCompensatedGoal();
    var fullyCompensatedDistanceToGoal = shooterTranslation.getDistance(compensatedGoal);

    var swerveAngle =
        ShooterConfig.calculateAimingAngle(shooterTranslation, compensatedGoal).getDegrees();
    // Use same goal for goal centric turret tolerance
    var swerveTolerance =
        ShooterConfig.getGoalCentricTolerance(
            compensatedGoal, shooterPose, FEEDING_GOAL_CENTRIC_TOLERANCE);

    // Calculate translational FF for turret to account for linear turret velocity
    var realDistanceToGoal = shooterTranslation.getDistance(feedTranslation);
    double tangentialVelocity = separatedVelocityCompensatedGoal.tangentialVelocity();
    double translationalFF = tangentialVelocity / realDistanceToGoal;

    // Sum translational FF and rotational FF
    double swerveFeedForwardRadians = -translationalFF;

    return new AimingParameters(
        swerveAngle, fullyCompensatedDistanceToGoal, swerveTolerance, swerveFeedForwardRadians);
  }

  public static AimingParameters getScoringParameters(
      Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
    // Calculate translation of turret on the field
    var shooterPose = ShooterConfig.getShooterPose(robotPose);

    var shooterTranslation = shooterPose.getTranslation();

    // Calculate speeds of turret (x, y, omega)
    var shooterFieldRelativeSpeeds =
        ShooterConfig.getShooterSpeeds(fieldRelativeSpeeds, robotPose.getRotation().getDegrees());

    Translation2d hubTranslation = FieldUtil.HUB_POSE.getTranslation();

    // Get velocity compensated goals
    var separatedVelocityCompensatedGoal =
        SCORING_SOTM.getSeparatedVelocityCompensatedGoalWithEffectiveTof(
            shooterTranslation, hubTranslation, shooterFieldRelativeSpeeds);

    // Calculate fully compensated distance to goal for shooter, hood, and turret
    var compensatedGoal = separatedVelocityCompensatedGoal.fullyCompensatedGoal();
    var fullyCompensatedDistanceToGoal = shooterTranslation.getDistance(compensatedGoal);

    var swerveAngle =
        ShooterConfig.calculateAimingAngle(shooterTranslation, compensatedGoal).getDegrees();
    // Use same goal for goal centric turret tolerance
    var swerveTolerance =
        ShooterConfig.getGoalCentricTolerance(
            compensatedGoal, shooterPose, SCORING_GOAL_CENTRIC_TOLERANCE);

    // Calculate translational FF for turret to account for linear turret velocity
    var realDistanceToGoal = shooterTranslation.getDistance(hubTranslation);
    double tangentialVelocity = separatedVelocityCompensatedGoal.tangentialVelocity();
    double translationalFF = tangentialVelocity / realDistanceToGoal;

    // Sum translational FF and rotational FF
    double swerveFeedForwardRadians = -translationalFF;

    return new AimingParameters(
        swerveAngle, fullyCompensatedDistanceToGoal, swerveTolerance, swerveFeedForwardRadians);
  }

  public static AimingParameters getStaticFeedingParameters(
      FeedLocation feedLocation, Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
    // Calculate translation of turret on the field
    var shooterPose = ShooterConfig.getShooterPose(robotPose);

    var shooterTranslation = shooterPose.getTranslation();

    // Calculate speeds of turret (x, y, omega)
    var shooterFieldRelativeSpeeds =
        ShooterConfig.getShooterSpeeds(fieldRelativeSpeeds, robotPose.getRotation().getDegrees());

    Translation2d feedTranslation = feedLocation.getTranslation(shooterPose);

    // Get velocity compensated goals
    var separatedVelocityCompensatedGoal =
        FEEDING_SOTM.getSeparatedVelocityCompensatedGoalWithEffectiveTof(
            shooterTranslation, feedTranslation, shooterFieldRelativeSpeeds);

    var swerveAngle =
        ShooterConfig.calculateAimingAngle(shooterTranslation, feedTranslation).getDegrees();
    // Use same goal for goal centric turret tolerance
    var swerveTolerance =
        ShooterConfig.getGoalCentricTolerance(
            feedTranslation, shooterPose, FEEDING_GOAL_CENTRIC_TOLERANCE);

    // Calculate translational FF for turret to account for linear turret velocity
    var realDistanceToGoal = shooterTranslation.getDistance(feedTranslation);
    double tangentialVelocity = separatedVelocityCompensatedGoal.tangentialVelocity();
    double translationalFF = tangentialVelocity / realDistanceToGoal;

    // Sum translational FF and rotational FF
    double swerveFeedForwardRadians = -translationalFF;

    return new AimingParameters(
        swerveAngle, realDistanceToGoal, swerveTolerance, swerveFeedForwardRadians);
  }

  public static AimingParameters getStaticScoringParameters(
      Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
    // Calculate translation of turret on the field
    var shooterPose = ShooterConfig.getShooterPose(robotPose);

    var shooterTranslation = shooterPose.getTranslation();

    // Calculate speeds of turret (x, y, omega)
    var shooterFieldRelativeSpeeds =
        ShooterConfig.getShooterSpeeds(fieldRelativeSpeeds, robotPose.getRotation().getDegrees());

    Translation2d hubTranslation = FieldUtil.HUB_POSE.getTranslation();

    // Get velocity compensated goals
    var separatedVelocityCompensatedGoal =
        SCORING_SOTM.getSeparatedVelocityCompensatedGoalWithEffectiveTof(
            shooterTranslation, hubTranslation, shooterFieldRelativeSpeeds);

    var swerveAngle =
        ShooterConfig.calculateAimingAngle(shooterTranslation, hubTranslation).getDegrees();
    // Use same goal for goal centric turret tolerance
    var swerveTolerance =
        ShooterConfig.getGoalCentricTolerance(
            hubTranslation, shooterPose, SCORING_GOAL_CENTRIC_TOLERANCE);

    // Calculate translational FF for turret to account for linear turret velocity
    var realDistanceToGoal = shooterTranslation.getDistance(hubTranslation);
    double tangentialVelocity = separatedVelocityCompensatedGoal.tangentialVelocity();
    double translationalFF = tangentialVelocity / realDistanceToGoal;

    // Sum translational FF and rotational FF
    double swerveFeedForwardRadians = -translationalFF;

    return new AimingParameters(
        swerveAngle, realDistanceToGoal, swerveTolerance, swerveFeedForwardRadians);
  }

  public record AimingParameters(
      double goalAngle, double distance, double swerveTolerance, double swerveFeedForwardRadians) {}
}
