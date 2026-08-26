package frc.robot.util;

import com.team581.math.MathHelpers;
import com.team581.math.ShootOnTheMove;
import com.team581.util.FeedLocation;
import com.team581.util.FieldUtil;
import com.team581.util.FmsUtil;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import frc.robot.shooter.ShooterConfig;
import frc.robot.turret.TurretCalculator;

public class AimParameterUtil {

  public record AimingParameters(
      double goalAngle,
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

  private static final DoubleSubscriber UPCOMING_TURRET_ANGLE_LOOKAHEAD =
      DogLog.tunable("AimingParameters/UpcomingTurretAngleLookahead", 0.5);

  public static AimingParameters getFallbackFeedingParameters(Rotation2d robotRotation) {
    var fieldRelativeGoal = FmsUtil.isRedAlliance() ? Rotation2d.kZero : Rotation2d.k180deg;
    var turretAngle = fieldRelativeGoal.minus(robotRotation);

    return new AimingParameters(
        turretAngle.getDegrees(),
        FEEDING_FALLBACK_DISTANCE_TO_GOAL,
        FEEDING_FALLBACK_TOLERANCE,
        0,
        0);
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
    double translationalFF = tangentialVelocity / realDistanceToGoal;

    double totalTurretFFRadians =
        -translationalFF - turretFieldRelativeSpeeds.omegaRadiansPerSecond;

    var upcomingTurretAngle =
        TurretCalculator.calculateTurretAimingAngle(
            MathHelpers.getLookaheadPose(
                turretPose, fieldRelativeSpeeds, UPCOMING_TURRET_ANGLE_LOOKAHEAD.get()),
            compensatedGoal);

    return new AimingParameters(
        turretAngle, distance, turretTolerance, totalTurretFFRadians, upcomingTurretAngle);
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
    double translationalFF = tangentialVelocity / distance;

    double totalTurretFFRadians =
        -translationalFF - turretFieldRelativeSpeeds.omegaRadiansPerSecond;

    var upcomingTurretAngle = turretAngle; // For static, upcoming is just the current angle

    return new AimingParameters(
        turretAngle, distance, turretTolerance, totalTurretFFRadians, upcomingTurretAngle);
  }
}
