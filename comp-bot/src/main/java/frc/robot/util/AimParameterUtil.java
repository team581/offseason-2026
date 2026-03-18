package frc.robot.util;

import com.team581.math.ShootOnTheMove;
import com.team581.util.FeedLocation;
import com.team581.util.FmsUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.shooter.ShooterConfig;

public class AimParameterUtil {

  private static final ShootOnTheMove SCORING_SOTM =
      new ShootOnTheMove(ShooterConfig.DISTANCE_TO_SCORE_TOF);

  private static final ShootOnTheMove FEEDING_SOTM =
      new ShootOnTheMove(ShooterConfig.DISTANCE_TO_FEED_TOF);

  private static final double FEEDING_FALLBACK_DISTANCE_TO_GOAL = 8.0;

  public static AimingParameters getFallbackFeedingParameters(Rotation2d robotRotation) {
    var fieldRelativeGoal = FmsUtil.isRedAlliance() ? Rotation2d.kZero : Rotation2d.k180deg;
    var turretAngle = fieldRelativeGoal.minus(robotRotation);

    return new AimingParameters(
        turretAngle.getDegrees(), FEEDING_FALLBACK_DISTANCE_TO_GOAL, 5, 0, 0);
  }

  public static AimingParameters getFeedingParameters(
      FeedLocation feedLocation, Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {

    return new AimingParameters(0, 0, 0, 0, 0);
  }

  public static AimingParameters getScoringParameters(
      Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {

    return new AimingParameters(0, 0, 0, 0, 0);
  }

  public static AimingParameters getTurretStuckFeedingParameters(
      FeedLocation feedLocation,
      Pose2d robot,
      double turretAngle,
      ChassisSpeeds fieldRelativeSpeeds) {

    return new AimingParameters(0, 0, 0, 0, 0);
  }

  public record AimingParameters(
      double turretAngle,
      double distance,
      double turretTolerance,
      double turretFeedForwardRadians,
      double upcomingTurretAngle) {}
}
