package frc.robot.util;

import com.team581.util.FeedLocation;
import com.team581.util.FieldUtil;
import com.team581.util.FmsUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.shooter.ShooterConfig;

public class AimParameterUtil {
  public record AimingParameters(double goalAngle, double distance, double swerveTolerance) {}

  private static final double FEEDING_GOAL_CENTRIC_TOLERANCE = Units.inchesToMeters(20);
  private static final double SCORING_GOAL_CENTRIC_TOLERANCE = Units.inchesToMeters(10);
  private static final double FEEDING_FALLBACK_DISTANCE_TO_GOAL = 8.0;
  private static final double FEEDING_FALLBACK_TOLERANCE = 5.0;

  public static AimingParameters getFallbackFeedingParameters() {
    var swerveAngle =
        FmsUtil.isRedAlliance() ? Rotation2d.k180deg.getDegrees() : Rotation2d.kZero.getDegrees();
    return new AimingParameters(
        swerveAngle, FEEDING_FALLBACK_DISTANCE_TO_GOAL, FEEDING_FALLBACK_TOLERANCE);
  }

  public static AimingParameters getFeedingParameters(FeedLocation feedLocation, Pose2d robotPose) {
    return getAimingParameters(
        feedLocation.getTranslation(), FEEDING_GOAL_CENTRIC_TOLERANCE, robotPose);
  }

  public static AimingParameters getScoringParameters(Pose2d robotPose) {
    return getAimingParameters(
        FieldUtil.HUB_POSE.getTranslation(), SCORING_GOAL_CENTRIC_TOLERANCE, robotPose);
  }

  private static AimingParameters getAimingParameters(
      Translation2d goalTranslation, double tolerance, Pose2d robotPose) {
    var shooterPose = ShooterConfig.getShooterPose(robotPose);
    var shooterTranslation = shooterPose.getTranslation();
    double distance = shooterTranslation.getDistance(goalTranslation);
    double swerveAngle =
        ShooterConfig.calculateAimingAngle(shooterTranslation, goalTranslation).getDegrees();
    double swerveTolerance =
        ShooterConfig.getGoalCentricTolerance(goalTranslation, shooterPose, tolerance);
    return new AimingParameters(swerveAngle, distance, swerveTolerance);
  }
}
