package frc.robot.util;

import com.team581.math.ShootOnTheMove;
import com.team581.util.FeedLocation;
import com.team581.util.FieldUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.turret.TurretCalculator;

public class AimParameterUtil {
  public static AimingParameters getFeedingParameters(
      FeedLocation feedLocation,
      Pose2d robot,
      ChassisSpeeds fieldRelativeSpeeds,
      double currentTimeofFlight) {

    var feedGoalTranslation =
        ShootOnTheMove.getVelocityCompensatedGoal(
            feedLocation.getTranslation(robot), fieldRelativeSpeeds, currentTimeofFlight);
    var feedGoalAngle = TurretCalculator.calculateTurretAimingAngle(robot, feedGoalTranslation);
    var feedDistance = robot.getTranslation().getDistance(feedGoalTranslation);

    return new AimingParameters(feedGoalAngle, feedDistance);
  }

  public static AimingParameters getScoringParameters(
      Pose2d robot, ChassisSpeeds fieldRelativeSpeeds, double currentTimeofFlight) {
    var hubGoalPose =
        ShootOnTheMove.getVelocityCompensatedGoal(
            FieldUtil.HUB_POSE.getPose().getTranslation(),
            fieldRelativeSpeeds,
            currentTimeofFlight);

    var robotPoseInAllianceZone = FieldUtil.clampPoseToAllianceZone(robot);
    double scoringAngle =
        TurretCalculator.calculateTurretAimingAngle(robotPoseInAllianceZone, hubGoalPose);
    double hubDistance = robotPoseInAllianceZone.getTranslation().getDistance(hubGoalPose);

    return new AimingParameters(scoringAngle, hubDistance);
  }

  public record AimingParameters(double angle, double distance) {}
}
