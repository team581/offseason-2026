package com.team581.math;

import com.team581.util.FieldUtil;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveAssist {
  private static final double TRENCH_ASSIST_VELOCITY_THRESHOLD = 0.75;
  private static final double TRENCH_ASSIST_ANGLE_TOLERANCE = 30.0;
  private static final double BUMP_ASSIST_VELOCITY_THRESHOLD = 0.5;
  private static final double BUMP_ASSIST_ANGLE_TOLERANCE = 45.0;
  private static final double ROBOT_INTAKE_TO_BUMP_ANGLE = 0.0;

  private static final PIDController TRENCH_PID_CONTROLLER = new PIDController(10, 0, 0);

  public static boolean ableToBumpAssist(Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
    // Check if in bump assist zone
    if (FieldUtil.getCurrentBumpAssistZone(robotPose.getTranslation()).isEmpty()) {
      return false;
    }

    // Check if velocity meets threshold
    if (MathHelpers.getLinearVelocity(fieldRelativeSpeeds) <= BUMP_ASSIST_VELOCITY_THRESHOLD) {
      DogLog.log("SwerveAssist/Bump/VelocityThreshold", false);
      return false;
    }
    DogLog.log("SwerveAssist/Bump/VelocityThreshold", true);

    // Check if our angle is within the tolerance
    var angleTolerance =
        MathUtil.isNear(
            robotPose.getRotation().getDegrees(),
            getBumpSnapAngle(fieldRelativeSpeeds.vxMetersPerSecond),
            BUMP_ASSIST_ANGLE_TOLERANCE,
            -180.0,
            180.0);
    DogLog.log("SwerveAssist/Bump/AngleTolerance", angleTolerance);
    return angleTolerance;
  }

  public static boolean ableToTrenchAssist(Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
    var robotTranslation = robotPose.getTranslation();
    // Check if in trench assist zone
    if (FieldUtil.getCurrentTrenchAssistZone(robotTranslation).isEmpty()) {
      return false;
    }

    // Check if velocity meets threshold
    if (MathHelpers.getLinearVelocity(fieldRelativeSpeeds) <= TRENCH_ASSIST_VELOCITY_THRESHOLD) {
      DogLog.log("SwerveAssist/Trench/VelocityThreshold", false);
      return false;
    }
    DogLog.log("SwerveAssist/Trench/VelocityThreshold", true);

    var allianceZoneAssistPoint = FieldUtil.getClosestAllianceZoneTrenchMidpoint(robotTranslation);
    var neutralZoneAssistPoint = FieldUtil.getClosestNeutralZoneTrenchMidpoint(robotTranslation);

    DogLog.log(
        "SwerveAssist/Trench/ClosestAllianceZoneTrenchMidpoint",
        new Pose2d(allianceZoneAssistPoint, Rotation2d.kCCW_90deg));
    DogLog.log(
        "SwerveAssist/Trench/ClosestNeutralZoneTrenchMidpoint",
        new Pose2d(neutralZoneAssistPoint, Rotation2d.kCCW_90deg));

    // Check if angle is toward trench
    var velocityAngle = MathHelpers.getDriveDirection(fieldRelativeSpeeds);
    var angleToAllianceZoneAssistPoint =
        MathHelpers.getDriveDirection(robotPose, allianceZoneAssistPoint);
    var angleToNeutralZoneAssistPoint =
        MathHelpers.getDriveDirection(robotPose, neutralZoneAssistPoint);

    if (MathUtil.isNear(
            velocityAngle.getDegrees(),
            angleToAllianceZoneAssistPoint.getDegrees(),
            TRENCH_ASSIST_ANGLE_TOLERANCE,
            -180.0,
            180.0)
        || MathUtil.isNear(
            velocityAngle.getDegrees(),
            angleToNeutralZoneAssistPoint.getDegrees(),
            TRENCH_ASSIST_ANGLE_TOLERANCE,
            -180.0,
            180.0)) {
      DogLog.log("SwerveAssist/Trench/AngleTolerance", true);
      return true;
    }
    DogLog.log("SwerveAssist/Trench/AngleTolerance", false);
    return false;
  }

  public static double getBumpSnapAngle(double vxMetersPerSecond) {
    // Decides which way to snap based on which direction our velocity is going
    return vxMetersPerSecond > 0.0
        ? ROBOT_INTAKE_TO_BUMP_ANGLE
        : ROBOT_INTAKE_TO_BUMP_ANGLE + 180.0;
  }

  public static double getTrenchAssistVelocity(Translation2d robotTranslation) {
    return -TRENCH_PID_CONTROLLER.calculate(
        robotTranslation.getY(),
        FieldUtil.getClosestAllianceZoneTrenchMidpoint(robotTranslation).getY());
  }

  public static double getTrenchSnapAngle(Rotation2d robotHeading) {
    return Math.round(robotHeading.getDegrees() / 90.0) * 90.0;
  }
}
