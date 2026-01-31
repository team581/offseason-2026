package com.team581.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class BaseTurretCalculator {

  public static double calculateSwerveTurretCompensationAngle(
      double wantedTurretAngle,
      Rotation2d robotRotation,
      double minTurretAngle,
      double maxTurretAngle) {
    var robotHeading = robotRotation.getDegrees();
    robotHeading = MathHelpers.angleModulus(robotHeading);
    wantedTurretAngle = MathHelpers.angleModulus(wantedTurretAngle);
    var actualTargetRotation = MathHelpers.angleModulus(wantedTurretAngle + robotHeading);
    if (wantedTurretAngle > 0.0) {

      return MathHelpers.angleModulus(actualTargetRotation + 60.0) - minTurretAngle;
    } else {
      return MathHelpers.angleModulus(actualTargetRotation - 60.0) - maxTurretAngle;
    }
  }

  public static double calculateTurretAimingAngle(
      Pose2d robot, Translation2d target, Transform2d turretToRobot) {
    var fieldRelativeTurretPose =
        robot.getTranslation().plus(turretToRobot.getTranslation().rotateBy(robot.getRotation()));
    var targetAngle =
        Math.toDegrees(
            Math.atan2(
                target.getY() - fieldRelativeTurretPose.getY(),
                target.getX() - fieldRelativeTurretPose.getX()));
    var robotHeading = robot.getRotation().getDegrees();

    targetAngle = MathHelpers.angleModulus(targetAngle);
    robotHeading = MathHelpers.angleModulus(robotHeading);

    return MathHelpers.angleModulus(targetAngle - robotHeading);
  }

  public static boolean doesTurretHaveRoom(
      double turretAngle,
      double minTurretAngle,
      double maxTurretAngle,
      double wantedSpaceFromHardstop,
      double spaceFromHardstopTolerance) {
    if (turretAngle > 0) {
      if (turretAngle < maxTurretAngle - wantedSpaceFromHardstop + spaceFromHardstopTolerance) {
        return true;
      }
    }
    if (turretAngle < 0) {
      if (turretAngle > minTurretAngle + wantedSpaceFromHardstop - spaceFromHardstopTolerance) {
        return true;
      }
    }
    return false;
  }
}
