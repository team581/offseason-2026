package com.team581.math;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
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

  public static double getOptimalAngle(
      double target, double current, double minTurretAngle, double maxTurretAngle) {
    target = MathUtil.inputModulus(target, -180, 180);

    // Get smallest delta
    var delta = ((target - current + 180) % 360 + 360) % 360 - 180;

    var option1 = current + delta;

    var option2 = (option1 > 0) ? option1 - 360 : option1 + 360;

    var opt1Valid = (option1 >= minTurretAngle && option1 <= maxTurretAngle);
    var opt2Valid = (option2 >= minTurretAngle && option2 <= maxTurretAngle);

    if (opt1Valid && opt2Valid) {
      // If both are reachable, pick the with the least movement
      return MathUtil.clamp(
          (Math.abs(option1 - current) <= Math.abs(option2 - current)) ? option1 : option2,
          minTurretAngle,
          maxTurretAngle);
    } else if (opt1Valid) {
      return MathUtil.clamp(option1, minTurretAngle, maxTurretAngle);
    } else if (opt2Valid) {
      return MathUtil.clamp(option2, minTurretAngle, maxTurretAngle);
    } else {
      return MathUtil.clamp(option1, minTurretAngle, maxTurretAngle);
    }
  }

  // Look at UnwrapAngleDiagram.png
  public static double getUnwrapAngle(
      double target,
      double current,
      double minTurretAngle,
      double maxTurretAngle,
      double tolerance) {

    var totalRangeOfMotion = Math.abs(maxTurretAngle) + Math.abs(minTurretAngle);
    if (totalRangeOfMotion <= 360) {
      return target;
    }

    // Make sure that no angle can be equally in both ends' bad range
    var maxTolerance = (totalRangeOfMotion - 360) / 4;
    tolerance = MathUtil.clamp(tolerance, 0, maxTolerance);
    target = getOptimalAngle(target, current, minTurretAngle, maxTurretAngle);
    DogLog.log("Turret/TARGET", target);
    if (MathUtil.isNear(maxTurretAngle - tolerance, target, tolerance)) {
      DogLog.timestamp("Turret/BY_UPPER_END");
      return MathUtil.clamp(target - 360, minTurretAngle, maxTurretAngle);
    }

    if (MathUtil.isNear(minTurretAngle + tolerance, target, tolerance)) {
      DogLog.timestamp("Turret/BY_LOWER_END");

      return MathUtil.clamp(target + 360, minTurretAngle, maxTurretAngle);
    }

    DogLog.timestamp("Turret/NO_UNWRAP_NEEDED");

    return target;
  }
}
