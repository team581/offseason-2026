package frc.robot.turret;

import com.team581.math.MathHelpers;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class TurretCalculator {
  private static final Transform2d TURRET_TO_ROBOT =
      new Transform2d(Units.inchesToMeters(-0.5), 0.0, Rotation2d.kZero);
  private static final double MIN_TURRET_ANGLE = -149.105;
  private static final double MAX_TURRET_ANGLE = 149.105;

  private static final double SPACE_FROM_HARDSTOP = 30.0;
  private static final double SPACE_FROM_HARDSTOP_TOLERANCE = 2.0;

  public static double calculateSwerveTurretCompensationAngle(
      double wantedTurretAngle, Rotation2d robotRotation) {
    var robotHeading = robotRotation.getDegrees();
    robotHeading = MathHelpers.angleModulus(robotHeading);
    wantedTurretAngle = MathHelpers.angleModulus(wantedTurretAngle);
    var actualTargetRotation = MathHelpers.angleModulus(wantedTurretAngle + robotHeading);
    if (wantedTurretAngle > 0.0) {

      return MathHelpers.angleModulus(actualTargetRotation + 60.0) - MAX_TURRET_ANGLE;
    } else {
      return MathHelpers.angleModulus(actualTargetRotation - 60.0) - MIN_TURRET_ANGLE;
    }
  }

  public static double calculateTurretAimingAngle(
      Translation2d robot, Rotation2d robotRotation, Translation2d target) {
    robot = robot.plus(TURRET_TO_ROBOT.getTranslation().rotateBy(robotRotation));
    var targetAngle =
        Math.toDegrees(Math.atan2(target.getY() - robot.getY(), target.getX() - robot.getX()));
    var robotHeading = robotRotation.getDegrees();

    targetAngle = MathHelpers.angleModulus(targetAngle);
    robotHeading = MathHelpers.angleModulus(robotHeading);

    return MathHelpers.angleModulus(targetAngle - robotHeading);
  }

  public static boolean doesTurretHaveRoom(double turretAngle) {
    if (turretAngle > 0) {
      if (turretAngle < MAX_TURRET_ANGLE - SPACE_FROM_HARDSTOP + SPACE_FROM_HARDSTOP_TOLERANCE) {
        return true;
      }
    }
    if (turretAngle < 0) {
      if (turretAngle > MIN_TURRET_ANGLE + SPACE_FROM_HARDSTOP - SPACE_FROM_HARDSTOP_TOLERANCE) {
        return true;
      }
    }
    return false;
  }
}
