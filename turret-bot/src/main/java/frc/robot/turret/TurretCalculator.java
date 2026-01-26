package frc.robot.turret;

import com.team581.math.BaseTurretCalculator;
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
    return BaseTurretCalculator.calculateSwerveTurretCompensationAngle(
        wantedTurretAngle, robotRotation, MIN_TURRET_ANGLE, MAX_TURRET_ANGLE);
  }

  public static double calculateTurretAimingAngle(
      Translation2d robot, Rotation2d robotRotation, Translation2d target) {
    return BaseTurretCalculator.calculateTurretAimingAngle(
        robot, robotRotation, target, TURRET_TO_ROBOT);
  }

  public static boolean doesTurretHaveRoom(double turretAngle) {
    return BaseTurretCalculator.doesTurretHaveRoom(
        turretAngle,
        MIN_TURRET_ANGLE,
        MAX_TURRET_ANGLE,
        SPACE_FROM_HARDSTOP,
        SPACE_FROM_HARDSTOP_TOLERANCE);
  }
}
