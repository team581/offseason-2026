package frc.robot.turret;

import com.team581.math.BaseTurretCalculator;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class TurretCalculator {

  private static final double SPACE_FROM_HARDSTOP = 0.0;
  private static final double SPACE_FROM_HARDSTOP_TOLERANCE = 0.0;

  public static double calculateSwerveTurretCompensationAngle(
      double wantedTurretAngle, Rotation2d robotRotation) {
    return BaseTurretCalculator.calculateSwerveTurretCompensationAngle(
        wantedTurretAngle, robotRotation, TurretConfig.MIN_ANGLE, TurretConfig.MAX_ANGLE);
  }

  public static double calculateTurretAimingAngle(Pose2d robot, Translation2d target) {
    return BaseTurretCalculator.calculateTurretAimingAngle(
        robot, target, TurretConfig.TURRET_TO_ROBOT);
  }

  public static boolean doesTurretHaveRoom(double turretAngle) {
    return BaseTurretCalculator.doesTurretHaveRoom(
        turretAngle,
        TurretConfig.MIN_ANGLE,
        TurretConfig.MAX_ANGLE,
        SPACE_FROM_HARDSTOP,
        SPACE_FROM_HARDSTOP_TOLERANCE);
  }

  public static double clamp(double wantedAngle) {

    return MathUtil.clamp(wantedAngle, TurretConfig.MIN_ANGLE, TurretConfig.MAX_ANGLE);
  }

  public static double getOptimalAngle(double target, double current) {
    target = MathUtil.inputModulus(target, -180, 180);

    // Get smallest delta
    var delta = ((target - current + 180) % 360 + 360) % 360 - 180;
    DogLog.log("Turret/Delta", delta);

    var option1 = current + delta;
    DogLog.log("Turret/Option1", option1);

    var option2 = (option1 > 0) ? option1 - 360 : option1 + 360;
    DogLog.log("Turret/Option2", option2);

    var opt1Valid = (option1 >= TurretConfig.MIN_ANGLE && option1 <= TurretConfig.MAX_ANGLE);
    var opt2Valid = (option2 >= TurretConfig.MIN_ANGLE && option2 <= TurretConfig.MAX_ANGLE);

    if (opt1Valid && opt2Valid) {
      // If both are reachable, pick the with the least movement
      return clamp(
          (Math.abs(option1 - current) <= Math.abs(option2 - current)) ? option1 : option2);
    } else if (opt1Valid) {
      return clamp(option1);
    } else if (opt2Valid) {
      return clamp(option2);
    } else {
      return clamp(option1);
    }
  }
}
