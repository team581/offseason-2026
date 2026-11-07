package frc.robot.turret;

import com.team581.math.BaseTurretCalculator;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;

public class TurretCalculator {

  private static final double SPACE_FROM_HARDSTOP = 0.0;
  private static final double SPACE_FROM_HARDSTOP_TOLERANCE = 0.0;

  /**
   * Returns turret rotations, assuming the turret was hand-set within half a motor turn of the
   * homing reference. Wrapping the calibrated rotor delta selects the nearest motor revolution.
   */
  public static double calculateHomedPosition(double rotorPosition) {
    double rotorDelta = MathUtil.inputModulus(rotorPosition, -0.5, 0.5);
    return Units.degreesToRotations(TurretConfig.HOMING_END_POSITION)
        + rotorDelta / TurretConfig.MOTOR_TO_TURRET;
  }

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

  public static double getGoalCentricTurretTolerance(
      Translation2d goalTranslation, Pose2d robotPose, double goalCentricToleranceMeters) {
    return BaseTurretCalculator.getGoalCentricTurretTolerance(
        goalTranslation, robotPose, goalCentricToleranceMeters, TurretConfig.TURRET_TO_ROBOT);
  }

  public static double getOptimalAngle(double target, double current) {
    return BaseTurretCalculator.getOptimalAngle(
        target, current, TurretConfig.MIN_ANGLE, TurretConfig.MAX_ANGLE);
  }

  public static double getSmartUnwrapAngle(double target, double current) {
    return BaseTurretCalculator.getSmartUnwrapAngle(
        target, current, TurretConfig.MIN_ANGLE, TurretConfig.MAX_ANGLE, 80);
  }

  public static ChassisSpeeds getTurretChassisSpeeds(
      ChassisSpeeds robotSpeeds, double robotHeading) {
    return BaseTurretCalculator.getTurretChassisSpeeds(
        robotSpeeds, robotHeading, TurretConfig.TURRET_TO_ROBOT.getTranslation());
  }

  public static Pose2d getTurretPose(Pose2d robot) {
    return BaseTurretCalculator.getTurretPose(robot, TurretConfig.TURRET_TO_ROBOT);
  }
}
