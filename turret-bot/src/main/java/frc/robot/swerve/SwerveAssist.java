package frc.robot.swerve;

import com.team581.math.MathHelpers;
import com.team581.util.FieldUtil;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class SwerveAssist {
  private static final double TRENCH_ASSIST_VELOCITY_THRESHOLD = 1.5;
  private static final double TRENCH_ASSIST_ANGLE_THRESHOLD = 30.0;

  private static final PIDController TRENCH_PID_CONTROLLER = new PIDController(10, 0, 0);

  public static boolean ableToTrenchAssist(Pose2d robotPose, ChassisSpeeds fieldRelativeSpeeds) {
    // Check if in trench assist zone
    if (FieldUtil.getCurrentTrenchAssistZone(robotPose.getTranslation()).isEmpty()) {
      return false;
    }

    // Check if velocity meets threshold
    if (MathHelpers.getLinearVelocity(fieldRelativeSpeeds) <= TRENCH_ASSIST_VELOCITY_THRESHOLD) {
      DogLog.log("SwerveAssist/Trench/VelocityThreshold", false);
      return false;
    }
    DogLog.log("SwerveAssist/Trench/VelocityThreshold", true);

    var allianceZoneAssistPoint =
        FieldUtil.getClosestAllianceZoneTrenchMidpoint(robotPose.getTranslation());
    var neutralZoneAssistPoint =
        FieldUtil.getClosestNeutralZoneTrenchMidpoint(robotPose.getTranslation());

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
            TRENCH_ASSIST_ANGLE_THRESHOLD,
            -180,
            180)
        || MathUtil.isNear(
            velocityAngle.getDegrees(),
            angleToNeutralZoneAssistPoint.getDegrees(),
            TRENCH_ASSIST_ANGLE_THRESHOLD,
            -180,
            180)) {
      DogLog.log("SwerveAssist/Trench/AngleThreshold", true);
      return true;
    }
    return false;
  }

  public static double getTrenchAssistVelocity(Pose2d robotPose) {
    return -TRENCH_PID_CONTROLLER.calculate(
        robotPose.getY(),
        FieldUtil.getClosestAllianceZoneTrenchMidpoint(robotPose.getTranslation()).getY());
  }

  public static double getTrenchSnapAngle(Pose2d robotPose) {
    return Math.round(robotPose.getRotation().getDegrees() / 90.0) * 90.0;
  }
}
