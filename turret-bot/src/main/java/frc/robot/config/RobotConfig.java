package frc.robot.config;

import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.team581.config.CameraConfig;
import edu.wpi.first.math.geometry.Pose3d;

public record RobotConfig(
    VisionConfig vision,
    SwerveConfig swerve,
    TurretConfig turret) {

  public record VisionConfig(
      double xyStdDev,
      double thetaStdDev,
      Pose3d robotPoseRelativeToCalibration,
      CameraConfig mainLimelightConfig) {}

  public record SwerveConfig(
      PhoenixPIDController snapController,
      boolean invertRotation,
      boolean invertX,
      boolean invertY) {}

  public record TurretConfig() {}

  public static RobotConfig get() {
    return CompConfig.COMPETITION_BOT;
  }
}
