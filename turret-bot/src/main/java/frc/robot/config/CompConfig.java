package frc.robot.config;

import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.team581.config.CameraConfig;
import com.team581.config.LimelightModel;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.config.RobotConfig.SwerveConfig;
import frc.robot.config.RobotConfig.VisionConfig;

class CompConfig {

  private static final String RIO_CAN_NAME = "rio";

  public static final RobotConfig COMPETITION_BOT =
      new RobotConfig(
          new VisionConfig(
              0.005,
              0.8,
              // Translation: Positive X = Forward, Positive Y = Left, Positive Z = Up
              // Rotation: Positive X = Roll Right, Positive Y = Pitch Down, Positive Z = Yaw Left

              // Robot pose to calibration rig
              new Pose3d(
                  0.0,
                  Units.inchesToMeters(0.0),
                  Units.inchesToMeters(0.0),
                  new Rotation3d(0.0, 0.0, 0.0)),

              // Limelight position relative to robot bellypan center (meters)
              // Limelight class takes this in to set position from code
              new CameraConfig(
                  LimelightModel.THREE, true, 0.261747, 0.0, 0.235966, 22.0, 0.0, 0.0)),
          new SwerveConfig(new PhoenixPIDController(5.75, 0, 0), true, true, true), null);

  private CompConfig() {}
}
