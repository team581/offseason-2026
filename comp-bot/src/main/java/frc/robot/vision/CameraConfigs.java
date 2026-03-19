package frc.robot.vision;

import com.team581.config.CameraConfig;
import com.team581.config.LimelightModel;
import edu.wpi.first.math.util.Units;

public class CameraConfigs {

  public static final CameraConfig FRONT =
      new CameraConfig(
          LimelightModel.FOUR,
          true,
          true,
          // Forward
          Units.inchesToMeters(0.0),
          // Right
          Units.inchesToMeters(0.0),
          // Up
          Units.inchesToMeters(0.0),
          // Pitch
          0.0,
          // Yaw
          0.0,
          // Roll
          0.0);

  public static final CameraConfig LEFT =
      new CameraConfig(
          LimelightModel.FOUR,
          true,
          true,
          // Forward
          Units.inchesToMeters(0.0),
          // Right
          Units.inchesToMeters(0.0),
          // Up
          Units.inchesToMeters(0.0),
          // Pitch
          0.0,
          // Yaw
          0.0,
          // Roll
          0.0);

  public static final CameraConfig RIGHT =
      new CameraConfig(
          LimelightModel.FOUR,
          true,
          true,
          // Forward
          Units.inchesToMeters(0.0),
          // Right
          Units.inchesToMeters(0.0),
          // Up
          Units.inchesToMeters(0.0),
          // Pitch
          0.0,
          // Yaw
          0.0,
          // Roll
          0.0);

  public static final CameraConfig GROUND =
      new CameraConfig(
          LimelightModel.THREE,
          false,
          false,
          // Forward
          Units.inchesToMeters(0.0),
          // Right
          Units.inchesToMeters(0.0),
          // Up
          Units.inchesToMeters(0.0),
          // Pitch
          0.0,
          // Yaw
          0.0,
          // Roll
          0.0);

  private CameraConfigs() {}
}
