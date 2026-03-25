package frc.robot.vision;

import com.team581.config.CameraConfig;
import com.team581.config.LimelightModel;
import edu.wpi.first.math.util.Units;
import frc.robot.config.RobotKind;

public class CameraConfigs {
  public static final CameraConfig TURRET =
      new CameraConfig(
          LimelightModel.FOUR,
          true,
          false,
          Units.inchesToMeters(0.0),
          Units.inchesToMeters(0.0),
          Units.inchesToMeters(21.0309),
          14.0,
          0.0,
          0.0);

  public static final CameraConfig BACK =
      new CameraConfig(
          LimelightModel.FOUR,
          true,
          true,
          // back
          Units.inchesToMeters(RobotKind.IS_COMP_BOT ? -12.889 : -13.389),
          // left
          Units.inchesToMeters(RobotKind.IS_COMP_BOT ? -8.3379 : -8.3370),
          Units.inchesToMeters(RobotKind.IS_COMP_BOT ? 19.758 : 19.7564),
          RobotKind.IS_COMP_BOT ? 10.816941 : 10.0,
          RobotKind.IS_COMP_BOT ? -175.592715 : -175.5,
          RobotKind.IS_COMP_BOT ? 0.83 : -0.732935);

  // ground when stowed
  // Units.inchesToMeters(12.9742),
  // Units.inchesToMeters(0.0),
  // Units.inchesToMeters(16.8886)

  public static final CameraConfig GROUND =
      new CameraConfig(
          LimelightModel.THREE,
          false,
          false,
          Units.inchesToMeters(25.671),
          Units.inchesToMeters(0.0),
          Units.inchesToMeters(12.9525),
          -20.0,
          0.0,
          0.0);

  private CameraConfigs() {}
}
