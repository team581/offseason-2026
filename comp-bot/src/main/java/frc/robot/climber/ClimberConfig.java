package frc.robot.climber;

import com.team581.autos.Point;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class ClimberConfig {
  // TODO: UPDATE WITH REAL GOAL LOCATIONS FROM CAD
  public static final Point CLIMB_LEFT_LOCATION =
      Point.ofRed(new Pose2d(15.0, 1.0, Rotation2d.kZero));
  public static final Point CLIMB_RIGHT_LOCATION =
      Point.ofRed(new Pose2d(15.0, 7.0, Rotation2d.kZero));
}
