package frc.robot.autos.auto_state_machines;

import com.team581.autos.Point;
import com.team581.util.FieldUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

final class BumpAutoStart {
  // Same bumper envelope and hub width used by FieldUtil's field geometry.
  static final double HALF_BUMPER_METERS = Units.inchesToMeters(35.0 / 2.0);
  static final double HALF_HUB_METERS = Units.inchesToMeters(47.0 / 2.0);
  static final double RED_STARTING_LINE_X = FieldUtil.FIELD_LENGTH_X - Units.inchesToMeters(156.61);

  static Point atHubCorner(boolean left) {
    return Point.ofRed(
        new Pose2d(
            RED_STARTING_LINE_X + HALF_BUMPER_METERS,
            FieldUtil.RED_HUB_CENTER.getY()
                + (left ? -1.0 : 1.0) * (HALF_HUB_METERS + HALF_BUMPER_METERS),
            Rotation2d.k180deg));
  }

  private BumpAutoStart() {}
}
