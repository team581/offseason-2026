package frc.robot.config;

import com.team581.trailblazer3.model.AutoSegment;
import com.team581.trailblazer3.model.FieldClearance;
import com.team581.trailblazer3.model.FieldGeometry;
import com.team581.trailblazer3.model.FieldSymmetry;
import com.team581.util.FieldUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.List;

public final class CompAutoField {
  public static final Pose2d RIGHT_TRENCH_START =
      new Pose2d(FieldUtil.BLUE_OUTPOST_TRENCH_CENTER.getTranslation(), Rotation2d.k180deg);
  private static final double TRENCH_HALF_WIDTH_METERS = Units.inchesToMeters(48.94 / 2);
  private static final FieldGeometry FIELD =
      new FieldGeometry(
          FieldUtil.FIELD_LENGTH_X, FieldUtil.FIELD_WIDTH_Y, FieldSymmetry.ROTATIONAL_180);

  public static FieldClearance clearance(Alliance alliance) {
    var center = pose(RIGHT_TRENCH_START, alliance);
    double low = center.getY() - TRENCH_HALF_WIDTH_METERS;
    double high = center.getY() + TRENCH_HALF_WIDTH_METERS;
    double near = center.getX() - Units.inchesToMeters(47.0 / 2);
    double far = center.getX() + Units.inchesToMeters(47.0 / 2);
    return new FieldClearance(
        FIELD,
        CompTrailblazer.BUMPER_HALF_SIZE_METERS,
        CompTrailblazer.BUMPER_HALF_SIZE_METERS,
        0.04,
        List.of(
            rectangle(near, -1, far, low), rectangle(near, high, far, FIELD.widthMeters() + 1)));
  }

  public static Pose2d pose(Pose2d blue, Alliance alliance) {
    return alliance == Alliance.Blue ? blue : FIELD.flipPose(blue);
  }

  public static AutoSegment segment(AutoSegment blue, Alliance alliance) {
    return blue.forAlliance(alliance, FIELD);
  }

  private static List<Translation2d> rectangle(
      double left, double bottom, double right, double top) {
    return List.of(
        new Translation2d(left, bottom),
        new Translation2d(right, bottom),
        new Translation2d(right, top),
        new Translation2d(left, top));
  }

  private CompAutoField() {}
}
