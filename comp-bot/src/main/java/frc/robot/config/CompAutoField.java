package frc.robot.config;

import com.team581.trailblazer3.config.Field2026;
import com.team581.trailblazer3.model.AutoSegment;
import com.team581.trailblazer3.model.FieldClearance;
import com.team581.trailblazer3.model.FieldGeometry;
import com.team581.util.FieldUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class CompAutoField {
  public static final FieldGeometry FIELD = Field2026.FIELD_2026;

  public static final Pose2d RIGHT_TRENCH_START =
      new Pose2d(FieldUtil.BLUE_OUTPOST_TRENCH_CENTER.getTranslation(), Rotation2d.k180deg);

  public static final Pose2d RIGHT_TRENCH_EXIT = offset(RIGHT_TRENCH_START, -3.0, 0.0);

  public static final Pose2d LEFT_TRENCH_START =
      new Pose2d(
          RIGHT_TRENCH_START.getX(),
          FIELD.widthMeters() - RIGHT_TRENCH_START.getY(),
          Rotation2d.k180deg);

  private static final double TRENCH_WIDTH_METERS = Units.inchesToMeters(48.94);
  private static final double TRENCH_BUMP_BARRIER_WIDTH_METERS = Units.inchesToMeters(12.0);
  private static final double BUMP_WIDTH_METERS = Units.inchesToMeters(73.0);

  public static final double RIGHT_TRENCH_Y_METERS = RIGHT_TRENCH_START.getY();
  public static final double RIGHT_BUMP_CENTER_Y_METERS =
      TRENCH_WIDTH_METERS + TRENCH_BUMP_BARRIER_WIDTH_METERS + BUMP_WIDTH_METERS / 2.0;

  public static final FieldClearance CLEARANCE =
      Field2026.clearance(CompDriveConfig.BUMPER_HALF_SIZE_METERS);

  public static FieldClearance clearance(Alliance alliance) {
    return CLEARANCE.forAlliance(alliance);
  }

  /** Accepts blue-side coordinates in meters and a heading in degrees. */
  public static Pose2d fromBlue(double xMeters, double yMeters, double headingDegrees) {
    return pose(xMeters, yMeters, headingDegrees);
  }

  /** Converts red-side coordinates in meters and degrees to the blue authoring frame. */
  public static Pose2d fromRed(double xMeters, double yMeters, double headingDegrees) {
    return FIELD.flipPose(fromBlue(xMeters, yMeters, headingDegrees));
  }

  public static Pose2d offset(Pose2d base, double dxMeters, double dyMeters) {
    return new Pose2d(base.getX() + dxMeters, base.getY() + dyMeters, base.getRotation());
  }

  public static Pose2d offsetWithHeading(
      Pose2d base, double dxMeters, double dyMeters, double headingDegrees) {
    return new Pose2d(
        base.getX() + dxMeters, base.getY() + dyMeters, Rotation2d.fromDegrees(headingDegrees));
  }

  public static Translation2d point(double xMeters, double yMeters) {
    return new Translation2d(xMeters, yMeters);
  }

  public static Pose2d pose(double xMeters, double yMeters, double headingDegrees) {
    return new Pose2d(xMeters, yMeters, Rotation2d.fromDegrees(headingDegrees));
  }

  public static Pose2d pose(Pose2d blue, Alliance alliance) {
    return alliance == Alliance.Blue ? blue : FIELD.flipPose(blue);
  }

  public static AutoSegment segment(AutoSegment blue, Alliance alliance) {
    return blue.forAlliance(alliance, FIELD);
  }

  private CompAutoField() {}
}
