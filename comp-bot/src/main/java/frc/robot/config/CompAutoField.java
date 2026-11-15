package frc.robot.config;

import com.google.common.collect.ImmutableList;
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
  public static final FieldGeometry FIELD =
      new FieldGeometry(
          FieldUtil.FIELD_LENGTH_X, FieldUtil.FIELD_WIDTH_Y, FieldSymmetry.ROTATIONAL_180);

  public static final Pose2d RIGHT_TRENCH_START =
      new Pose2d(FieldUtil.BLUE_OUTPOST_TRENCH_CENTER.getTranslation(), Rotation2d.k180deg);

  public static final Pose2d LEFT_TRENCH_START =
      new Pose2d(
          RIGHT_TRENCH_START.getX(),
          FIELD.widthMeters() - RIGHT_TRENCH_START.getY(),
          Rotation2d.k180deg);

  private static final double TRENCH_HALF_WIDTH_METERS = Units.inchesToMeters(48.94 / 2);
  private static final double TRENCH_HALF_DEPTH_METERS = Units.inchesToMeters(47.0 / 2);
  private static final double CLEARANCE_MARGIN_METERS = 0.04;

  public static final FieldClearance RIGHT_TRENCH_CLEARANCE =
      trenchClearance(RIGHT_TRENCH_START.getTranslation());

  public static final FieldClearance LEFT_TRENCH_CLEARANCE =
      trenchClearance(LEFT_TRENCH_START.getTranslation());

  public static final FieldClearance CLEARANCE = fullFieldClearance();

  public static FieldClearance clearance(Alliance alliance) {
    return CLEARANCE.forAlliance(alliance);
  }

  public static Pose2d forAlliance(Pose2d bluePose, Alliance alliance) {
    return pose(bluePose, alliance);
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

  private static FieldClearance fullFieldClearance() {
    double rightLow = RIGHT_TRENCH_START.getY() - TRENCH_HALF_WIDTH_METERS;
    double rightHigh = RIGHT_TRENCH_START.getY() + TRENCH_HALF_WIDTH_METERS;
    double leftLow = LEFT_TRENCH_START.getY() - TRENCH_HALF_WIDTH_METERS;
    double leftHigh = LEFT_TRENCH_START.getY() + TRENCH_HALF_WIDTH_METERS;
    double blueNear = RIGHT_TRENCH_START.getX() - TRENCH_HALF_DEPTH_METERS;
    double blueFar = RIGHT_TRENCH_START.getX() + TRENCH_HALF_DEPTH_METERS;
    double redNear = FIELD.lengthMeters() - blueFar;
    double redFar = FIELD.lengthMeters() - blueNear;
    return new FieldClearance(
        FIELD,
        CompTrailblazer.BUMPER_HALF_SIZE_METERS,
        CompTrailblazer.BUMPER_HALF_SIZE_METERS,
        CLEARANCE_MARGIN_METERS,
        ImmutableList.of(
            rectangle(blueNear, -1, blueFar, rightLow),
            rectangle(blueNear, rightHigh, blueFar, leftLow),
            rectangle(blueNear, leftHigh, blueFar, FIELD.widthMeters() + 1),
            rectangle(redNear, -1, redFar, rightLow),
            rectangle(redNear, rightHigh, redFar, leftLow),
            rectangle(redNear, leftHigh, redFar, FIELD.widthMeters() + 1)));
  }

  private static List<Translation2d> rectangle(
      double left, double bottom, double right, double top) {
    return ImmutableList.of(
        new Translation2d(left, bottom),
        new Translation2d(right, bottom),
        new Translation2d(right, top),
        new Translation2d(left, top));
  }

  private static FieldClearance trenchClearance(Translation2d trenchCenter) {
    double low = trenchCenter.getY() - TRENCH_HALF_WIDTH_METERS;
    double high = trenchCenter.getY() + TRENCH_HALF_WIDTH_METERS;
    double near = trenchCenter.getX() - TRENCH_HALF_DEPTH_METERS;
    double far = trenchCenter.getX() + TRENCH_HALF_DEPTH_METERS;
    return new FieldClearance(
        FIELD,
        CompTrailblazer.BUMPER_HALF_SIZE_METERS,
        CompTrailblazer.BUMPER_HALF_SIZE_METERS,
        CLEARANCE_MARGIN_METERS,
        ImmutableList.of(
            rectangle(near, -1, far, low), rectangle(near, high, far, FIELD.widthMeters() + 1)));
  }

  private CompAutoField() {}
}
