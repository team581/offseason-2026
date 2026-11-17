package frc.robot.config;

import static com.google.common.collect.ImmutableList.toImmutableList;

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

  public static final Pose2d RIGHT_TRENCH_EXIT = offset(RIGHT_TRENCH_START, -2.0, 0.0);

  public static final Pose2d LEFT_TRENCH_START =
      new Pose2d(
          RIGHT_TRENCH_START.getX(),
          FIELD.widthMeters() - RIGHT_TRENCH_START.getY(),
          Rotation2d.k180deg);

  private static final double TRENCH_WIDTH_METERS = Units.inchesToMeters(48.94);
  private static final double TRENCH_HALF_DEPTH_METERS = Units.inchesToMeters(47.0 / 2);
  private static final double TRENCH_BUMP_BARRIER_WIDTH_METERS = Units.inchesToMeters(12.0);
  private static final double BUMP_WIDTH_METERS = Units.inchesToMeters(73.0);
  private static final double HUB_HALF_WIDTH_METERS = Units.inchesToMeters(47.0 / 2);
  private static final double HUB_NET_EXTENSION_X_METERS = Units.inchesToMeters(10.26);
  private static final double HUB_NET_HALF_WIDTH_Y_METERS = Units.inchesToMeters(58.41 / 2);
  private static final double BLUE_CLIMB_MAX_X_METERS =
      FieldUtil.FIELD_LENGTH_X - Units.inchesToMeters(608.375814);
  private static final double BLUE_CLIMB_MIN_Y_METERS =
      FieldUtil.FIELD_WIDTH_Y - Units.inchesToMeters(193.718750);
  private static final double BLUE_CLIMB_MAX_Y_METERS =
      FieldUtil.FIELD_WIDTH_Y - Units.inchesToMeters(146.718750);
  private static final double CLEARANCE_MARGIN_METERS = 0.04;

  public static final double RIGHT_TRENCH_Y_METERS = RIGHT_TRENCH_START.getY();
  public static final double RIGHT_BUMP_CENTER_Y_METERS =
      TRENCH_WIDTH_METERS + TRENCH_BUMP_BARRIER_WIDTH_METERS + BUMP_WIDTH_METERS / 2.0;

  public static final FieldClearance CLEARANCE = fullFieldClearance();

  public static FieldClearance clearance(Alliance alliance) {
    return CLEARANCE.forAlliance(alliance);
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

  private static List<Translation2d> flipPolygon(List<Translation2d> bluePolygon) {
    return bluePolygon.stream().map(FIELD::flipPoint).collect(toImmutableList());
  }

  private static FieldClearance fullFieldClearance() {
    double blueObstacleX = RIGHT_TRENCH_START.getX();
    double blueNear = blueObstacleX - TRENCH_HALF_DEPTH_METERS;
    double blueFar = blueObstacleX + TRENCH_HALF_DEPTH_METERS;
    double rightBarrierLowY = TRENCH_WIDTH_METERS;
    double rightBarrierHighY = TRENCH_WIDTH_METERS + TRENCH_BUMP_BARRIER_WIDTH_METERS;
    double leftBarrierHighY = FIELD.widthMeters() - TRENCH_WIDTH_METERS;
    double leftBarrierLowY = leftBarrierHighY - TRENCH_BUMP_BARRIER_WIDTH_METERS;
    double hubCenterY = FIELD.widthMeters() / 2.0;

    var blueRightBarrier = rectangle(blueNear, rightBarrierLowY, blueFar, rightBarrierHighY);
    var blueLeftBarrier = rectangle(blueNear, leftBarrierLowY, blueFar, leftBarrierHighY);
    var blueHub =
        ImmutableList.of(
            new Translation2d(
                blueObstacleX - HUB_HALF_WIDTH_METERS, hubCenterY - HUB_HALF_WIDTH_METERS),
            new Translation2d(
                blueObstacleX + HUB_HALF_WIDTH_METERS + HUB_NET_EXTENSION_X_METERS,
                hubCenterY - HUB_NET_HALF_WIDTH_Y_METERS),
            new Translation2d(
                blueObstacleX + HUB_HALF_WIDTH_METERS + HUB_NET_EXTENSION_X_METERS,
                hubCenterY + HUB_NET_HALF_WIDTH_Y_METERS),
            new Translation2d(
                blueObstacleX - HUB_HALF_WIDTH_METERS, hubCenterY + HUB_HALF_WIDTH_METERS));
    var blueClimbTower =
        rectangle(0.0, BLUE_CLIMB_MIN_Y_METERS, BLUE_CLIMB_MAX_X_METERS, BLUE_CLIMB_MAX_Y_METERS);

    return new FieldClearance(
        FIELD,
        CompTrailblazer.BUMPER_HALF_SIZE_METERS,
        CompTrailblazer.BUMPER_HALF_SIZE_METERS,
        CLEARANCE_MARGIN_METERS,
        ImmutableList.of(
            blueRightBarrier,
            blueLeftBarrier,
            blueHub,
            blueClimbTower,
            flipPolygon(blueRightBarrier),
            flipPolygon(blueLeftBarrier),
            flipPolygon(blueHub),
            flipPolygon(blueClimbTower)));
  }

  private static List<Translation2d> rectangle(
      double left, double bottom, double right, double top) {
    return ImmutableList.of(
        new Translation2d(left, bottom),
        new Translation2d(right, bottom),
        new Translation2d(right, top),
        new Translation2d(left, top));
  }

  private CompAutoField() {}
}
