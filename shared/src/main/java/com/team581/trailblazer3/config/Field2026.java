package com.team581.trailblazer3.config;

import com.google.common.collect.ImmutableList;
import com.team581.trailblazer3.model.FieldClearance;
import com.team581.trailblazer3.model.FieldGeometry;
import com.team581.trailblazer3.model.FieldSymmetry;
import com.team581.util.FieldUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.util.List;

/** Shared 2026 field dimensions and keepouts for robot code and previews. */
public final class Field2026 {
  public static final FieldGeometry FIELD_2026 =
      new FieldGeometry(
          FieldUtil.FIELD_LENGTH_X, FieldUtil.FIELD_WIDTH_Y, FieldSymmetry.ROTATIONAL_180);

  private static final double CLEARANCE_MARGIN_METERS = 0.04;

  private static final double TRENCH_WIDTH_METERS = Units.inchesToMeters(48.94);
  private static final double TRENCH_HALF_DEPTH_METERS = Units.inchesToMeters(47.0 / 2.0);
  private static final double TRENCH_BUMP_BARRIER_WIDTH_METERS = Units.inchesToMeters(12.0);
  private static final double HUB_HALF_WIDTH_METERS = Units.inchesToMeters(47.0 / 2.0);
  private static final double HUB_NET_EXTENSION_X_METERS = Units.inchesToMeters(10.26);
  private static final double HUB_NET_HALF_WIDTH_Y_METERS = Units.inchesToMeters(58.41 / 2.0);
  private static final double BLUE_CLIMB_MAX_X_METERS =
      FieldUtil.FIELD_LENGTH_X - Units.inchesToMeters(608.375814);
  private static final double BLUE_CLIMB_MIN_Y_METERS =
      FieldUtil.FIELD_WIDTH_Y - Units.inchesToMeters(193.718750);
  private static final double BLUE_CLIMB_MAX_Y_METERS =
      FieldUtil.FIELD_WIDTH_Y - Units.inchesToMeters(146.718750);

  /** Builds blue-field clearance for square bumpers with the given half-size in meters. */
  public static FieldClearance clearance(double bumperHalfSizeMeters) {
    double blueObstacleX = FieldUtil.BLUE_OUTPOST_TRENCH_CENTER.getX();
    double blueNear = blueObstacleX - TRENCH_HALF_DEPTH_METERS;
    double blueFar = blueObstacleX + TRENCH_HALF_DEPTH_METERS;
    double rightBarrierLowY = TRENCH_WIDTH_METERS;
    double rightBarrierHighY = TRENCH_WIDTH_METERS + TRENCH_BUMP_BARRIER_WIDTH_METERS;
    double leftBarrierHighY = FIELD_2026.widthMeters() - TRENCH_WIDTH_METERS;
    double leftBarrierLowY = leftBarrierHighY - TRENCH_BUMP_BARRIER_WIDTH_METERS;
    double hubCenterY = FIELD_2026.widthMeters() / 2.0;

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
        FIELD_2026,
        bumperHalfSizeMeters,
        bumperHalfSizeMeters,
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

  private static List<Translation2d> flipPolygon(List<Translation2d> bluePolygon) {
    return bluePolygon.stream().map(FIELD_2026::flipPoint).toList();
  }

  private static List<Translation2d> rectangle(
      double left, double bottom, double right, double top) {
    return ImmutableList.of(
        new Translation2d(left, bottom),
        new Translation2d(right, bottom),
        new Translation2d(right, top),
        new Translation2d(left, top));
  }

  private Field2026() {}
}
