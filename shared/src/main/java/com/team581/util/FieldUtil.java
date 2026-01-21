package com.team581.util;

import com.google.common.collect.ImmutableList;
import com.team581.autos.Point;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.util.List;
import java.util.Optional;

public class FieldUtil {
  public static final double EXTRA_NEUTRAL_ZONE_THRESHOLD = 0.5;
  // In meters
  public static final double FIELD_LENGTH = 16.540988;
  public static final double FIELD_WIDTH = 8.069326;

  public static final Point HUB_POSE =
      Point.ofRed(new Pose2d(11.915394, 4.034663, Rotation2d.kZero));
  public static final Point FEED_1_POSE = Point.ofRed(new Pose2d(15.75, 0.75, Rotation2d.kZero));
  public static final Point FEED_2_POSE = Point.ofRed(new Pose2d(15.75, 7.25, Rotation2d.kZero));

  // calculations

  // given
  public static final double RED_STARTING_LINE_X = Units.inchesToMeters(156.61);
  public static final double RED_TRENCH_X = Units.inchesToMeters(182.11);
  public static final double TRENCH_WIDTH_Y = Units.inchesToMeters(25.62 * 2);

  // calculated
  public static final double BLUE_STARTING_LINE_X = FIELD_LENGTH - RED_STARTING_LINE_X;

  public static final double RED_TRENCH_BOX_LENGTH_X = RED_STARTING_LINE_X + 2*(RED_TRENCH_X-RED_STARTING_LINE_X);
  public static final double BLUE_TRENCH_BOX_LENGTH_X = FIELD_LENGTH-RED_TRENCH_BOX_LENGTH_X;

  public static final double TOPSIDE_TO_DOWN_TRENCH_WIDTH_Y = FIELD_WIDTH-TRENCH_WIDTH_Y;

  // end of calculations
  public static final Rectangle2d RED_DEPOT_SIDE_TRENCH_BOX =
      new Rectangle2d(
          new Translation2d(RED_STARTING_LINE_X, FIELD_WIDTH),
          new Translation2d(RED_TRENCH_BOX_LENGTH_X, TOPSIDE_TO_DOWN_TRENCH_WIDTH_Y));
  public static final Rectangle2d RED_OUTPOST_SIDE_TRENCH_BOX =
      new Rectangle2d(
          new Translation2d(RED_STARTING_LINE_X, 0.0),
          new Translation2d(RED_TRENCH_BOX_LENGTH_X, TRENCH_WIDTH_Y));

  public static final Rectangle2d BLUE_OUTPOST_SIDE_TRENCH_BOX =
      new Rectangle2d(
          new Translation2d(BLUE_STARTING_LINE_X, FIELD_WIDTH),
          new Translation2d(BLUE_TRENCH_BOX_LENGTH_X, TOPSIDE_TO_DOWN_TRENCH_WIDTH_Y));
  public static final Rectangle2d BLUE_DEPOT_SIDE_TRENCH_BOX =
      new Rectangle2d(
          new Translation2d(BLUE_STARTING_LINE_X, 0.0),
          new Translation2d(BLUE_TRENCH_BOX_LENGTH_X, TRENCH_WIDTH_Y));

  public static final Rectangle2d RED_ALLIANCE_ZONE =
      new Rectangle2d(
          Translation2d.kZero, new Translation2d(RED_STARTING_LINE_X, FIELD_WIDTH));
  public static final Rectangle2d BLUE_ALLIANCE_ZONE =
      new Rectangle2d(
          new Translation2d(BLUE_STARTING_LINE_X, 0.0),
          new Translation2d(FIELD_LENGTH, FIELD_WIDTH));

  public static final List<Rectangle2d> TRENCH_BOXES =
      ImmutableList.of(
          FieldUtil.BLUE_OUTPOST_SIDE_TRENCH_BOX,
          FieldUtil.BLUE_DEPOT_SIDE_TRENCH_BOX,
          FieldUtil.RED_DEPOT_SIDE_TRENCH_BOX,
          FieldUtil.RED_OUTPOST_SIDE_TRENCH_BOX);

  // TODO: Fill out trench assist zones
  // Custom zones to enable trench assist
  private static final Rectangle2d RED_LEFT_TRENCH_ASSIST_ZONE =
      new Rectangle2d(new Translation2d(), new Translation2d());
  private static final Rectangle2d RED_RIGHT_TRENCH_ASSIST_ZONE =
      new Rectangle2d(new Translation2d(), new Translation2d());
  private static final Rectangle2d BLUE_LEFT_TRENCH_ASSIST_ZONE =
      new Rectangle2d(new Translation2d(), new Translation2d());
  private static final Rectangle2d BLUE_RIGHT_TRENCH_ASSIST_ZONE =
      new Rectangle2d(new Translation2d(), new Translation2d());

  public static final List<Rectangle2d> TRENCH_ASSIST_ZONES =
      ImmutableList.of(
          RED_LEFT_TRENCH_ASSIST_ZONE,
          RED_RIGHT_TRENCH_ASSIST_ZONE,
          BLUE_LEFT_TRENCH_ASSIST_ZONE,
          BLUE_RIGHT_TRENCH_ASSIST_ZONE);

  public static Rectangle2d getAllianceZone() {
    return FmsUtil.isRedAlliance() ? RED_ALLIANCE_ZONE : BLUE_ALLIANCE_ZONE;
  }

  /** Returns the trench assist zone that the robot is currently in, if it exists. */
  public static Optional<Rectangle2d> getCurrentTrenchAssistZone(Translation2d robotPose) {
    return TRENCH_ASSIST_ZONES.stream().filter(zone -> zone.contains(robotPose)).findFirst();
  }

  public static Translation2d getFeed1Pose() {
    return FmsUtil.isRedAlliance()
        ? FEED_1_POSE.redPose().getTranslation()
        : FEED_1_POSE.bluePose().getTranslation();
  }

  public static Translation2d getFeed2Pose() {
    return FmsUtil.isRedAlliance()
        ? FEED_2_POSE.redPose().getTranslation()
        : FEED_2_POSE.bluePose().getTranslation();
  }

  public static Translation2d getHubPose() {
    return FmsUtil.isRedAlliance()
        ? HUB_POSE.redPose().getTranslation()
        : HUB_POSE.bluePose().getTranslation();
  }

  // TODO(@rhetorr): Make smarter for different rotations (would need to store bumper size)
  public static boolean isRobotInAllianceZone(Pose2d robot) {
    var goalX = getHubPose().getX();
    if (FmsUtil.isRedAlliance()) {
      return robot.getX() > goalX + EXTRA_NEUTRAL_ZONE_THRESHOLD;
    }
    return robot.getX() < goalX - EXTRA_NEUTRAL_ZONE_THRESHOLD;
  }
}
