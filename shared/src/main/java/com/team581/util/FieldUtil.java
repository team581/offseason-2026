package com.team581.util;

import com.google.common.collect.ImmutableList;
import com.team581.autos.Point;
import com.team581.math.MathHelpers;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import java.util.List;
import java.util.Optional;

public class FieldUtil {
  private static final double EXTRA_NEUTRAL_ZONE_THRESHOLD = 0.5;

  public static final double FIELD_LENGTH_X = AprilTags.FIELD_LAYOUT.getFieldLength();
  public static final double FIELD_WIDTH_Y = AprilTags.FIELD_LAYOUT.getFieldWidth();
  public static final Rectangle2d FIELD_BOUNDS =
      new Rectangle2d(Translation2d.kZero, new Translation2d(FIELD_LENGTH_X, FIELD_WIDTH_Y));

  public static final Point HUB_POSE =
      Point.ofRed(new Pose2d(11.915394, 4.034663, Rotation2d.kZero));
  public static final double HUB_RADIUS_METERS = Units.inchesToMeters(48.106087 / 2);
  public static final Point FEED_1_POSE = Point.ofRed(new Pose2d(15.75, 0.75, Rotation2d.kZero));
  public static final Point FEED_2_POSE = Point.ofRed(new Pose2d(15.75, 7.25, Rotation2d.kZero));

  private static final double BLUE_STARTING_LINE_X = Units.inchesToMeters(156.61);
  private static final double RED_STARTING_LINE_X = FIELD_LENGTH_X - BLUE_STARTING_LINE_X;

  // OBSTACLE_X referring to the trenches and bumps x coordinate of the corresponding alliance
  private static final double BLUE_OBSTACLE_X =
      MathHelpers.average(AprilTags.TAG_7.getX(), AprilTags.TAG_6.getX());
  private static final double RED_OBSTACLE_X =
      MathHelpers.average(AprilTags.TAG_17.getX(), AprilTags.TAG_28.getX());

  private static final Rectangle2d BLUE_ALLIANCE_ZONE =
      new Rectangle2d(Translation2d.kZero, new Translation2d(BLUE_STARTING_LINE_X, FIELD_WIDTH_Y));
  private static final Rectangle2d RED_ALLIANCE_ZONE =
      new Rectangle2d(
          new Translation2d(RED_STARTING_LINE_X, 0.0),
          new Translation2d(FIELD_LENGTH_X, FIELD_WIDTH_Y));

  private static final Rectangle2d RED_HUB_NO_FEED_ZONE =
      new Rectangle2d(
          new Translation2d(RED_STARTING_LINE_X - (HUB_RADIUS_METERS * 2), 3.034663),
          new Translation2d(RED_STARTING_LINE_X - (HUB_RADIUS_METERS * 2) - 1.0, 5.034663));
  private static final Rectangle2d BLUE_HUB_NO_FEED_ZONE =
      new Rectangle2d(
          new Translation2d(BLUE_STARTING_LINE_X + (HUB_RADIUS_METERS * 2), 3.034663),
          new Translation2d(BLUE_STARTING_LINE_X + (HUB_RADIUS_METERS * 2) + 1.0, 5.034663));

  // TODO: Validate all points & zones
  // Custom zones to enable swerve assist to cleanly drive through field obstacles with speed
  private static final double TRENCH_LENGTH_X = Units.inchesToMeters(47.0);
  private static final double TRENCH_LENGTH_Y = Units.inchesToMeters(48.94);
  private static final double TRENCH_ASSIST_ZONE_LENGTH_X = TRENCH_LENGTH_X * 3;
  private static final double TRENCH_ASSIST_ZONE_LENGTH_Y = Units.inchesToMeters(68.0);

  private static final Pose2d RED_DEPOT_TRENCH_CENTER =
      new Pose2d(RED_OBSTACLE_X, AprilTags.TAG_7.getY(), Rotation2d.kZero);
  private static final Pose2d RED_OUTPOST_TRENCH_CENTER =
      new Pose2d(RED_OBSTACLE_X, AprilTags.TAG_12.getY(), Rotation2d.kZero);
  private static final Pose2d BLUE_DEPOT_TRENCH_CENTER =
      new Pose2d(BLUE_OBSTACLE_X, AprilTags.TAG_17.getY(), Rotation2d.kZero);
  private static final Pose2d BLUE_OUTPOST_TRENCH_CENTER =
      new Pose2d(BLUE_OBSTACLE_X, AprilTags.TAG_22.getY(), Rotation2d.kZero);

  private static final List<Rectangle2d> TRENCH_ZONES =
      ImmutableList.of(
          new Rectangle2d(RED_DEPOT_TRENCH_CENTER, TRENCH_LENGTH_X, TRENCH_LENGTH_Y),
          new Rectangle2d(RED_OUTPOST_TRENCH_CENTER, TRENCH_LENGTH_X, TRENCH_LENGTH_Y),
          new Rectangle2d(BLUE_DEPOT_TRENCH_CENTER, TRENCH_LENGTH_X, TRENCH_LENGTH_Y),
          new Rectangle2d(BLUE_OUTPOST_TRENCH_CENTER, TRENCH_LENGTH_X, TRENCH_LENGTH_Y));

  private static final List<Rectangle2d> TRENCH_ASSIST_ZONES =
      ImmutableList.of(
          // Red depot
          new Rectangle2d(
              RED_DEPOT_TRENCH_CENTER, TRENCH_ASSIST_ZONE_LENGTH_X, TRENCH_ASSIST_ZONE_LENGTH_Y),
          // Red outpost
          new Rectangle2d(
              RED_OUTPOST_TRENCH_CENTER, TRENCH_ASSIST_ZONE_LENGTH_X, TRENCH_ASSIST_ZONE_LENGTH_Y),
          // Blue outpost
          new Rectangle2d(
              BLUE_DEPOT_TRENCH_CENTER, TRENCH_ASSIST_ZONE_LENGTH_X, TRENCH_ASSIST_ZONE_LENGTH_Y),
          // Blue depot
          new Rectangle2d(
              BLUE_OUTPOST_TRENCH_CENTER,
              TRENCH_ASSIST_ZONE_LENGTH_X,
              TRENCH_ASSIST_ZONE_LENGTH_Y));

  // Midpoints of the entryways of trenches; two for each trench, one on alliance zone side, one on
  // neutral zone side
  private static final List<Translation2d> ALLIANCE_ZONE_TRENCH_MIDPOINTS =
      ImmutableList.of(
          // Red depot side
          new Translation2d(RED_OBSTACLE_X + TRENCH_LENGTH_X / 2.0, AprilTags.TAG_7.getY()),
          // Red outpost side
          new Translation2d(RED_OBSTACLE_X + TRENCH_LENGTH_X / 2.0, AprilTags.TAG_12.getY()),
          // Blue depot side
          new Translation2d(BLUE_OBSTACLE_X - TRENCH_LENGTH_X / 2.0, AprilTags.TAG_22.getY()),
          // Blue outpost side
          new Translation2d(BLUE_OBSTACLE_X - TRENCH_LENGTH_X / 2.0, AprilTags.TAG_17.getY()));
  private static final List<Translation2d> NEUTRAL_ZONE_TRENCH_MIDPOINTS =
      ImmutableList.of(
          // Red depot side
          new Translation2d(RED_OBSTACLE_X - TRENCH_LENGTH_X / 2.0, AprilTags.TAG_7.getY()),
          // Red outpost side
          new Translation2d(RED_OBSTACLE_X - TRENCH_LENGTH_X / 2.0, AprilTags.TAG_12.getY()),
          // Blue depot side
          new Translation2d(BLUE_OBSTACLE_X + TRENCH_LENGTH_X / 2.0, AprilTags.TAG_22.getY()),
          // Blue outpost side
          new Translation2d(BLUE_OBSTACLE_X + TRENCH_LENGTH_X / 2.0, AprilTags.TAG_17.getY()));

  private static final double BUMP_LENGTH_X = Units.inchesToMeters(44.4);
  private static final double BUMP_LENGTH_Y = Units.inchesToMeters(73.0);
  // Trench bump border is the little border sitting between them
  private static final double TRENCH_BUMP_BORDER_LENGTH_Y = Units.inchesToMeters(12.0);
  private static final double BUMP_CENTER_Y_DISTANCE_FROM_WALL =
      TRENCH_LENGTH_Y + TRENCH_BUMP_BORDER_LENGTH_Y + (BUMP_LENGTH_Y / 2.0);

  private static final double BUMP_ASSIST_ZONE_LENGTH_X = BUMP_LENGTH_X * 2.0;
  private static final double BUMP_ASSIST_ZONE_LENGTH_Y = BUMP_LENGTH_Y;

  private static final Pose2d RED_DEPOT_BUMP_CENTER =
      new Pose2d(RED_OBSTACLE_X, BUMP_CENTER_Y_DISTANCE_FROM_WALL, Rotation2d.kZero);
  private static final Pose2d RED_OUTPOST_BUMP_CENTER =
      new Pose2d(
          RED_OBSTACLE_X, FIELD_WIDTH_Y - BUMP_CENTER_Y_DISTANCE_FROM_WALL, Rotation2d.kZero);
  private static final Pose2d BLUE_DEPOT_BUMP_CENTER =
      new Pose2d(
          BLUE_OBSTACLE_X, FIELD_WIDTH_Y - BUMP_CENTER_Y_DISTANCE_FROM_WALL, Rotation2d.kZero);
  private static final Pose2d BLUE_OUTPOST_BUMP_CENTER =
      new Pose2d(BLUE_OBSTACLE_X, BUMP_CENTER_Y_DISTANCE_FROM_WALL, Rotation2d.kZero);

  private static final List<Rectangle2d> BUMP_ASSIST_ZONES =
      ImmutableList.of(
          // Red depot
          new Rectangle2d(
              RED_DEPOT_BUMP_CENTER, BUMP_ASSIST_ZONE_LENGTH_X, BUMP_ASSIST_ZONE_LENGTH_Y),
          // Red outpost
          new Rectangle2d(
              RED_OUTPOST_BUMP_CENTER, BUMP_ASSIST_ZONE_LENGTH_X, BUMP_ASSIST_ZONE_LENGTH_Y),
          // Blue outpost
          new Rectangle2d(
              BLUE_DEPOT_BUMP_CENTER, BUMP_ASSIST_ZONE_LENGTH_X, BUMP_ASSIST_ZONE_LENGTH_Y),
          // Blue depot
          new Rectangle2d(
              BLUE_OUTPOST_BUMP_CENTER, BUMP_ASSIST_ZONE_LENGTH_X, BUMP_ASSIST_ZONE_LENGTH_Y));

  public static Translation2d clampPoseToAllianceZone(Translation2d pose) {
    var allianceZone = FmsUtil.isRedAlliance() ? RED_ALLIANCE_ZONE : BLUE_ALLIANCE_ZONE;

    return allianceZone.nearest(pose);
  }

  public static Translation2d getClosestAllianceZoneTrenchMidpoint(Translation2d robotPose) {
    return robotPose.nearest(ALLIANCE_ZONE_TRENCH_MIDPOINTS);
  }

  public static Translation2d getClosestNeutralZoneTrenchMidpoint(Translation2d robotPose) {
    return robotPose.nearest(NEUTRAL_ZONE_TRENCH_MIDPOINTS);
  }

  public static Optional<Rectangle2d> getCurrentBumpAssistZone(Translation2d robotPose) {
    return BUMP_ASSIST_ZONES.stream().filter(zone -> zone.contains(robotPose)).findFirst();
  }

  /** Returns the trench assist zone that the robot is currently in, if it exists. */
  public static Optional<Rectangle2d> getCurrentTrenchAssistZone(Translation2d robotPose) {
    return TRENCH_ASSIST_ZONES.stream().filter(zone -> zone.contains(robotPose)).findFirst();
  }

  // TODO(@rhetorr): Make smarter for different rotations (would need to store bumper size)
  // TODO: This seems like it duplicates functionality you can get from clampPoseToAllianceZone -
  // you can just check if clampPoseToAllianceZone(robotPose) == robotPose
  public static boolean isRobotInAllianceZone(Pose2d robot) {
    var goalX = HUB_POSE.getPose().getX();
    if (FmsUtil.isRedAlliance()) {
      return robot.getX() > goalX + EXTRA_NEUTRAL_ZONE_THRESHOLD;
    }
    return robot.getX() < goalX - EXTRA_NEUTRAL_ZONE_THRESHOLD;
  }

  public static boolean isRobotInNoFeedZone(Pose2d robotPose) {
    // Check if line from robot to target collides with hub no feed zone
    var noFeedZone = FmsUtil.isRedAlliance() ? RED_HUB_NO_FEED_ZONE : BLUE_HUB_NO_FEED_ZONE;

    return noFeedZone.contains(robotPose.getTranslation());
  }

  /**
   * Returns the input pose flipped from red to blue (or vice versa).
   *
   * @param input Pose to transform
   */
  public static Pose2d pathflip(Pose2d input) {
    return input.rotateAround(FIELD_BOUNDS.getCenter().getTranslation(), Rotation2d.k180deg);
  }

  public static boolean robotInTrench(Translation2d robotPose) {
    return TRENCH_ZONES.stream().anyMatch(zone -> zone.contains(robotPose));
  }
}
