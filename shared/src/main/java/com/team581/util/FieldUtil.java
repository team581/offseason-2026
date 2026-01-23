package com.team581.util;

import java.util.List;
import java.util.Optional;

import com.google.common.collect.ImmutableList;
import com.team581.autos.Point;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FieldUtil {
  private static final double EXTRA_NEUTRAL_ZONE_THRESHOLD = 0.5;
  // In meters
  public static final double FIELD_LENGTH = 16.540988;
  public static final double FIELD_WIDTH = 8.069326;

  public static final Point HUB_POSE =
      Point.ofRed(new Pose2d(11.915394, 4.034663, Rotation2d.kZero));
      public static final double HUB_RADIUS_METERS = Units.inchesToMeters(48.106087/2);
      public static final Point FEED_1_POSE = Point.ofRed(new Pose2d(15.75, 0.75, Rotation2d.kZero));
      public static final Point FEED_2_POSE = Point.ofRed(new Pose2d(15.75, 7.25, Rotation2d.kZero));

      private static final double BLUE_STARTING_LINE_X = Units.inchesToMeters(156.61);
      private static final double BLUE_TRENCH_X = Units.inchesToMeters(182.11);
      private static final double RED_TRENCH_X = FIELD_LENGTH - BLUE_TRENCH_X;

      private static final double RED_STARTING_LINE_X = FIELD_LENGTH - BLUE_STARTING_LINE_X;

      private static final double TRENCH_LENGTH_Y = Units.inchesToMeters(49.84);

      private static final Rectangle2d BLUE_ALLIANCE_ZONE =
      new Rectangle2d(Translation2d.kZero, new Translation2d(BLUE_STARTING_LINE_X, FIELD_WIDTH));
      private static final Rectangle2d RED_ALLIANCE_ZONE =
      new Rectangle2d(
        new Translation2d(RED_STARTING_LINE_X, 0.0),
        new Translation2d(FIELD_LENGTH, FIELD_WIDTH));

        public static final Rectangle2d RED_HUB_NO_FEED_ZONE =
        new Rectangle2d(
            new Translation2d(RED_STARTING_LINE_X-(HUB_RADIUS_METERS*2), 3.034663), new Translation2d(RED_STARTING_LINE_X-(HUB_RADIUS_METERS*2)-1.0, 5.034663));
             public static final Rectangle2d BLUE_HUB_NO_FEED_ZONE =
        new Rectangle2d(
            new Translation2d(BLUE_STARTING_LINE_X+(HUB_RADIUS_METERS*2), 3.034663), new Translation2d(BLUE_STARTING_LINE_X+(HUB_RADIUS_METERS*2)+1.0, 5.034663));
        // Custom zones to enable trench assist for driver to cleanly drive through with speed
        public static final double BOTTOM_TRENCH_Y = TRENCH_LENGTH_Y / 2.0;
  public static final double TOP_TRENCH_Y = FIELD_WIDTH - TRENCH_LENGTH_Y / 2.0;
  private static final double TRENCH_ASSIST_ZONE_LENGTH_X = Units.inchesToMeters(140);
  private static final double TRENCH_ASSIST_ZONE_LENGTH_Y = Units.inchesToMeters(75.0);

  // TODO: we're never using "right" or "left" in the names again it's gonna be OUTPOST_SIDE or
  // DEPOT_SIDE
  private static final Rectangle2d RED_DEPOT_SIDE_TRENCH_ASSIST_ZONE =
      new Rectangle2d(
          new Translation2d(RED_TRENCH_X - TRENCH_ASSIST_ZONE_LENGTH_X / 2.0, 0.0),
          new Translation2d(
              RED_TRENCH_X + TRENCH_ASSIST_ZONE_LENGTH_X / 2.0, TRENCH_ASSIST_ZONE_LENGTH_Y));
  private static final Rectangle2d RED_OUTPOST_SIDE_TRENCH_ASSIST_ZONE =
      new Rectangle2d(
          new Translation2d(
              RED_TRENCH_X - TRENCH_ASSIST_ZONE_LENGTH_X / 2.0,
              FIELD_WIDTH - TRENCH_ASSIST_ZONE_LENGTH_Y),
          new Translation2d(RED_TRENCH_X + TRENCH_ASSIST_ZONE_LENGTH_X / 2.0, FIELD_WIDTH));
  private static final Rectangle2d BLUE_OUTPOST_SIDE_TRENCH_ASSIST_ZONE =
      new Rectangle2d(
          new Translation2d(
              BLUE_TRENCH_X - TRENCH_ASSIST_ZONE_LENGTH_X / 2.0,
              FIELD_WIDTH - TRENCH_ASSIST_ZONE_LENGTH_Y),
          new Translation2d(BLUE_TRENCH_X + TRENCH_ASSIST_ZONE_LENGTH_X / 2.0, FIELD_WIDTH));
  private static final Rectangle2d BLUE_DEPOT_SIDE_TRENCH_ASSIST_ZONE =
      new Rectangle2d(
          new Translation2d(BLUE_TRENCH_X - TRENCH_ASSIST_ZONE_LENGTH_X / 2.0, 0.0),
          new Translation2d(
              BLUE_TRENCH_X + TRENCH_ASSIST_ZONE_LENGTH_X / 2.0, TRENCH_ASSIST_ZONE_LENGTH_Y));

  private static final List<Rectangle2d> TRENCH_ASSIST_ZONES =
      ImmutableList.of(
          RED_DEPOT_SIDE_TRENCH_ASSIST_ZONE,
          RED_OUTPOST_SIDE_TRENCH_ASSIST_ZONE,
          BLUE_OUTPOST_SIDE_TRENCH_ASSIST_ZONE,
          BLUE_DEPOT_SIDE_TRENCH_ASSIST_ZONE);

  public static Translation2d clampPoseToAllianceZone(Translation2d pose) {
    var allianceZone = FmsUtil.isRedAlliance() ? RED_ALLIANCE_ZONE : BLUE_ALLIANCE_ZONE;

    return allianceZone.nearest(pose);
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

  private static boolean isPointInsideHub(Translation2d point) {
    double distance = point.getDistance(HUB_POSE.getPose().getTranslation());
    return distance < HUB_RADIUS_METERS;
  }

  private static boolean doesLineCollideWithCircle(
      Translation2d start, Translation2d end) {
    // Return True if there is a collision, False if not

    // Special logic for if the start and/or end point are inside a collision zone (happens in
    // matches from bad localization data)
    if (isPointInsideHub(start) || isPointInsideHub(end)) {
      return true;
    }

    // If start to end dot start to circle is less than 0, use start as closest point.
    double startToEndDotStartToCircle =
        ((end.getX() - start.getX()) * (HUB_POSE.getPose().getX() - start.getX()))
            + ((end.getY() - start.getY()) * (HUB_POSE.getPose().getY() - start.getY()));
    if (startToEndDotStartToCircle < 0) {
      double startToCircleDistance = start.getDistance(HUB_POSE.getPose().getTranslation());
      return startToCircleDistance < HUB_RADIUS_METERS;
    }

    // If end to start dot end to circle is less than 0, use end as closest point.
    double endToStartDotEndToCircle =
        ((start.getX() - end.getX()) * (HUB_POSE.getPose().getX() - end.getX()))
            + ((start.getY() - end.getY()) * (HUB_POSE.getPose().getY() - end.getY()));
    if (endToStartDotEndToCircle < 0) {
      double endToCircleDistance = end.getDistance(HUB_POSE.getPose().getTranslation());
      return endToCircleDistance < HUB_RADIUS_METERS;
    }

    double lineVectorX = end.getX() - start.getX();
    double lineVectorY = end.getY() - start.getY();
    double circleVectorX = HUB_POSE.getPose().getTranslation().getX() - start.getX();
    double circleVectorY = HUB_POSE.getPose().getTranslation().getY() - start.getY();

    double dotProduct = lineVectorX * circleVectorX + lineVectorY * circleVectorY;

    double lineVectorMagnitude = Math.hypot(lineVectorX, lineVectorY);
    double circleVectorMagnitude = Math.hypot(circleVectorX, circleVectorY);

    // Angle = acos(docProduct / (lineVectorMagnitude * circleVectorMagnitude))
    // Closest Point on Line to Center of Circle = circleVectorMagnitude * sin(angle)
    // sin(acos(x)) = sqrt(1 - x^2) - gets rid of trig operations
    double x = dotProduct / (lineVectorMagnitude * circleVectorMagnitude);
    double closestPointOnLineToCircleDistance =
        circleVectorMagnitude * Math.sqrt(1 - Math.pow(x, 2));

    return closestPointOnLineToCircleDistance < HUB_RADIUS_METERS;
  }

  public static boolean isRobotInNoFeedZone(Pose2d robotPose) {
    // Check if line from robot to target collides with hub no feed zone
    var noFeedZone =
        FmsUtil.isRedAlliance() ? RED_HUB_NO_FEED_ZONE : BLUE_HUB_NO_FEED_ZONE;

    return noFeedZone.contains(robotPose.getTranslation());
  }

}
