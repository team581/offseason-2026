package com.team581.util;

import com.team581.autos.Point;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class FieldUtil {
  private static final double EXTRA_NEUTRAL_ZONE_THRESHOLD = 0.5;
  // In meters
  public static double FIELD_LENGTH = 16.540988;
  public static double FIELD_WIDTH = 8.069326;

  public static Point HUB_POSE = Point.ofRed(new Pose2d(11.915394, 4.034663, Rotation2d.kZero));
  public static Point FEED_1_POSE =
      Point.ofRed(new Pose2d(15.75, 0.75, Rotation2d.kZero));
  public static Point FEED_2_POSE =
      Point.ofRed(new Pose2d(15.75, 7.25, Rotation2d.kZero));

  public static Translation2d getHubPose() {
    return FmsUtil.isRedAlliance()
        ? HUB_POSE.redPose().getTranslation()
        : HUB_POSE.bluePose().getTranslation();
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

  // TODO: Make smarter for different rotations (would need to store bumper size)
  public static boolean isRobotInAllianceZone(Pose2d robot) {
    var goalX = getHubPose().getX();
    if (FmsUtil.isRedAlliance()) {
      if (robot.getX() > goalX + EXTRA_NEUTRAL_ZONE_THRESHOLD) {
        return true;
      }
      return false;
    }
    if (robot.getX() < goalX - EXTRA_NEUTRAL_ZONE_THRESHOLD) {
      return true;
    }
    return false;
  }
}
