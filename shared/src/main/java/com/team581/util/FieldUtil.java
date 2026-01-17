package com.team581.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public class FieldUtil {
  // In Meters
  public static double FIELD_LENGTH = 16.5354;
  public static double FIELD_WIDTH = 8.07;

  // TODO: fill out poses

  // Numbers taken from field drawing
  public static Translation2d RED_HUB_POSE = new Translation2d(11.91, 4.035);
  public static Translation2d RED_FEED_1_POSE = new Translation2d(14.479, 5.748);
  public static Translation2d RED_FEED_2_POSE = new Translation2d();

  public static Translation2d BLUE_HUB_POSE = new Translation2d();
  public static Translation2d BLUE_FEED_1_POSE = new Translation2d();
  public static Translation2d BLUE_FEED_2_POSE = new Translation2d();

  public static Translation2d getHubPose() {
    return FmsUtil.isRedAlliance() ? FieldUtil.RED_HUB_POSE : FieldUtil.BLUE_HUB_POSE;
  }

  public static Translation2d getFeed1Pose() {
    return FmsUtil.isRedAlliance() ? FieldUtil.RED_FEED_1_POSE : FieldUtil.BLUE_FEED_1_POSE;
  }

  public static Translation2d getFeed2Pose() {
    return FmsUtil.isRedAlliance() ? FieldUtil.RED_FEED_2_POSE : FieldUtil.BLUE_FEED_2_POSE;
  }

  // TODO: Make smarter for different rotations (would need to store bumper size)
  public static boolean isRobotInAllianceZone(Pose2d robot) {
    var goalX = getHubPose().getX();
    if (FmsUtil.isRedAlliance()) {
      if (robot.getX() > goalX) {
        return true;
      }
      return false;
    }
    if (robot.getX() < goalX) {
      return true;
    }
    return false;
  }
}
