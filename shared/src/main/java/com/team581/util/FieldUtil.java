package com.team581.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class FieldUtil {
  // In Meters
  public static double FIELD_LENGTH = 16.5354;
  public static double FIELD_WIDTH = 8.07;

  // TODO: fill out poses

  // Numbers taken from field drawing
  public static Pose2d RED_HUB_POSE = new Pose2d(11.91, 4.035, Rotation2d.kZero);

  public static Pose2d BLUE_HUB_POSE = new Pose2d();

  // Heuristic pose
  public static Pose2d RED_FEED_POSE = new Pose2d(14.479, 5.748, Rotation2d.kZero);

  public static Pose2d getHubPose() {
    return FmsUtil.isRedAlliance() ? FieldUtil.RED_HUB_POSE : FieldUtil.BLUE_HUB_POSE;
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
