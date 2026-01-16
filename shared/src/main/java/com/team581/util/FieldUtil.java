package com.team581.util;

import edu.wpi.first.math.geometry.Pose2d;

public class FieldUtil {
  // In Meters
  public static double FIELD_LENGTH = 16.5354;
  public static double FIELD_WIDTH = 8.07;

  // TODO: fill out poses
  public static Pose2d RED_HUB_POSE = new Pose2d();

  public static Pose2d BLUE_HUB_POSE = new Pose2d();

  public static Pose2d getHubPose(boolean isRedAlliance) {
    return isRedAlliance ? FieldUtil.RED_HUB_POSE : FieldUtil.BLUE_HUB_POSE;
  }
}
