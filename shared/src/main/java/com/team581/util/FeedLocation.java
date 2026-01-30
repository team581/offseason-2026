package com.team581.util;

import edu.wpi.first.math.geometry.Pose2d;

public enum FeedLocation {
  /** The default feeding location, whatever is closest to the robot. */
  LEFT,
  RIGHT,
  CLOSEST;

  public static FeedLocation getNearest(Pose2d robot) {
    var yDistanceToLeft = Math.abs(robot.getY() - FieldUtil.FEED_LEFT_POSE.getY());
    var yDistanceToRight = Math.abs(robot.getY() - FieldUtil.FEED_RIGHT_POSE.getY());
    var closestFeedLocation = Math.min(yDistanceToLeft, yDistanceToRight);
    if (closestFeedLocation == yDistanceToLeft) {
      return FeedLocation.LEFT;
    }

    return FeedLocation.RIGHT;
  }
}
