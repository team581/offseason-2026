package com.team581.util;

import com.team581.autos.Point;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

public enum FeedLocation {
  LEFT(FieldUtil.FEED_LEFT_POSE),
  RIGHT(FieldUtil.FEED_RIGHT_POSE),
  /** The default feeding location, whatever is closest to the robot. */
  CLOSEST(FieldUtil.FEED_RIGHT_POSE);

  public final Point point;

  FeedLocation(Point point) {
    this.point = point;
  }

  private static FeedLocation getNearest(Pose2d robot) {
    var yDistanceToLeft = Math.abs(robot.getY() - FieldUtil.FEED_LEFT_POSE.getY());
    var yDistanceToRight = Math.abs(robot.getY() - FieldUtil.FEED_RIGHT_POSE.getY());
    var closestFeedLocation = Math.min(yDistanceToLeft, yDistanceToRight);
    if (closestFeedLocation == yDistanceToLeft) {
      return FeedLocation.LEFT;
    }

    return FeedLocation.RIGHT;
  }

  public Translation2d getTranslation(Pose2d robot) {
    var resolvedPoint =
        switch (this) {
          case CLOSEST -> getNearest(robot).point;
          default -> point;
        };

    return resolvedPoint.getTranslation();
  }
}
