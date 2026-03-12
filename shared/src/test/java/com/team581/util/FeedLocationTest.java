package com.team581.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

final class FeedLocationTest {
  @Test
  void feedLocationLeftTest() {
    Pose2d robotPose = new Pose2d(1, 1.65, new Rotation2d());
    var result = FeedLocation.CLOSEST.getTranslation(robotPose);
    var expected = FieldUtil.FEED_LEFT_POSE.getPose().getTranslation();

    assertEquals(expected, result);
  }

  @Test
  void feedLocationRightTest() {
    Pose2d robotPose = new Pose2d(1, 6.5, new Rotation2d());
    var result = FeedLocation.CLOSEST.getTranslation(robotPose);
    var expected = FieldUtil.FEED_RIGHT_POSE.getPose().getTranslation();

    assertEquals(expected, result);
  }
}
