package com.team581.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

final class FeedLocationTest {
  @Test
  void feedLocationLeftTest() {
    Pose2d robotPose = new Pose2d(1, 1.65, new Rotation2d());
    var result = FeedLocation.getNearest(robotPose);
    var exected = FeedLocation.LEFT;

    assertEquals(exected, result);
  }

  @Test
  void feedLocationRightTest() {
    Pose2d robotPose = new Pose2d(1, 6.5, new Rotation2d());
    var result = FeedLocation.getNearest(robotPose);
    var exected = FeedLocation.RIGHT;

    assertEquals(exected, result);
  }
}
