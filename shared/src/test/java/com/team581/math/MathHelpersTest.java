package com.team581.math;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.Test;

final class MathHelpersTest {
  @Test
  void roundToTest() {
    var result = MathHelpers.roundTo(123.45, 1);
    var expected = 123.5;

    assertEquals(expected, result);
  }

  @Test
  void signedSqrtTest() {
    var result = MathHelpers.signedSqrt(-36);

    assertEquals(-6, result);
  }

  @Test
  void interpolateChassisSpeedsTest() {
    var a = new ChassisSpeeds(0, 10, 0);
    var b = new ChassisSpeeds(10, 20, 0);
    var t = 0.25;
    var expected = new ChassisSpeeds(2.5, 12.5, 0);

    var result = MathHelpers.interpolate(a, b, t);

    assertEquals(expected, result);
  }

  @Test
  void pathflipTest() {
    var input = new Pose2d(0, 0, Rotation2d.kZero);
    var expected = new Pose2d(16.540988, 8.069326, Rotation2d.k180deg);

    var result = MathHelpers.pathflip(input);

    assertEquals(expected, result);
  }

  @Test
  void getDriveDirectionReturnsDirectionFromStartToEnd() {
    var start = new Translation2d(0, 0);
    var end = new Translation2d(2, 0);

    var result = MathHelpers.getDriveDirection(start, end);

    // Direction from (0,0) to (2,0) should be 0 degrees (pointing right along +X)
    assertEquals(Rotation2d.kZero, result);
  }
}
