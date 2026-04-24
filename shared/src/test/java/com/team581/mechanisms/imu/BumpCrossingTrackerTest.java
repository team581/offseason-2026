package com.team581.mechanisms.imu;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

final class BumpCrossingTrackerTest {
  private static final double EPSILON = 1e-9;

  @Test
  void autoGeometryDownhillReportsNegative() {
    // Same geometry, but field-frame tilt pointing in +X (opposite of crossing direction 180).
    var yawDeg = 42.8;
    var yawRad = Math.toRadians(yawDeg);
    var fieldX = 10.0;
    var fieldY = 0.0;
    var pitch = fieldX * Math.cos(-yawRad) - fieldY * Math.sin(-yawRad);
    var roll = fieldX * Math.sin(-yawRad) + fieldY * Math.cos(-yawRad);

    var result =
        BumpCrossingTracker.calculateDirectionalTilt(pitch, roll, yawDeg, Rotation2d.k180deg);

    assertEquals(-10.0, result, EPSILON);
  }

  @Test
  void autoGeometryUphillReportsPositive() {
    // Simulates right-normal auto: yaw 42.8 degrees, crossing direction 180 degrees.
    // Construct a robot-frame tilt that corresponds to a field-frame tilt of 10 degrees
    // pointing in the -X field direction (i.e. uphill aligned with crossing direction 180).
    // Inverse rotation: field (-10, 0) rotated by -42.8 into robot frame.
    var yawDeg = 42.8;
    var yawRad = Math.toRadians(yawDeg);
    var fieldX = -10.0;
    var fieldY = 0.0;
    var pitch = fieldX * Math.cos(-yawRad) - fieldY * Math.sin(-yawRad);
    var roll = fieldX * Math.sin(-yawRad) + fieldY * Math.cos(-yawRad);

    var result =
        BumpCrossingTracker.calculateDirectionalTilt(pitch, roll, yawDeg, Rotation2d.k180deg);

    assertEquals(10.0, result, EPSILON);
  }

  @Test
  void flatRobotHasZeroTilt() {
    var result = BumpCrossingTracker.calculateDirectionalTilt(0, 0, 0, Rotation2d.k180deg);

    assertEquals(0.0, result, EPSILON);
  }

  @Test
  void magnitudeUsesHypotOfPitchAndRoll() {
    // 3-4-5 triangle: pitch=3, roll=4 => hypot=5. Yaw=0, so robot frame == field frame.
    // Field-frame tilt is (3, 4). Crossing direction +X, projection = 3 (positive).
    // Magnitude returned is hypot = 5.
    var result = BumpCrossingTracker.calculateDirectionalTilt(3, 4, 0, Rotation2d.kZero);

    assertEquals(5.0, result, EPSILON);
  }

  @Test
  void pitchedNoseDownAlignedWithCrossingDirectionReportsNegative() {
    // Robot yaw 0, pitch -10 means robot-frame tilt (-10, 0) -> field frame (-10, 0).
    // Crossing direction is +X. Projection is negative -> downhill.
    var result = BumpCrossingTracker.calculateDirectionalTilt(-10, 0, 0, Rotation2d.kZero);

    assertEquals(-10.0, result, EPSILON);
  }

  @Test
  void pitchedNoseUpAlignedWithCrossingDirectionReportsPositive() {
    // Robot yaw 0 (facing +X field). Pitch 10 means robot-frame tilt vector (10, 0).
    // Rotated to field frame by yaw 0, tilt is still (10, 0) -> pointing +X.
    // Crossing direction is +X (0 degrees), so projection is positive.
    var result = BumpCrossingTracker.calculateDirectionalTilt(10, 0, 0, Rotation2d.kZero);

    assertEquals(10.0, result, EPSILON);
  }

  @Test
  void pureRollContributesWhenYawed90Degrees() {
    // Robot yaw 90, pitch 0, roll 10 -> robot-frame tilt (0, 10) rotated by 90 -> field (-10, 0).
    // Crossing direction is +X. Projection is negative.
    var result = BumpCrossingTracker.calculateDirectionalTilt(0, 10, 90, Rotation2d.kZero);

    assertEquals(-10.0, result, EPSILON);
  }

  @Test
  void robotFacingOppositeOfCrossingDirectionFlipsSign() {
    // Robot yaw 180, pitch 10 -> robot-frame tilt (10, 0) rotated by 180 -> field frame (-10, 0).
    // Crossing direction is +X. Field-frame tilt points in -X -> projection negative -> downhill.
    var result = BumpCrossingTracker.calculateDirectionalTilt(10, 0, 180, Rotation2d.kZero);

    assertEquals(-10.0, result, EPSILON);
  }
}
