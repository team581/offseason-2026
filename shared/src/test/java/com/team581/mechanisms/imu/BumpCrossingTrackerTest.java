package com.team581.mechanisms.imu;

import static org.assertj.core.api.Assertions.assertThat;
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
  void magnitudeReachesHypotWhenFullyAlignedWithCrossingDirection() {
    // pitch=3, roll=4 -> gradient direction in robot frame at atan2(4,3) ~ 53.13 degrees.
    // With yaw=0, that is also the field-frame direction. Projecting onto a crossing
    // direction aligned with the gradient should yield the full magnitude (hypot = 5).
    var crossing = new Rotation2d(Math.atan2(4, 3));
    var result = BumpCrossingTracker.calculateDirectionalTilt(3, 4, 0, crossing);

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
  void returnsScalarProjectionOfTiltOntoCrossingDirection() {
    // 3-4-5 triangle: pitch=3, roll=4. Yaw=0, so robot frame == field frame.
    // Field-frame tilt is (3, 4). Crossing direction +X, projection = 3.
    var result = BumpCrossingTracker.calculateDirectionalTilt(3, 4, 0, Rotation2d.kZero);

    assertEquals(3.0, result, EPSILON);
  }

  @Test
  void robotFacingOppositeOfCrossingDirectionFlipsSign() {
    // Robot yaw 180, pitch 10 -> robot-frame tilt (10, 0) rotated by 180 -> field frame (-10, 0).
    // Crossing direction is +X. Field-frame tilt points in -X -> projection negative -> downhill.
    var result = BumpCrossingTracker.calculateDirectionalTilt(10, 0, 180, Rotation2d.kZero);

    assertEquals(-10.0, result, EPSILON);
  }

  @Test
  void signDoesNotFlipDiscontinuouslyAsYawCrosses45Degrees() {
    // Pitch and roll are both -7.5. The robot-frame tilt gradient is (-7.5, -7.5), which
    // points at 225 degrees. As yaw increases from 0 to 90, the field-frame gradient sweeps
    // from 225 to 315 degrees. Projecting onto crossing direction +X gives a value that
    // varies smoothly through zero around yaw = 45.
    //
    // The previous implementation returned signum(projection) * hypot(pitch, roll), which
    // caused a discontinuous jump from -hypot to +hypot at yaw = 45. The corrected
    // implementation returns the projection itself, which varies smoothly.
    var pitch = -7.5;
    var roll = -7.5;
    var crossing = Rotation2d.kZero;
    var expectedHypot = Math.hypot(pitch, roll);

    var atYaw0 = BumpCrossingTracker.calculateDirectionalTilt(pitch, roll, 0, crossing);
    var atYaw44 = BumpCrossingTracker.calculateDirectionalTilt(pitch, roll, 44, crossing);
    var atYaw46 = BumpCrossingTracker.calculateDirectionalTilt(pitch, roll, 46, crossing);
    var atYaw90 = BumpCrossingTracker.calculateDirectionalTilt(pitch, roll, 90, crossing);

    // At yaw 0, gradient is at 225 deg, projection on +X is -7.5
    assertEquals(-7.5, atYaw0, EPSILON);
    // At yaw 90, gradient is at 315 deg, projection on +X is +7.5
    assertEquals(7.5, atYaw90, EPSILON);
    // Values near yaw 45 should be near zero, NOT jumping between +/- hypot
    assertThat(Math.abs(atYaw44))
        .withFailMessage("Expected magnitude at yaw 44 to be smaller than hypot, got %s", atYaw44)
        .isLessThan(expectedHypot);
    assertThat(Math.abs(atYaw46))
        .withFailMessage("Expected magnitude at yaw 46 to be smaller than hypot, got %s", atYaw46)
        .isLessThan(expectedHypot);
    // The transition through yaw 45 must be continuous: yaw 44 and yaw 46 should be close.
    assertThat(Math.abs(atYaw46 - atYaw44))
        .withFailMessage(
            "Expected smooth transition through yaw 45, but values jumped from %s to %s",
            atYaw44, atYaw46)
        .isLessThan(1.0);
  }
}
