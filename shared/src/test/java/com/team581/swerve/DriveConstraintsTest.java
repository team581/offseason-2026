package com.team581.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.Test;

final class DriveConstraintsTest {
  private static final double EPSILON = 1e-9;

  @Test
  void constraintUsesClosestDriverCommandWhenAccelerationWindowIsUnreachable() {
    ChassisSpeeds constrained =
        DriveConstraints.getTurretConstrainedSpeeds(
            new ChassisSpeeds(0.0, 10.0, 0.0),
            Pose2d.kZero,
            new Translation2d(10.0, 0.0),
            Translation2d.kZero,
            -2.0,
            10.0,
            2.0,
            0.1);

    // Full input requires -1 rad/s, which is closer to the allowed -1.8 than stopping at zero.
    assertEquals(10.0, constrained.vyMetersPerSecond, EPSILON);
  }

  @Test
  void turretAccelerationConstraintStartsFromMeasuredTurretVelocity() {
    ChassisSpeeds constrained =
        DriveConstraints.getTurretConstrainedSpeeds(
            new ChassisSpeeds(0.0, 20.0, 0.0),
            Pose2d.kZero,
            new Translation2d(10.0, 0.0),
            Translation2d.kZero,
            -0.8,
            10.0,
            2.0,
            0.1);

    // The requested turret rate is -2 rad/s. From -0.8 rad/s, one cycle permits -1.0 rad/s.
    assertEquals(10.0, constrained.vyMetersPerSecond, EPSILON);
  }

  @Test
  void turretOffsetRotatesWithRobotHeading() {
    double velocity =
        DriveConstraints.getTurretTrackingAngularVelocity(
            new ChassisSpeeds(0.0, 0.0, 1.0),
            new Pose2d(0.0, 0.0, Rotation2d.kCCW_90deg),
            new Translation2d(10.0, 0.0),
            new Translation2d(1.0, 1.0));

    assertEquals(-1.0 + 12.0 / 122.0, velocity, EPSILON);
  }

  @Test
  void turretTrackingVelocityIncludesPivotSwingDuringRotation() {
    double velocity =
        DriveConstraints.getTurretTrackingAngularVelocity(
            new ChassisSpeeds(0.0, 0.0, 1.0),
            Pose2d.kZero,
            new Translation2d(10.0, 0.0),
            new Translation2d(1.0, 1.0));

    assertEquals(-1.0 - 8.0 / 82.0, velocity, EPSILON);
  }

  @Test
  void turretTrackingVelocityUsesOffsetPivotForTranslation() {
    double velocity =
        DriveConstraints.getTurretTrackingAngularVelocity(
            new ChassisSpeeds(0.0, 1.0, 0.0),
            Pose2d.kZero,
            new Translation2d(10.0, 0.0),
            new Translation2d(1.0, 1.0));

    assertEquals(-9.0 / 82.0, velocity, EPSILON);
  }

  @Test
  void turretVelocityConstraintPreservesRequestedChassisBlend() {
    ChassisSpeeds constrained =
        DriveConstraints.getTurretConstrainedSpeeds(
            new ChassisSpeeds(2.0, 10.0, 1.0),
            Pose2d.kZero,
            new Translation2d(10.0, 0.0),
            Translation2d.kZero,
            0.0,
            1.0,
            1000.0,
            0.02);

    assertEquals(1.0, constrained.vxMetersPerSecond, EPSILON);
    assertEquals(5.0, constrained.vyMetersPerSecond, EPSILON);
    assertEquals(0.5, constrained.omegaRadiansPerSecond, EPSILON);
  }

  @Test
  void turretVelocityTargetRampsDownWhileChassisIsStillMoving() {
    double limitedVelocity =
        DriveConstraints.getLimitedTurretTrackingAngularVelocity(0.0, -2.0, 10.0, 2.0, 0.1);

    assertEquals(-1.8, limitedVelocity, EPSILON);
  }
}
