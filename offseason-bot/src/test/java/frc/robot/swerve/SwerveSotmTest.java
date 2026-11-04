package frc.robot.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.junit.jupiter.api.Test;

final class SwerveSotmTest {
  @Test
  void emptyOrNullTargetsReturnZeroCommand() {
    var kinematics =
        new SwerveDriveKinematics(
            new Translation2d(0.3, 0.3),
            new Translation2d(0.3, -0.3),
            new Translation2d(-0.3, 0.3),
            new Translation2d(-0.3, -0.3));
    assertEquals(
        new ChassisSpeeds(), Swerve.appliedFieldRelativeSpeeds(kinematics, null, Rotation2d.kZero));
    assertEquals(
        new ChassisSpeeds(),
        Swerve.appliedFieldRelativeSpeeds(kinematics, new SwerveModuleState[0], Rotation2d.kZero));
  }

  @Test
  void recoversActualFieldCommandAtAnyRobotHeadingAndCenterOfRotation() {
    var kinematics =
        new SwerveDriveKinematics(
            new Translation2d(0.3, 0.3),
            new Translation2d(0.3, -0.3),
            new Translation2d(-0.3, 0.3),
            new Translation2d(-0.3, -0.3));
    for (double headingDegrees : new double[] {0.0, 90.0, 180.0, -135.0}) {
      var heading = Rotation2d.fromDegrees(headingDegrees);
      for (double direction : new double[] {-1.0, 1.0}) {
        var field = new ChassisSpeeds(direction * 2.0, direction * 0.5, 1.0);
        var robot = ChassisSpeeds.fromFieldRelativeSpeeds(field, heading);
        var targets = kinematics.toSwerveModuleStates(robot);
        var recovered = Swerve.appliedFieldRelativeSpeeds(kinematics, targets, heading);
        assertEquals(field.vxMetersPerSecond, recovered.vxMetersPerSecond, 1e-9);
        assertEquals(field.vyMetersPerSecond, recovered.vyMetersPerSecond, 1e-9);
        assertEquals(field.omegaRadiansPerSecond, recovered.omegaRadiansPerSecond, 1e-9);
      }
      var center = new Translation2d(0.2, -0.1);
      var targets = kinematics.toSwerveModuleStates(new ChassisSpeeds(0.0, 0.0, 1.0), center);
      var recovered = Swerve.appliedFieldRelativeSpeeds(kinematics, targets, heading);
      // v at robot center = -omega cross center-of-rotation offset.
      var expected = new Translation2d(center.getY(), -center.getX()).rotateBy(heading);
      assertEquals(expected.getX(), recovered.vxMetersPerSecond, 1e-9);
      assertEquals(expected.getY(), recovered.vyMetersPerSecond, 1e-9);
    }
  }

  @Test
  void rotationAndCoastingRetainMotionCompensation() {
    assertTrue(Swerve.isMovingForSotm(new ChassisSpeeds(0.0, 0.0, 1.0), new ChassisSpeeds()));
    assertTrue(Swerve.isMovingForSotm(new ChassisSpeeds(), new ChassisSpeeds(1.0, 0.0, 0.0)));
    assertTrue(Swerve.isMovingForSotm(new ChassisSpeeds(), new ChassisSpeeds(0.0, 0.0, 1.0)));
    assertFalse(Swerve.isMovingForSotm(new ChassisSpeeds(), new ChassisSpeeds(0.01, 0.01, 0.01)));
  }
}
