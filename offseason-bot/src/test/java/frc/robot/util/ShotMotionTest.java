package frc.robot.util;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

final class ShotMotionTest {
  @BeforeAll
  static void initializeHal() {
    HAL.initialize(500, 0);
  }

  @Test
  void feedforwardRemainsFiniteAcrossAngleWrap() {
    var pose = new Pose2d(8, 4, Rotation2d.fromDegrees(179.9));
    var result =
        AimParameterUtil.getPredictiveScoringParameters(
            pose, new ChassisSpeeds(0, 0, 1), new ChassisSpeeds(0, 0, 1), 0.1);
    assertThat(Double.isFinite(result.turretFeedForwardRadians())).isTrue();
    assertThat(Math.abs(result.turretFeedForwardRadians())).isLessThan(10);
  }

  @Test
  void rotationDoesNotAddFutureChassisYawToTheCurrentTurretCommand() {
    double maximumError = 0;
    for (double omega : new double[] {-3, -1, 1, 3}) {
      for (int step = 0; step < 100; step++) {
        var pose = new Pose2d(3, 4, Rotation2d.fromRadians(omega * step * 0.02));
        var measured = new ChassisSpeeds(0, 0, omega);
        var release = ShotMotion.predict(pose, measured, measured, 0.1);
        var releaseShot = AimParameterUtil.getScoringParameters(release.pose(), release.speeds());
        var command =
            AimParameterUtil.getPredictiveScoringParameters(pose, measured, measured, 0.1);
        var desiredFieldAngle =
            Rotation2d.fromDegrees(releaseShot.turretAngle()).plus(release.pose().getRotation());
        var commandedFieldAngle =
            Rotation2d.fromDegrees(command.turretAngle()).plus(pose.getRotation());
        maximumError =
            Math.max(
                maximumError, Math.abs(commandedFieldAngle.minus(desiredFieldAngle).getDegrees()));
      }
    }
    System.out.println("Maximum extra yaw compensation (degrees): " + maximumError);
    assertThat(maximumError).isLessThan(1e-6);
  }

  @Test
  void shotRespondsToTranslationAndRotationBeforeMeasuredMotionChanges() {
    var pose = new Pose2d(3, 4, Rotation2d.kZero);
    var measured = new ChassisSpeeds();
    var stationary = AimParameterUtil.getPredictiveScoringParameters(pose, measured, measured, 0.1);
    var rotating =
        AimParameterUtil.getPredictiveScoringParameters(
            pose, measured, new ChassisSpeeds(0, 0, 3), 0.1);
    var translating =
        AimParameterUtil.getPredictiveScoringParameters(
            pose, measured, new ChassisSpeeds(3, 1, 0), 0.1);
    assertThat(rotating.turretAngle()).isNotEqualTo(stationary.turretAngle());
    assertThat(Double.isFinite(rotating.turretFeedForwardRadians())).isTrue();
    assertThat(translating.distance()).isNotEqualTo(stationary.distance());
    assertThat(translating.turretAngle()).isNotEqualTo(stationary.turretAngle());
  }

  @Test
  void startAnticipatesInputWithoutAssumingInstantaneousDriveResponse() {
    var result =
        ShotMotion.predict(Pose2d.kZero, new ChassisSpeeds(), new ChassisSpeeds(4, 3, 6), 0.1);
    assertThat(Math.hypot(result.speeds().vxMetersPerSecond, result.speeds().vyMetersPerSecond))
        .isBetween(0.01, 0.5000001);
    assertThat(result.speeds().omegaRadiansPerSecond).isBetween(0.01, 2.0000001);
    assertThat(result.pose().getTranslation().getNorm()).isBetween(0.0, 0.0500001);
  }

  @Test
  void steadyMotionAndZeroHorizonHaveNoInventedAcceleration() {
    var pose = new Pose2d(1, 2, Rotation2d.fromDegrees(30));
    var measured = new ChassisSpeeds(1, -2, 0.5);
    var result = ShotMotion.predict(pose, measured, measured, 0.1);
    assertEquals(1.1, result.pose().getX(), 1e-9);
    assertEquals(1.8, result.pose().getY(), 1e-9);
    assertEquals(
        pose.getRotation().getRadians() + 0.05, result.pose().getRotation().getRadians(), 1e-9);
    var zero = ShotMotion.predict(pose, measured, new ChassisSpeeds(4, 4, 4), 0);
    assertEquals(pose, zero.pose());
    assertEquals(measured.vxMetersPerSecond, zero.speeds().vxMetersPerSecond);
  }

  @Test
  void stopAndReversalRemainAnchoredToMeasuredMotion() {
    var measured = new ChassisSpeeds(3, 0, 2);
    var stop = ShotMotion.predict(Pose2d.kZero, measured, new ChassisSpeeds(), 0.1);
    var reverse = ShotMotion.predict(Pose2d.kZero, measured, new ChassisSpeeds(-3, 0, -2), 0.1);
    assertThat(stop.speeds().vxMetersPerSecond).isBetween(2.49, 3.0);
    assertThat(reverse.speeds().vxMetersPerSecond).isBetween(2.49, 3.0);
    assertThat(reverse.pose().getX()).isPositive();
  }
}
