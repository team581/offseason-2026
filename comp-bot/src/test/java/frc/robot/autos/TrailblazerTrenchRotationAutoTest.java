package frc.robot.autos;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import com.team581.math.MathHelpers;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.config.CompTrailblazer;
import org.junit.jupiter.api.Test;

final class TrailblazerTrenchRotationAutoTest {
  @Test
  void turnsWhileDrivingLeftOnEitherAlliance() {
    assertThat(HAL.initialize(500, 0)).isTrue();
    for (var alliance : Alliance.values()) {
      double dt = 0.020;
      double[] time = {0};
      var planner = CompTrailblazer.create(() -> time[0]);
      var auto =
          new TrailblazerTrenchRotationAuto(
              TrailblazerTrenchAutoTest.testContext(planner, alliance));
      var start = auto.startPose().orElseThrow();
      var pose = start;
      var speeds = new ChassisSpeeds();
      double overlapRadians = 0;
      double prevLinearSpeed = 0;
      double peakLinearSpeed = 0;
      for (; time[0] < 14; time[0] += dt) {
        auto.periodic();
        var result = planner.update(time[0], pose, speeds);
        assertNotEquals("KEEPOUT_VIOLATION", result.diagnostic());
        speeds = result.commandedFieldSpeeds();
        double linearSpeed = MathHelpers.getLinearVelocity(speeds);
        peakLinearSpeed = Math.max(peakLinearSpeed, linearSpeed);
        if (MathUtil.isNear(pose.getY(), start.getY(), 0.60)) {
          assertThat(linearSpeed >= prevLinearSpeed - 1e-6)
              .withFailMessage(
                  "Expected smooth monotonic launch without dip: prev="
                      + prevLinearSpeed
                      + ", curr="
                      + linearSpeed
                      + " at yDelta="
                      + Math.abs(pose.getY() - start.getY()))
              .isTrue();
        }
        prevLinearSpeed = linearSpeed;
        if (!MathUtil.isNear(0, speeds.vyMetersPerSecond, 0.25)) {
          overlapRadians += speeds.omegaRadiansPerSecond * dt;
        }
        pose =
            new Pose2d(
                pose.getX() + speeds.vxMetersPerSecond * dt,
                pose.getY() + speeds.vyMetersPerSecond * dt,
                pose.getRotation().plus(Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * dt)));
        if (overlapRadians > 0 && planner.atGoal()) {
          break;
        }
      }
      assertThat(planner.atGoal()).isTrue();
      assertThat(peakLinearSpeed > 1.5)
          .withFailMessage("Expected peak speed > 1.5 m/s, got " + peakLinearSpeed)
          .isTrue();
      assertThat(overlapRadians > Math.PI / 2)
          .withFailMessage(
              "Expected over 90 degrees of rotation while strafing, got "
                  + Math.toDegrees(overlapRadians))
          .isTrue();
      assertEquals(start.getX(), pose.getX(), 0.06);
      assertEquals(start.getY() + (alliance == Alliance.Blue ? 3 : -3), pose.getY(), 0.06);
      assertEquals(180, Math.abs(pose.getRotation().minus(start.getRotation()).getDegrees()), 3);
    }
  }
}
