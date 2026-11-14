package frc.robot.autos;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.team581.trailblazer3.model.RobotStateSnapshot;
import com.team581.trailblazer3.model.TickResult.RuntimeStatus;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.config.CompTrailblazer;
import org.junit.jupiter.api.Test;

final class TrailblazerTrenchRotationAutoTest {
  @Test
  void turnsWhileDrivingLeftOnEitherAlliance() {
    assertTrue(HAL.initialize(500, 0));
    for (var alliance : Alliance.values()) {
      double dt = 0.020;
      double[] time = {0};
      var planner = CompTrailblazer.create(() -> time[0]);
      var auto = new TrailblazerTrenchRotationAuto(planner, alliance);
      var start = TrailblazerTrenchRotationAuto.start(alliance);
      var pose = start;
      var speeds = new ChassisSpeeds();
      double overlapRadians = 0;
      for (; time[0] < 14; time[0] += dt) {
        auto.periodic();
        var result = planner.update(RobotStateSnapshot.measured(time[0], pose, speeds, true));
        assertNotEquals(RuntimeStatus.FAULT, result.status(), result.diagnostic());
        speeds = result.commandedFieldSpeeds();
        if (Math.abs(speeds.vyMetersPerSecond) > 0.25) {
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
      assertTrue(planner.atGoal());
      assertTrue(
          overlapRadians > Math.PI / 2,
          "Expected over 90 degrees of rotation while strafing, got "
              + Math.toDegrees(overlapRadians));
      assertEquals(start.getX(), pose.getX(), 0.06);
      assertEquals(start.getY() + (alliance == Alliance.Blue ? 3 : -3), pose.getY(), 0.06);
      assertEquals(180, Math.abs(pose.getRotation().minus(start.getRotation()).getDegrees()), 3);
    }
  }
}
