package frc.robot.autos;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.team581.trailblazer3.model.RobotStateSnapshot;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.config.CompTrailblazer;
import org.junit.jupiter.api.Test;

final class TrailblazerTrenchAutoTest {
  @Test
  void freshAutoActivatesAfterDisableAndCommandsTowardAllianceWall() {
    assertTrue(HAL.initialize(500, 0));
    double[] time = {0};
    var planner = CompTrailblazer.create(() -> time[0]);
    for (var alliance : Alliance.values()) {
      var auto = new TrailblazerTrenchAuto(planner, alliance);
      var start = TrailblazerTrenchAuto.start(alliance);
      assertTrue(auto.acceptsStart(start));
      assertFalse(auto.acceptsStart(Pose2d.kZero));
      auto.periodic();
      var result =
          planner.update(RobotStateSnapshot.measured(time[0], start, new ChassisSpeeds(), true));
      assertTrue(
          result.commandedFieldSpeeds().vxMetersPerSecond * start.getRotation().getCos() > 0);
      time[0] += 0.02;
      planner.update(RobotStateSnapshot.measured(time[0], start, new ChassisSpeeds(), false));
      time[0] += 0.02;
    }
  }
}
