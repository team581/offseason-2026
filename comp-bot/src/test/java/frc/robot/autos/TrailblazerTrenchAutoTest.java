package frc.robot.autos;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.team581.trailblazer3.model.AutoSegment;
import com.team581.trailblazer3.model.RobotStateSnapshot;
import com.team581.trailblazer3.model.TickResult;
import com.team581.trailblazer3.runtime.Trailblazer;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.config.CompAutoField;
import frc.robot.config.CompTrailblazer;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

final class TrailblazerTrenchAutoTest {
  private enum TestMarker {
    MIDPOINT
  }

  private static TickResult stepKinematics(
      Trailblazer planner, double[] time, Pose2d[] pose, ChassisSpeeds[] speeds) {
    var result = planner.update(RobotStateSnapshot.measured(time[0], pose[0], speeds[0], true));
    var cmd = result.commandedFieldSpeeds();
    double dt = 0.02;
    pose[0] =
        new Pose2d(
            pose[0].getX() + cmd.vxMetersPerSecond * dt,
            pose[0].getY() + cmd.vyMetersPerSecond * dt,
            pose[0].getRotation().plus(Rotation2d.fromRadians(cmd.omegaRadiansPerSecond * dt)));
    speeds[0] = cmd;
    time[0] += dt;
    return result;
  }

  @Test
  void autoPreparedWhileDisabledCommandsTowardAllianceWallAfterWaiting() {
    assertTrue(HAL.initialize(500, 0));
    double[] time = {0};
    var planner = CompTrailblazer.create(() -> time[0]);
    for (var alliance : Alliance.values()) {
      var rightAuto = new TrailblazerTrenchAuto(planner, alliance);
      var rightStart = TrailblazerTrenchAuto.start(alliance);
      assertTrue(rightAuto.isNearStartPose(rightStart));
      assertFalse(rightAuto.isNearStartPose(Pose2d.kZero));

      var rightWheelTarget = rightAuto.initialDisabledWheelTarget(rightStart);
      assertTrue(rightWheelTarget.isPresent());
      double expectedDir = alliance == Alliance.Red ? 1.0 : -1.0;
      assertTrue(expectedDir * rightWheelTarget.orElseThrow().vxMetersPerSecond > 0);
      assertEquals(0.0, rightWheelTarget.orElseThrow().vyMetersPerSecond, 1e-6);

      rightAuto.periodic();
      time[0] += 30;
      var rightResult =
          planner.update(
              RobotStateSnapshot.measured(time[0], rightStart, new ChassisSpeeds(), true));
      assertEquals(TickResult.RuntimeStatus.RUNNING, rightResult.status());
      assertTrue(
          rightResult.commandedFieldSpeeds().vxMetersPerSecond * rightStart.getRotation().getCos()
              > 0);
      time[0] += 0.02;
      planner.preempt();
      planner.update(RobotStateSnapshot.measured(time[0], rightStart, new ChassisSpeeds(), false));
      time[0] += 0.02;

      var leftStart = CompAutoField.pose(CompAutoField.LEFT_TRENCH_START, alliance);
      var leftAuto =
          new TrailblazerAuto(CompAutoField.LEFT_TRENCH_START, planner, alliance) {
            {
              drive("leave_left_trench", LeftTrenchAuto.LEAVE_TRENCH_SEGMENT);
            }
          };
      assertTrue(leftAuto.isNearStartPose(leftStart));
      var leftWheelTarget = leftAuto.initialDisabledWheelTarget(leftStart);
      assertTrue(leftWheelTarget.isPresent());
      assertTrue(expectedDir * leftWheelTarget.orElseThrow().vxMetersPerSecond > 0);
      assertEquals(0.0, leftWheelTarget.orElseThrow().vyMetersPerSecond, 1e-6);

      leftAuto.periodic();
      var leftResult =
          planner.update(
              RobotStateSnapshot.measured(time[0], leftStart, new ChassisSpeeds(), true));
      assertEquals(TickResult.RuntimeStatus.RUNNING, leftResult.status());
      assertTrue(
          leftResult.commandedFieldSpeeds().vxMetersPerSecond * leftStart.getRotation().getCos()
              > 0);
      time[0] += 0.02;
      planner.preempt();
      planner.update(RobotStateSnapshot.measured(time[0], leftStart, new ChassisSpeeds(), false));
      time[0] += 0.02;
    }
  }

  @Test
  void sequentialDslRunsStepsMarkersWaitsAndNestedBranchesInOrder() {
    assertTrue(HAL.initialize(500, 0));
    double[] time = {10.0};
    var planner = CompTrailblazer.create(() -> time[0]);
    Pose2d[] pose = {CompAutoField.pose(1.0, 0.30, 0.0)};
    ChassisSpeeds[] speeds = {new ChassisSpeeds()};
    List<String> log = new ArrayList<>();
    AtomicBoolean ready = new AtomicBoolean(false);
    AtomicInteger waitLoops = new AtomicInteger(0);

    AutoSegment leg1 =
        Trailblazer.segment("leg1")
            .maxSpeed(3.0)
            .through(2.0, 0.30, g -> g.within(0.20).mark(TestMarker.MIDPOINT))
            .stopAt(2.5, 0.30, 0.0)
            .build();

    var auto =
        new TrailblazerAuto(CompAutoField.pose(1.0, 0.30, 0.0), planner, Alliance.Blue) {
          {
            action("setup", () -> log.add("setup"));
            branch(
                "outer",
                () -> true,
                () -> {
                  action("outer_true", () -> log.add("outer_true"));
                  branch(
                      "inner",
                      () -> false,
                      () -> log.add("inner_true"),
                      () -> {
                        drive("drive_leg1", leg1)
                            .withoutClearance()
                            .onStart(() -> log.add("drive_start"))
                            .onMarker(TestMarker.MIDPOINT, () -> log.add("midpoint"));
                        waitFor("wait_ready", 1.0, ready::get)
                            .onStart(() -> {
                              // Even if ready becomes true inside onStart, periodic() must wait
                              // at least one full tick before exiting the WaitStep.
                              ready.set(true);
                              log.add("wait_start");
                            })
                            .everyLoop(waitLoops::incrementAndGet);
                        waitSeconds("short_pause", 0.04);
                      });
                },
                () -> log.add("outer_false"));
            action("done", () -> log.add("done"));
          }
        };

    // Tick 0 enters setup -> outer -> inner -> drive_leg1 and stays on drive_leg1.
    auto.periodic();
    assertEquals(List.of("setup", "outer_true", "drive_start"), log);
    assertEquals("drive_leg1", auto.currentStepName());
    stepKinematics(planner, time, pose, speeds);
    assertFalse(planner.isFaulted(), planner.lastResult().diagnostic());

    int steps = 0;
    while (!"wait_ready".equals(auto.currentStepName()) && steps++ < 250) {
      auto.periodic();
      if (!"wait_ready".equals(auto.currentStepName())) {
        stepKinematics(planner, time, pose, speeds);
        assertFalse(planner.isFaulted(), planner.lastResult().diagnostic());
      }
    }
    // WaitStep just entered on this tick with ready=true set in onStart; it must NOT finish yet.
    assertEquals("wait_ready", auto.currentStepName());
    assertEquals(1, waitLoops.get());
    assertTrue(log.contains("midpoint"));
    stepKinematics(planner, time, pose, speeds);

    // Next tick sees ready == true and chains immediately into short_pause.
    auto.periodic();
    assertEquals("short_pause", auto.currentStepName());
    stepKinematics(planner, time, pose, speeds);

    for (int i = 0; i < 6 && !auto.isFinished(); i++) {
      auto.periodic();
      stepKinematics(planner, time, pose, speeds);
    }
    assertTrue(auto.isFinished());
    assertEquals("DONE", auto.currentStepName());
    assertEquals(
        List.of("setup", "outer_true", "drive_start", "midpoint", "wait_start", "done"), log);
    assertTrue(auto.isNearStartPose(CompAutoField.pose(1.1, 0.30, 5.0)));
    assertFalse(auto.isNearStartPose(Pose2d.kZero));
  }
}
