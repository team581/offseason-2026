package frc.robot.autos;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import com.google.common.collect.ImmutableList;
import com.team581.math.MathHelpers;
import com.team581.trailblazer3.math.PathBuilder;
import com.team581.trailblazer3.model.AutoSegment;
import com.team581.trailblazer3.model.TickResult.RuntimeStatus;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.config.CompAutoField;
import frc.robot.config.CompTrailblazer;
import frc.robot.robot_manager.RobotState;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import org.junit.jupiter.api.Test;

final class RightTrenchAutoTest {
  @Test
  void allSegmentsBuildCleanlyWithoutPathWarnings() {
    assertThat(HAL.initialize(500, 0)).isTrue();
    for (var alliance : Alliance.values()) {
      var clearance = CompAutoField.clearance(alliance);
      var start = CompAutoField.pose(RightTrenchAuto.START, alliance);
      var shoot = RightTrenchAuto.shootPose(alliance);
      var cycleStarts = ImmutableList.of(start, shoot, shoot);
      var segments =
          ImmutableList.of(
              RightTrenchAuto.CYCLE_1_SEGMENT,
              RightTrenchAuto.CYCLE_2_SEGMENT,
              RightTrenchAuto.CYCLE_3_SEGMENT);
      for (int i = 0; i < segments.size(); i++) {
        AutoSegment segment = CompAutoField.segment(segments.get(i), alliance);
        var built =
            PathBuilder.build(
                cycleStarts.get(i).getTranslation(),
                segment.goals(),
                CompTrailblazer.CALIBRATION.maxBrakingDeceleration(),
                clearance);
        assertEquals(
            ImmutableList.of(),
            built.warnings(),
            alliance + " cycle " + (i + 1) + " warnings: " + built.warnings());
        assertThat(built.length() > 1.0)
            .withFailMessage("%s cycle %d length", alliance, i + 1)
            .isTrue();
      }
    }
  }

  @Test
  void drivesFullThreeCycleSequenceCleanlyOnBothAlliances() {
    assertThat(HAL.initialize(500, 0)).isTrue();
    double dt = 0.020;
    for (var alliance : Alliance.values()) {
      double[] time = {0.0};
      var planner = CompTrailblazer.create(() -> time[0]);
      var context = TrailblazerTrenchAutoTest.testContext(planner, alliance);
      var auto = new RightTrenchAuto(context);
      var start = auto.startPose().orElseThrow();
      var expectedShoot = RightTrenchAuto.shootPose(alliance);
      var clearance = CompAutoField.clearance(alliance);
      assertThat(auto.previewPath(start).length > 50)
          .withFailMessage("%s preview path too sparse", alliance)
          .isTrue();

      Pose2d pose = start;
      ChassisSpeeds speeds = new ChassisSpeeds();
      var statesVisited = new LinkedHashSet<RightTrenchAuto.State>();
      var maxProgressByState = new LinkedHashMap<RightTrenchAuto.State, Double>();
      int infeasibleTicks = 0;
      double peakSpeedMps = 0.0;
      double driveSeconds = 0.0;

      for (; time[0] < 35.0 && auto.getState() != RightTrenchAuto.State.DONE; time[0] += dt) {
        statesVisited.add(auto.getState());
        auto.periodic();
        var state = auto.getState();
        statesVisited.add(state);
        if (state == RightTrenchAuto.State.DONE) {
          break;
        }
        if (state == RightTrenchAuto.State.INTAKE_CYCLE_1
            || state == RightTrenchAuto.State.INTAKE_CYCLE_2
            || state == RightTrenchAuto.State.INTAKE_CYCLE_3) {
          driveSeconds += dt;
        }
        var result = planner.update(time[0], pose, speeds);
        maxProgressByState.merge(state, result.progress(), Math::max);
        assertNotEquals(
            "KEEPOUT_VIOLATION", result.diagnostic(), alliance + " keepout violation at " + state);
        assertNotEquals(
            "NONFINITE_STATE", result.diagnostic(), alliance + " nonfinite state at " + state);
        if (result.infeasible()) {
          infeasibleTicks++;
        }
        speeds = result.commandedFieldSpeeds();
        peakSpeedMps = Math.max(peakSpeedMps, MathHelpers.getLinearVelocity(speeds));
        pose =
            new Pose2d(
                pose.getX() + speeds.vxMetersPerSecond * dt,
                pose.getY() + speeds.vyMetersPerSecond * dt,
                pose.getRotation().plus(Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * dt)));
        if (result.status() == RuntimeStatus.RUNNING) {
          assertThat(clearance.clearance(pose) > 0.0)
              .withFailMessage("%s keepout violation at %s during %s", alliance, pose, state)
              .isTrue();
        }
        if (state == RightTrenchAuto.State.SHOOT_1) {
          assertEquals(RobotState.PREPARE_SCORE, context.robot().getState());
          assertThat(pose.getTranslation().getDistance(expectedShoot.getTranslation()) < 0.06)
              .withFailMessage("%s cycle 1 shoot pose error at %s", alliance, pose)
              .isTrue();
          assertEquals(
              0.0,
              Math.abs(pose.getRotation().minus(expectedShoot.getRotation()).getDegrees()),
              3.0,
              alliance + " cycle 1 hub aim heading error");
        }
      }

      assertEquals(
          RightTrenchAuto.State.DONE,
          auto.getState(),
          alliance + " unfinished in state " + auto.getState());
      assertEquals(RobotState.IDLE, context.robot().getState());
      assertEquals(0, infeasibleTicks, alliance + " had infeasible ticks");
      assertThat(peakSpeedMps > 2.5)
          .withFailMessage("%s peak speed unexpectedly slow: %s", alliance, peakSpeedMps)
          .isTrue();
      assertThat(maxProgressByState.getOrDefault(RightTrenchAuto.State.INTAKE_CYCLE_1, 0.0) > 8.0)
          .withFailMessage("%s cycle 1 did not traverse full path", alliance)
          .isTrue();
      assertThat(maxProgressByState.getOrDefault(RightTrenchAuto.State.INTAKE_CYCLE_2, 0.0) > 8.0)
          .withFailMessage("%s cycle 2 did not traverse full path", alliance)
          .isTrue();
      assertThat(maxProgressByState.getOrDefault(RightTrenchAuto.State.INTAKE_CYCLE_3, 0.0) > 8.0)
          .withFailMessage("%s cycle 3 did not traverse full path", alliance)
          .isTrue();
      assertThat(pose.getTranslation().getDistance(expectedShoot.getTranslation()) < 0.06)
          .withFailMessage("%s final pose error at %s", alliance, pose)
          .isTrue();
      assertEquals(
          0.0,
          Math.abs(pose.getRotation().minus(expectedShoot.getRotation()).getDegrees()),
          3.0,
          alliance + " final heading error");
      assertThat(statesVisited)
          .contains(
              RightTrenchAuto.State.INTAKE_CYCLE_1,
              RightTrenchAuto.State.SHOOT_1,
              RightTrenchAuto.State.INTAKE_CYCLE_2,
              RightTrenchAuto.State.SHOOT_2,
              RightTrenchAuto.State.INTAKE_CYCLE_3,
              RightTrenchAuto.State.SHOOT_3,
              RightTrenchAuto.State.DONE);
      assertThat(driveSeconds < 17.0)
          .withFailMessage("%s pure drive time too slow: %s s", alliance, driveSeconds)
          .isTrue();
    }
  }
}
