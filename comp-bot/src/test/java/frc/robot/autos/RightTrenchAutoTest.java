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
  private static AutoSegment intakeSegment(RightTrenchAuto.State state) {
    return switch (state) {
      case INTAKE_CYCLE_1 -> RightTrenchAuto.CYCLE_1_SEGMENT;
      case INTAKE_CYCLE_2 -> RightTrenchAuto.CYCLE_2_SEGMENT;
      case INTAKE_CYCLE_3 -> RightTrenchAuto.CYCLE_3_SEGMENT;
      default -> throw new IllegalArgumentException("Unexpected state " + state);
    };
  }

  @Test
  void allSegmentsBuildWithOnlyTheArchivedCycleTwoSwitchbackWarning() {
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
            i == 1 ? ImmutableList.of("NEAR_REVERSAL_CORNER goal-10") : ImmutableList.of(),
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

      var launchGoals = CompAutoField.segment(RightTrenchAuto.CYCLE_1_SEGMENT, alliance).goals();
      double launchAnchor =
          PathBuilder.build(
                  start.getTranslation(),
                  launchGoals,
                  CompTrailblazer.CALIBRATION.maxBrakingDeceleration(),
                  clearance)
              .anchors()
              .get(0);
      var returnGoals = CompAutoField.segment(RightTrenchAuto.CYCLE_2_SEGMENT, alliance).goals();
      var trenchHeading = returnGoals.get(1).pose().getRotation();
      double trenchStartX = returnGoals.get(1).pose().getX();
      double trenchEndX = returnGoals.get(3).pose().getX();
      double trenchY = returnGoals.get(1).pose().getY();
      double trenchMinX = Math.min(trenchStartX, trenchEndX);
      double trenchMaxX = Math.max(trenchStartX, trenchEndX);
      var trenchHeadingChecked = new LinkedHashSet<RightTrenchAuto.State>();

      Pose2d pose = start;
      ChassisSpeeds speeds = new ChassisSpeeds();
      double previousCommandOmega = 0;
      var statesVisited = new LinkedHashSet<RightTrenchAuto.State>();
      var maxProgressByState = new LinkedHashMap<RightTrenchAuto.State, Double>();
      var bumpHeadingsReached = new LinkedHashSet<RightTrenchAuto.State>();
      PathBuilder.Path referencePath = null;
      RightTrenchAuto.State referenceState = null;
      double peakCurveError = 0;
      double fourthHeadingError = Double.NaN;
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
        boolean intaking =
            state == RightTrenchAuto.State.INTAKE_CYCLE_1
                || state == RightTrenchAuto.State.INTAKE_CYCLE_2
                || state == RightTrenchAuto.State.INTAKE_CYCLE_3;
        if (intaking) {
          driveSeconds += dt;
          if (state != referenceState) {
            referenceState = state;
            referencePath =
                PathBuilder.build(
                    pose.getTranslation(),
                    CompAutoField.segment(intakeSegment(state), alliance).goals(),
                    CompTrailblazer.CALIBRATION.maxBrakingDeceleration(),
                    clearance);
          }
        }
        var result = planner.update(time[0], pose, speeds);
        maxProgressByState.merge(state, result.progress(), Math::max);
        assertNotEquals(
            "KEEPOUT_VIOLATION", result.diagnostic(), alliance + " keepout violation at " + state);
        assertNotEquals(
            "NONFINITE_STATE", result.diagnostic(), alliance + " nonfinite state at " + state);
        assertNotEquals("RELATCH", result.diagnostic(), alliance + " relatch at " + state);
        assertNotEquals("HEADING_SCHEDULE_INFEASIBLE", result.diagnostic(), alliance + " " + state);
        assertThat(result.infeasible())
            .withFailMessage("%s %s: %s", alliance, state, result)
            .isFalse();
        if (intaking) {
          peakCurveError =
              Math.max(
                  peakCurveError,
                  referencePath
                      .sample(result.progress())
                      .position()
                      .getDistance(pose.getTranslation()));
          var segment = intakeSegment(state);
          if (!bumpHeadingsReached.contains(state)
              && planner.headingArrivals().stream()
                  .anyMatch(arrival -> arrival.goalIndex() == segment.goals().size() - 2)) {
            long expected =
                segment.goals().stream()
                    .filter(goal -> goal.heading() != null && goal.heading().explicitTolerance())
                    .count();
            assertThat(planner.headingArrivals())
                .hasSize((int) expected)
                .withFailMessage(
                    "%s %s heading arrivals: %s", alliance, state, planner.headingArrivals())
                .allMatch(arrival -> arrival.withinTolerance());
            if (state == RightTrenchAuto.State.INTAKE_CYCLE_1) {
              fourthHeadingError =
                  Math.abs(Math.toDegrees(planner.headingArrivals().get(3).errorRadians()));
            }
            bumpHeadingsReached.add(state);
          }
        }
        if (state == RightTrenchAuto.State.INTAKE_CYCLE_1 && result.progress() < launchAnchor) {
          assertThat(Math.abs(pose.getRotation().minus(start.getRotation()).getDegrees()))
              .withFailMessage("%s launch heading at %.2f m", alliance, result.progress())
              .isLessThanOrEqualTo(1.5);
        }
        if ((state == RightTrenchAuto.State.INTAKE_CYCLE_2
                || state == RightTrenchAuto.State.INTAKE_CYCLE_3)
            && pose.getX() > trenchMinX + 0.1
            && pose.getX() < trenchMaxX - 0.1
            && Math.abs(pose.getY() - trenchY) < 0.2) {
          assertThat(Math.abs(pose.getRotation().minus(trenchHeading).getDegrees()))
              .withFailMessage("%s %s heading in trench at %s", alliance, state, pose)
              .isLessThanOrEqualTo(5.0);
          if (Math.abs(pose.getX() - (trenchMinX + trenchMaxX) / 2) < 0.10) {
            trenchHeadingChecked.add(state);
          }
        }
        var command = result.commandedFieldSpeeds();
        assertThat(Math.abs(command.omegaRadiansPerSecond - previousCommandOmega))
            .withFailMessage("%s %s angular command jumped at %.2f s", alliance, state, time[0])
            .isLessThanOrEqualTo(Math.toRadians(320) * dt + 1e-6);
        previousCommandOmega = command.omegaRadiansPerSecond;
        // Stress yaw at twice the configured lag during the first intake.
        double translationResponse =
            state == RightTrenchAuto.State.INTAKE_CYCLE_1
                ? -Math.expm1(-dt / CompTrailblazer.CALIBRATION.driveResponseLagSeconds())
                : 1;
        double angularResponse =
            state == RightTrenchAuto.State.INTAKE_CYCLE_1
                ? -Math.expm1(-dt / (2 * CompTrailblazer.CALIBRATION.driveResponseLagSeconds()))
                : 1.0;
        speeds =
            new ChassisSpeeds(
                speeds.vxMetersPerSecond
                    + translationResponse * (command.vxMetersPerSecond - speeds.vxMetersPerSecond),
                speeds.vyMetersPerSecond
                    + translationResponse * (command.vyMetersPerSecond - speeds.vyMetersPerSecond),
                speeds.omegaRadiansPerSecond
                    + angularResponse
                        * (command.omegaRadiansPerSecond - speeds.omegaRadiansPerSecond));
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
      assertThat(trenchHeadingChecked)
          .containsExactly(
              RightTrenchAuto.State.INTAKE_CYCLE_2, RightTrenchAuto.State.INTAKE_CYCLE_3);
      assertThat(bumpHeadingsReached)
          .contains(
              RightTrenchAuto.State.INTAKE_CYCLE_1,
              RightTrenchAuto.State.INTAKE_CYCLE_2,
              RightTrenchAuto.State.INTAKE_CYCLE_3);
      assertEquals(RobotState.IDLE, context.robot().getState());
      assertThat(peakCurveError)
          .withFailMessage(
              "%s maximum deviation from the original curve: %.3f m", alliance, peakCurveError)
          .isLessThan(0.08);
      System.out.printf(
          "%s drive time %.2f s, peak speed %.2f m/s, peak curve error %.3f m, waypoint 4 error %.2f deg%n",
          alliance, driveSeconds, peakSpeedMps, peakCurveError, fourthHeadingError);
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

      // Budget for the archived intake sweeps with all their authored heading bands enforced.
      assertThat(driveSeconds < 24.0)
          .withFailMessage("%s pure drive time too slow: %s s", alliance, driveSeconds)
          .isTrue();
    }
  }
}
