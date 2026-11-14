package com.team581.trailblazer3;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.team581.trailblazer3.math.PathBuilder;
import com.team581.trailblazer3.model.EmpiricalDriveCalibration;
import com.team581.trailblazer3.model.FieldClearance;
import com.team581.trailblazer3.model.FieldGeometry;
import com.team581.trailblazer3.model.FieldSymmetry;
import com.team581.trailblazer3.model.Goal;
import com.team581.trailblazer3.model.HeadingPlan;
import com.team581.trailblazer3.model.MotionLimits;
import com.team581.trailblazer3.model.RobotDrivetrainSpec;
import com.team581.trailblazer3.model.RobotStateSnapshot;
import com.team581.trailblazer3.model.TargetLossPolicy;
import com.team581.trailblazer3.model.TickInput;
import com.team581.trailblazer3.model.TickResult.RuntimeStatus;
import com.team581.trailblazer3.model.TimestampedTarget;
import com.team581.trailblazer3.runtime.Trailblazer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Random;
import org.junit.jupiter.api.Test;

final class TrailblazerTest {
  private enum Marker {
    INTERMEDIATE,
    FINISH
  }

  private static final class Simulation {
    double time;
    final double dt;
    boolean enabled = true;
    boolean reset;
    Pose2d pose = Pose2d.kZero;
    ChassisSpeeds speed = new ChassisSpeeds();
    Optional<Translation2d> driver = Optional.empty();
    final Trailblazer planner;

    Simulation(double dt) {
      this.dt = dt;
      var robot =
          new RobotDrivetrainSpec(
              45,
              4,
              0.047,
              6.2,
              12,
              List.of(
                  new Translation2d(0.31, 0.24),
                  new Translation2d(0.31, -0.24),
                  new Translation2d(-0.31, 0.24),
                  new Translation2d(-0.31, -0.24)),
              DCMotor.getKrakenX60Foc(1),
              DCMotor.getKrakenX44Foc(1),
              80,
              0.20);
      planner = new Trailblazer(() -> time, robot, new EmpiricalDriveCalibration(5, 3, 0.040), dt);
    }

    void step() {
      var previousSpeed = speed;
      var result =
          planner.update(
              new TickInput(
                  time,
                  new RobotStateSnapshot(
                      time,
                      pose,
                      speed,
                      true,
                      reset,
                      Optional.empty(),
                      java.util.OptionalDouble.empty(),
                      java.util.OptionalDouble.empty(),
                      enabled),
                  Map.of(),
                  Map.of(),
                  driver));
      assertFalse(result.status() == RuntimeStatus.FAULT, result.diagnostic());
      reset = false;
      speed = result.commandedFieldSpeeds();
      if (enabled && result.status() == RuntimeStatus.RUNNING) {
        assertTrue(
            Math.hypot(
                    speed.vxMetersPerSecond - previousSpeed.vxMetersPerSecond,
                    speed.vyMetersPerSecond - previousSpeed.vyMetersPerSecond)
                <= 5 * dt + 1e-8);
      }
      pose =
          new Pose2d(
              pose.getX() + speed.vxMetersPerSecond * dt,
              pose.getY() + speed.vyMetersPerSecond * dt,
              pose.getRotation().plus(Rotation2d.fromRadians(speed.omegaRadiansPerSecond * dt)));
      time += dt;
    }

    void untilGoal() {
      double deadline = time + 8;
      while (!planner.atGoal() && time < deadline) {
        step();
      }
      assertTrue(
          planner.atGoal(), () -> "Planner missed stop at " + pose + ": " + planner.lastResult());
    }
  }

  private static Pose2d pose(double x, double y, double degrees) {
    return new Pose2d(x, y, Rotation2d.fromDegrees(degrees));
  }

  @Test
  void adjacentFilletsFitToleranceAndShareTangent() {
    var goals =
        List.of(
            Goal.through(new Translation2d(1, 0)).within(0.30),
            Goal.through(new Translation2d(1, 0.20)).within(0.30),
            Goal.stop(pose(2, 0.20, 0)));
    var path = PathBuilder.build(Translation2d.kZero, goals, 3);
    assertEquals(5, path.pieces().size());
    for (int i = 0; i < 2; i++) {
      var anchor = path.sample(path.anchors().get(i));
      assertTrue(anchor.position().getDistance(goals.get(i).pose().getTranslation()) <= 0.25);
      assertEquals(
          path.anchors().get(i),
          path.project(anchor.position(), 0, path.length()).progress(),
          1e-9);
    }
    for (int i = 1; i < path.pieces().size(); i++) {
      var before = path.pieces().get(i - 1);
      var after = path.pieces().get(i);
      assertEquals(
          0,
          before.sample(before.end()).tangent().getDistance(after.sample(after.start()).tangent()),
          1e-8);
    }
  }

  @Test
  void cruiseThroughKeepsMovingAfterMarkerAndStopsOnOverrun() {
    var sim = new Simulation(0.020);
    sim.planner.activate(
        Trailblazer.segment("cruise")
            .addGoal(Goal.cruiseThrough(new Translation2d(1.5, 0), 0.8, 1.5).marker(Marker.FINISH))
            .build());
    sim.untilGoal();
    assertTrue(sim.planner.passedMarker(Marker.FINISH));
    assertTrue(sim.speed.vxMetersPerSecond > 0.60);
    while (sim.time < 7) {
      sim.step();
    }
    assertEquals(3, sim.pose.getX(), 0.03);
    assertEquals(0, sim.speed.vxMetersPerSecond, 0.03);
  }

  @Test
  void dynamicRebuildAccumulatesDriftAgainstOriginalSnapshot() {
    var sim = new Simulation(0.020);
    double[] targetX = {1};
    int[] calls = {0};
    sim.planner.activate(
        Trailblazer.segment("drift")
            .addGoal(
                Goal.stop(
                    () -> {
                      calls[0]++;
                      return new TimestampedTarget<>(pose(targetX[0], 0, 0), sim.time, true);
                    },
                    TargetLossPolicy.controlledStop()))
            .build());
    sim.step();
    long firstGeneration = sim.planner.lastResult().geometryGeneration();
    targetX[0] += 0.006;
    sim.step();
    assertEquals(firstGeneration, sim.planner.lastResult().geometryGeneration());
    targetX[0] += 0.006;
    sim.step();
    assertEquals(firstGeneration + 1, sim.planner.lastResult().geometryGeneration());
    assertEquals(3, calls[0]);
  }

  @Test
  void fieldClearanceRejectsFootprintOverlap() {
    var field = new FieldGeometry(12, 7, FieldSymmetry.ROTATIONAL_180);
    var keepout =
        List.of(
            new Translation2d(4, 3),
            new Translation2d(5, 3),
            new Translation2d(5, 4),
            new Translation2d(4, 4));
    var clearance = new FieldClearance(field, 0.40, 0.35, 0.06, List.of(keepout));
    assertTrue(clearance.clearance(pose(3, 3.5, 0)) > 0.06);
    assertTrue(clearance.clearance(pose(3.7, 3.5, 0)) < 0);
  }

  @Test
  void hardGateBlocksMarkerUntilHeadingReady() {
    var sim = new Simulation(0.020);
    var hold = HeadingPlan.to(Rotation2d.k180deg).gate(HeadingPlan.GatePolicy.HARD);
    sim.planner.activate(
        Trailblazer.segment("gate")
            .addGoal(
                Goal.through(new Translation2d(0.50, 0)).heading(hold).marker(Marker.INTERMEDIATE))
            .addGoal(Goal.stop(pose(1.50, 0, 180)))
            .build());
    sim.step();
    assertFalse(sim.planner.passedMarker(Marker.INTERMEDIATE));
    sim.untilGoal();
    assertTrue(sim.planner.passedMarker(Marker.INTERMEDIATE));
  }

  @Test
  void launchesAtBothEndsOfSupportedLoopRangeAndDisableRequiresReactivation() {
    for (double dt : new double[] {0.004, 0.020}) {
      var sim = new Simulation(dt);
      sim.planner.activate(
          Trailblazer.segment("straight").addGoal(Goal.stop(pose(0.6, 0, 0))).build());
      sim.step();
      assertTrue(sim.speed.vxMetersPerSecond > 0);
      sim.enabled = false;
      sim.step();
      assertEquals(RuntimeStatus.DISABLED, sim.planner.lastResult().status());
      sim.enabled = true;
      sim.step();
      assertEquals(RuntimeStatus.DISABLED, sim.planner.lastResult().status());
    }
  }

  @Test
  void rejectsExactReversalWithoutStop() {
    assertThrows(
        IllegalArgumentException.class,
        () ->
            PathBuilder.build(
                Translation2d.kZero,
                List.of(Goal.through(new Translation2d(1, 0)), Goal.stop(Pose2d.kZero)),
                3));
  }

  @Test
  void resumesAfterMissedTicksWithFreshOdometryAndStillRejectsStaleOdometry() {
    var sim = new Simulation(0.020);
    sim.planner.activate(Trailblazer.segment("straight").stopAt(pose(2, 0, 0)).build());
    sim.step();
    long generation = sim.planner.lastResult().geometryGeneration();
    for (double delay : new double[] {0.123, 0.177, 0.126}) {
      sim.time += delay - sim.dt;
      sim.speed = new ChassisSpeeds();
      sim.step();
      var result = sim.planner.lastResult();
      assertEquals(RuntimeStatus.RUNNING, result.status());
      assertEquals("MISSED_TICK_RECOVERY", result.diagnostic());
      assertEquals(generation, result.geometryGeneration());
      assertTrue(sim.speed.vxMetersPerSecond > 0);
      assertTrue(sim.speed.vxMetersPerSecond <= 5 * sim.dt + 1e-9);
      sim.step();
    }
    sim.time += 0.2;
    var stale =
        sim.planner.update(RobotStateSnapshot.measured(sim.time - 0.15, sim.pose, sim.speed, true));
    assertEquals(RuntimeStatus.FAULT, stale.status());
    assertEquals(0, stale.commandedFieldSpeeds().vxMetersPerSecond);
  }

  @Test
  void rotationSnapLeavesTranslationWithDriverAndRespectsPreemption() {
    var sim = new Simulation(0.020);
    sim.planner.snapAngle(Rotation2d.kCCW_90deg, MotionLimits.robotMaximum());
    assertEquals(Trailblazer.Ownership.ROTATION, sim.planner.ownership());
    assertFalse(sim.planner.preempt(true, false));
    sim.driver = Optional.of(new Translation2d(0.65, 0));
    sim.untilGoal();
    assertEquals(Trailblazer.Ownership.NONE, sim.planner.ownership());
    assertTrue(sim.pose.getX() > 0.25);
    sim.planner.driveTo(pose(1, 1, 90), MotionLimits.robotMaximum());
    assertTrue(sim.planner.preempt(true, false));
    assertEquals(Trailblazer.Ownership.NONE, sim.planner.ownership());
  }

  @Test
  void roundsCornerAndReleasesIntermediateStop() {
    var sim = new Simulation(0.020);
    sim.planner.activate(
        Trailblazer.segment("corner")
            .addGoal(Goal.through(new Translation2d(1.0, 0)).within(0.30))
            .addGoal(Goal.stop(pose(1.0, 1.0, 0)).marker(Marker.INTERMEDIATE))
            .addGoal(Goal.stop(pose(2.0, 1.0, 0)).marker(Marker.FINISH))
            .build());
    sim.untilGoal();
    assertTrue(sim.planner.passedMarker(Marker.INTERMEDIATE));
    assertFalse(sim.planner.passedMarker(Marker.FINISH));
    assertTrue(sim.planner.release());
    sim.untilGoal();
    assertTrue(sim.planner.passedMarker(Marker.FINISH));
  }

  @Test
  void seededStartOffsetsAndLocalizationResetRecoverOnSyntheticRobot() {
    var random = new Random(581);
    for (int sample = 0; sample < 20; sample++) {
      var sim = new Simulation(0.004 + random.nextDouble() * 0.016);
      sim.pose =
          pose(
              random.nextDouble() * 0.16 - 0.08,
              random.nextDouble() * 0.16 - 0.08,
              random.nextDouble() * 10 - 5);
      sim.planner.activate(
          Trailblazer.segment("perturbation")
              .addGoal(Goal.stop(pose(4, 0, 0)).marker(Marker.FINISH))
              .build());
      while (sim.time < 0.8) {
        sim.step();
      }
      sim.pose = pose(sim.pose.getX(), sim.pose.getY() + 0.25, sim.pose.getRotation().getDegrees());
      sim.reset = true;
      sim.step();
      assertEquals(2, sim.planner.lastResult().geometryGeneration());
      sim.untilGoal();
      assertTrue(sim.planner.passedMarker(Marker.FINISH));
    }
  }

  @Test
  void turnsInPlaceAndMirrorsDirectedHeading() {
    var sim = new Simulation(0.020);
    var keyframe = HeadingPlan.to(Rotation2d.kCCW_90deg).direction(HeadingPlan.TurnDirection.CW);
    var plan =
        Trailblazer.segment("turn").addGoal(Goal.stop(pose(0, 0, 90)).heading(keyframe)).build();
    var reflected =
        plan.forAlliance(Alliance.Red, new FieldGeometry(12, 7, FieldSymmetry.MIRROR_X));
    assertEquals(HeadingPlan.TurnDirection.CCW, reflected.goals().get(0).heading().direction());
    sim.planner.activate(plan);
    sim.step();
    assertTrue(sim.speed.omegaRadiansPerSecond < 0);
    sim.untilGoal();
    assertEquals(90, sim.pose.getRotation().getDegrees(), 2);
  }
}
