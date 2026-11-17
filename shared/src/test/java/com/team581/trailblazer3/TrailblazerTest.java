package com.team581.trailblazer3;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatThrownBy;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import com.google.common.collect.ImmutableList;
import com.team581.trailblazer3.math.PathBuilder;
import com.team581.trailblazer3.model.EmpiricalDriveCalibration;
import com.team581.trailblazer3.model.FieldClearance;
import com.team581.trailblazer3.model.FieldGeometry;
import com.team581.trailblazer3.model.FieldSymmetry;
import com.team581.trailblazer3.model.Goal;
import com.team581.trailblazer3.model.HeadingPlan;
import com.team581.trailblazer3.model.MotionLimits;
import com.team581.trailblazer3.model.RobotDrivetrainSpec;
import com.team581.trailblazer3.model.TickResult.RuntimeStatus;
import com.team581.trailblazer3.runtime.Trailblazer;
import com.team581.util.FieldUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    Pose2d pose = Pose2d.kZero;
    ChassisSpeeds speed = new ChassisSpeeds();
    final double maxAccel;
    final Trailblazer planner;

    Simulation(double dt) {
      this(dt, new EmpiricalDriveCalibration(5, 3, 0.040));
    }

    Simulation(double dt, EmpiricalDriveCalibration calibration) {
      this.dt = dt;
      this.maxAccel = calibration.frictionLimit();
      var robot =
          new RobotDrivetrainSpec(
              45,
              4,
              0.047,
              6.2,
              12,
              ImmutableList.of(
                  new Translation2d(0.31, 0.24),
                  new Translation2d(0.31, -0.24),
                  new Translation2d(-0.31, 0.24),
                  new Translation2d(-0.31, -0.24)),
              DCMotor.getKrakenX60Foc(1),
              DCMotor.getKrakenX44Foc(1),
              80,
              0.20);
      planner = new Trailblazer(() -> time, robot, calibration, dt);
    }

    void step() {
      var previousSpeed = speed;
      var result = planner.update(time, pose, speed);
      assertNotEquals("KEEPOUT_VIOLATION", result.diagnostic());
      speed = result.commandedFieldSpeeds();
      if (result.status() == RuntimeStatus.RUNNING) {
        assertThat(
                Math.hypot(
                    speed.vxMetersPerSecond - previousSpeed.vxMetersPerSecond,
                    speed.vyMetersPerSecond - previousSpeed.vyMetersPerSecond))
            .isLessThanOrEqualTo(maxAccel * dt + 1e-8);
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
      assertThat(planner.atGoal())
          .withFailMessage(() -> "Planner missed stop at " + pose + ": " + planner.lastResult())
          .isTrue();
    }
  }

  private static Pose2d pose(double x, double y, double degrees) {
    return new Pose2d(x, y, Rotation2d.fromDegrees(degrees));
  }

  @Test
  void adjacentFilletsFitToleranceAndShareTangent() {
    var goals =
        ImmutableList.of(
            Goal.through(new Translation2d(1, 0)).within(0.30),
            Goal.through(new Translation2d(1, 0.20)).within(0.30),
            Goal.stop(pose(2, 0.20, 0)));
    var path = PathBuilder.build(Translation2d.kZero, goals, 3);
    assertEquals(5, path.pieces().size());
    for (int i = 0; i < 2; i++) {
      var anchor = path.sample(path.anchors().get(i));
      assertThat(anchor.position().getDistance(goals.get(i).pose().getTranslation()))
          .isLessThanOrEqualTo(0.25);
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
  void clearanceGatedTurnDelaysRotationUntilClearAndLogsKeepoutViolation() {
    var robot =
        new RobotDrivetrainSpec(
            55.0,
            4.5,
            0.048,
            6.12,
            15.0,
            ImmutableList.of(
                new Translation2d(0.28, 0.24),
                new Translation2d(0.28, -0.24),
                new Translation2d(-0.28, 0.24),
                new Translation2d(-0.28, -0.24)),
            DCMotor.getKrakenX60Foc(1),
            DCMotor.getKrakenX44Foc(1),
            80,
            0.18);
    var clearance =
        new FieldClearance(
            new FieldGeometry(16.5, 8.2, FieldSymmetry.ROTATIONAL_180),
            0.40,
            0.40,
            0.04,
            ImmutableList.of());

    double[] time = {0.0};
    var validPlanner =
        new Trailblazer(() -> time[0], robot, new EmpiricalDriveCalibration(4.5, 2.5, 0.04), 0.02);
    validPlanner.activate(
        Trailblazer.segment("auto-delayed-turn")
            .stopAt(pose(2, 3, 0), Goal::counterClockwise)
            .build(),
        clearance);
    Pose2d currentPose = pose(2.0, 0.50, 180);
    ChassisSpeeds currentSpeeds = new ChassisSpeeds();
    for (int i = 0; i < 300 && !validPlanner.atGoal(); i++) {
      var res = validPlanner.update(time[0], currentPose, currentSpeeds);
      assertNotEquals("KEEPOUT_VIOLATION", res.diagnostic());
      currentSpeeds = res.commandedFieldSpeeds();
      if (i == 0) {
        assertEquals(0.0, currentSpeeds.omegaRadiansPerSecond);
      }
      assertThat(clearance.clearance(currentPose)).isGreaterThanOrEqualTo(0.04 - 1e-3);
      currentPose =
          new Pose2d(
              currentPose.getX() + currentSpeeds.vxMetersPerSecond * 0.02,
              currentPose.getY() + currentSpeeds.vyMetersPerSecond * 0.02,
              currentPose
                  .getRotation()
                  .plus(Rotation2d.fromRadians(currentSpeeds.omegaRadiansPerSecond * 0.02)));
      time[0] += 0.02;
    }
    assertThat(validPlanner.atGoal()).isTrue();
    assertEquals(1.0, currentPose.getRotation().getCos(), 1e-2);

    // Turning parallel to the wall logs KEEPOUT_VIOLATION without stopping the auto.
    time[0] = 0.0;
    var parallelPlanner =
        new Trailblazer(() -> time[0], robot, new EmpiricalDriveCalibration(4.5, 2.5, 0.04), 0.02);
    parallelPlanner.activate(
        Trailblazer.segment("parallel-turn")
            .stopAt(pose(5.0, 0.50, 0), Goal::counterClockwise)
            .build(),
        clearance);
    Pose2d parallelPose = pose(2.0, 0.50, 180);
    ChassisSpeeds parallelSpeeds = new ChassisSpeeds();
    boolean sawKeepoutWarning = false;
    for (int i = 0; i < 40; i++) {
      var tick = parallelPlanner.update(time[0], parallelPose, parallelSpeeds);
      assertEquals(RuntimeStatus.RUNNING, tick.status());
      if ("KEEPOUT_VIOLATION".equals(tick.diagnostic())) {
        sawKeepoutWarning = true;
      }
      parallelSpeeds = tick.commandedFieldSpeeds();
      parallelPose =
          new Pose2d(
              parallelPose.getX() + parallelSpeeds.vxMetersPerSecond * 0.02,
              parallelPose.getY() + parallelSpeeds.vyMetersPerSecond * 0.02,
              parallelPose
                  .getRotation()
                  .plus(Rotation2d.fromRadians(parallelSpeeds.omegaRadiansPerSecond * 0.02)));
      time[0] += 0.02;
    }
    assertThat(sawKeepoutWarning).isTrue();
  }

  @Test
  void closedLoopSegmentStartingAndEndingAtSamePoseDrivesFullLoop() {
    var sim = new Simulation(0.020);
    Pose2d loopOrigin = pose(1.0, 1.0, 40.0);
    sim.pose = loopOrigin;
    sim.planner.activate(
        Trailblazer.segment("closed-loop")
            .through(3.0, 1.0, g -> g.faceDegrees(0))
            .through(pose(4.0, 2.5, 90))
            .mark(Marker.INTERMEDIATE)
            .through(2.0, 2.5, g -> g.faceDegrees(180))
            .stopAt(loopOrigin)
            .mark(Marker.FINISH)
            .build());

    double maxProgress = 0;
    double deadline = sim.time + 12.0;
    while (!sim.planner.atGoal() && sim.time < deadline) {
      sim.step();
      var res = sim.planner.lastResult();
      assertThat(res.infeasible())
          .withFailMessage(() -> "Unexpected infeasible at progress=" + res.progress())
          .isFalse();
      maxProgress = Math.max(maxProgress, res.progress());
    }
    assertThat(sim.planner.atGoal()).isTrue();
    assertThat(maxProgress > 4.0)
        .withFailMessage("Closed loop aborted early at maxProgress=%s", maxProgress)
        .isTrue();
    assertThat(sim.planner.passedMarker(Marker.INTERMEDIATE)).isTrue();
    assertThat(sim.planner.passedMarker(Marker.FINISH)).isTrue();
    assertEquals(1.0, sim.pose.getX(), 0.05);
    assertEquals(1.0, sim.pose.getY(), 0.05);
  }

  @Test
  void faceHubAimsRearShooterOnBothAlliancesAndSettlesAtStop() {
    var field =
        new FieldGeometry(
            FieldUtil.FIELD_LENGTH_X, FieldUtil.FIELD_WIDTH_Y, FieldSymmetry.ROTATIONAL_180);
    var blueSegment =
        Trailblazer.segment("face-hub-stop").stopAt(3.35, 2.58, Goal::faceHub).build();
    for (var alliance : Alliance.values()) {
      var sim = new Simulation(0.020);
      Pose2d start = pose(4.65, 2.58, 180.0);
      sim.pose = alliance == Alliance.Blue ? start : field.flipPose(start);
      sim.planner.activate(blueSegment.forAlliance(alliance, field));
      sim.untilGoal();
      Translation2d hub =
          alliance == Alliance.Blue
              ? FieldUtil.BLUE_HUB_TRANSLATION
              : FieldUtil.HUB_POSE.redPose().getTranslation();
      Rotation2d expectedHeading =
          hub.minus(sim.pose.getTranslation()).getAngle().plus(Rotation2d.k180deg);
      assertEquals(0.0, sim.pose.getRotation().minus(expectedHeading).getDegrees(), 2.5);
    }
  }

  @Test
  void fieldClearanceRejectsFootprintOverlap() {
    var field = new FieldGeometry(12, 7, FieldSymmetry.ROTATIONAL_180);
    var keepout =
        ImmutableList.of(
            new Translation2d(4, 3),
            new Translation2d(5, 3),
            new Translation2d(5, 4),
            new Translation2d(4, 4));
    var clearance = new FieldClearance(field, 0.40, 0.35, 0.06, ImmutableList.of(keepout));
    assertThat(clearance.clearance(pose(3, 3.5, 0))).isGreaterThan(0.06);
    assertThat(clearance.clearance(pose(3.7, 3.5, 0))).isNegative();
  }

  @Test
  void finishThroughKeepsMovingAfterMarkerAndStopsOnOverrun() {
    var sim = new Simulation(0.020);
    sim.planner.activate(
        Trailblazer.segment("finish-through")
            .addGoal(Goal.finishThrough(new Translation2d(1.5, 0), 0.8, 1.5))
            .mark(Marker.FINISH)
            .build());
    sim.untilGoal();
    assertThat(sim.planner.passedMarker(Marker.FINISH)).isTrue();
    assertThat(sim.speed.vxMetersPerSecond).isGreaterThan(0.60);
    while (sim.time < 7) {
      sim.step();
    }
    assertEquals(3, sim.pose.getX(), 0.03);
    assertEquals(0, sim.speed.vxMetersPerSecond, 0.03);
  }

  @Test
  void hardGateBlocksMarkerUntilHeadingReady() {
    var sim = new Simulation(0.020);
    var hold = HeadingPlan.to(Rotation2d.k180deg).gate(HeadingPlan.GatePolicy.HARD);
    sim.planner.activate(
        Trailblazer.segment("gate")
            .addGoal(
                Goal.through(new Translation2d(0.50, 0)).heading(hold).mark(Marker.INTERMEDIATE))
            .addGoal(Goal.stop(pose(1.50, 0, 180)))
            .build());
    sim.step();
    assertThat(sim.planner.passedMarker(Marker.INTERMEDIATE)).isFalse();
    sim.untilGoal();
    assertThat(sim.planner.passedMarker(Marker.INTERMEDIATE)).isTrue();
  }

  @Test
  void headingToleranceSlowsOnlyTheApproachWhenTheTurnNeedsMoreTime() {
    var rotating = new Simulation(0.02);
    var straight = new Simulation(0.02);
    var limits =
        MotionLimits.robotMaximum()
            .withMaxAngularSpeedDegrees(100)
            .withMaxAngularAccelerationDegrees(180);
    rotating.planner.activate(
        Trailblazer.segment("short-turn")
            .withLimits(limits)
            .through(pose(1.2, 0, 170), 0.2, g -> g.headingTolerance(Rotation2d.fromDegrees(8)))
            .stopAt(pose(4, 0, 170))
            .build());
    straight.planner.activate(
        Trailblazer.segment("straight")
            .withLimits(limits)
            .through(new Translation2d(1.2, 0), 0.2)
            .stopAt(pose(4, 0, 0))
            .build());

    double peakBeforeAnchor = 0;
    while (rotating.planner.headingArrivals().isEmpty() && rotating.time < 6) {
      rotating.step();
      straight.step();
      assertThat(rotating.planner.lastResult().infeasible()).isFalse();
      peakBeforeAnchor = Math.max(peakBeforeAnchor, rotating.speed.vxMetersPerSecond);
    }
    assertThat(rotating.planner.headingArrivals())
        .hasSize(1)
        .allMatch(Trailblazer.HeadingArrival::withinTolerance);
    assertThat(straight.pose.getX()).isGreaterThan(rotating.pose.getX() + 0.25);
    double peakAfterAnchor = 0;
    while (rotating.pose.getX() < 3 && rotating.time < 6) {
      rotating.step();
      peakAfterAnchor = Math.max(peakAfterAnchor, rotating.speed.vxMetersPerSecond);
    }
    assertThat(peakAfterAnchor).isGreaterThan(peakBeforeAnchor + 0.25);
  }

  @Test
  void headingToleranceUsesShiftedAnchorWithoutSlowingFeasibleTranslation() {
    var rotating = new Simulation(0.02);
    var straight = new Simulation(0.02);
    var limits = MotionLimits.robotMaximum().withMaxSpeed(1.5).withMaxAcceleration(1);
    var tolerance = Rotation2d.fromDegrees(6);
    rotating.planner.activate(
        Trailblazer.segment("heading-band")
            .withLimits(limits)
            .through(
                pose(3, 0, 20),
                0.2,
                g -> g.heading(HeadingPlan.faceDegrees(20).before(0.5).tolerance(tolerance)))
            .through(pose(4, 0, 35), 0.2, g -> g.headingTolerance(tolerance))
            .stopAt(pose(6, 0, 35))
            .build());
    straight.planner.activate(
        Trailblazer.segment("straight")
            .withLimits(limits)
            .through(new Translation2d(3, 0), 0.2)
            .through(new Translation2d(4, 0), 0.2)
            .stopAt(pose(6, 0, 0))
            .build());

    while (rotating.pose.getX() < 4.05 && rotating.time < 6) {
      rotating.step();
      straight.step();
      assertEquals(
          straight.speed.vxMetersPerSecond,
          rotating.speed.vxMetersPerSecond,
          1e-8,
          rotating.pose + " at " + rotating.time + ": " + rotating.planner.lastResult());
      if (rotating.planner.lastResult().progress() >= 2.5 && rotating.pose.getX() < 3) {
        assertThat(rotating.planner.headingArrivals()).hasSize(1);
      }
    }
    assertThat(rotating.planner.headingArrivals())
        .hasSize(2)
        .allMatch(Trailblazer.HeadingArrival::withinTolerance);
  }

  @Test
  void launchesAtBothEndsOfSupportedLoopRangeAndCancelStopsOutput() {
    for (double dt : new double[] {0.004, 0.020}) {
      var sim = new Simulation(dt);
      sim.planner.activate(Trailblazer.segment("straight").stopAt(0.6, 0, 0).build());
      sim.step();
      assertThat(sim.speed.vxMetersPerSecond).isPositive();
      sim.planner.cancel();
      var idle = sim.planner.update(sim.time, sim.pose, sim.speed);
      assertEquals(RuntimeStatus.IDLE, idle.status());
      assertEquals(0.0, idle.commandedFieldSpeeds().vxMetersPerSecond);
      assertEquals(0.0, idle.commandedFieldSpeeds().vyMetersPerSecond);
      assertEquals(0.0, idle.commandedFieldSpeeds().omegaRadiansPerSecond);
    }
  }

  @Test
  void marksLatestGoalAndPreservesPoseConfiguration() {
    var builder = Trailblazer.segment("configure-pose");
    assertThatThrownBy(() -> builder.mark(Marker.FINISH)).isInstanceOf(IllegalStateException.class);

    var waypoint = pose(2, 1, 90);
    var secondWaypoint = pose(3, 1, 60);
    var heading = HeadingPlan.to(Rotation2d.k180deg).before(0.25);
    var segment =
        builder
            .through(waypoint, goal -> goal)
            .mark(Marker.FINISH)
            .mark(Marker.INTERMEDIATE)
            .through(secondWaypoint, 0.4, goal -> goal.heading(heading))
            .stopAt(pose(4, 1, 0))
            .build();

    assertThat(segment.goals().get(0)).isEqualTo(Goal.through(waypoint).mark(Marker.INTERMEDIATE));
    assertThat(segment.goals().get(1))
        .isEqualTo(Goal.through(secondWaypoint).within(0.4).heading(heading));
  }

  @Test
  void multiCornerSweepExitsFilletsWithoutInfeasibleChatter() {
    var sim = new Simulation(0.020, new EmpiricalDriveCalibration(4.0, 1.6, 0.070));
    sim.pose = pose(1.0, 1.0, 90.0);
    sim.planner.activate(
        Trailblazer.segment("sweep")
            .withLimits(MotionLimits.robotMaximum())
            .through(3.0, 1.0, 0.35, g -> g.faceDegrees(90))
            .through(4.5, 2.5, 0.35, g -> g.faceDegrees(60))
            .through(3.5, 3.8, 0.35, g -> g.faceDegrees(180))
            .through(2.0, 3.5, 0.35, g -> g.faceDegrees(180))
            .stopAt(pose(0.8, 2.5, 180))
            .build());

    double deadline = sim.time + 12.0;
    while (!sim.planner.atGoal() && sim.time < deadline) {
      sim.step();
      var res = sim.planner.lastResult();
      assertThat(res.infeasible())
          .withFailMessage(() -> "Corner exit triggered infeasible at progress=" + res.progress())
          .isFalse();
    }
    assertThat(sim.planner.atGoal()).isTrue();
  }

  @Test
  void nonfiniteStateCommandsZeroAndRecoversNextTick() {
    var sim = new Simulation(0.020);
    sim.planner.activate(Trailblazer.segment("recover").stopAt(2.0, 0.0, 0.0).build());
    sim.step();
    assertThat(sim.speed.vxMetersPerSecond).isPositive();

    var bad =
        sim.planner.update(
            sim.time, new Pose2d(Double.NaN, 0.0, Rotation2d.kZero), new ChassisSpeeds());
    assertEquals(RuntimeStatus.RUNNING, bad.status());
    assertEquals("NONFINITE_STATE", bad.diagnostic());
    assertEquals(0.0, bad.commandedFieldSpeeds().vxMetersPerSecond);

    sim.time += sim.dt;
    sim.untilGoal();
    assertEquals(2.0, sim.pose.getX(), 0.05);
  }

  @Test
  void pathBuilderClampsCornerFilletNearKeepout() {
    var field = new FieldGeometry(12.0, 7.0, FieldSymmetry.ROTATIONAL_180);
    var keepout =
        ImmutableList.of(
            new Translation2d(2.0, 0.0),
            new Translation2d(4.0, 0.0),
            new Translation2d(4.0, 1.2),
            new Translation2d(2.0, 1.2));
    var clearance = new FieldClearance(field, 0.30, 0.30, 0.05, ImmutableList.of(keepout));
    var segment =
        Trailblazer.segment("corner-clamp")
            .through(3.0, 1.8, 1.5, g -> g.faceDegrees(0.0))
            .stopAt(6.0, 0.0, 0.0)
            .build();
    var built = PathBuilder.build(new Translation2d(0.0, 0.0), segment.goals(), 3.0, clearance);
    assertThat(built.warnings().contains("CORNER_FILLET_CLAMPED goal-0"))
        .withFailMessage(built.warnings().toString())
        .isTrue();
    var filletMid = built.sample(built.anchors().get(0)).position();
    assertThat(clearance.clearance(filletMid.getX(), filletMid.getY(), 1.0, 0.0))
        .isGreaterThanOrEqualTo(clearance.margin() - 1e-2);
  }

  @Test
  void rejectsExactReversalWithoutStop() {
    assertThatThrownBy(() ->
            PathBuilder.build(
                Translation2d.kZero,
                ImmutableList.of(Goal.through(new Translation2d(1, 0)), Goal.stop(Pose2d.kZero)),
                3))
        .isInstanceOf(IllegalArgumentException.class);
  }

  @Test
  void resumesSmoothlyAfterMissedLoopTicks() {
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
      assertThat(sim.speed.vxMetersPerSecond).isPositive();
      assertThat(sim.speed.vxMetersPerSecond).isLessThanOrEqualTo(5 * sim.dt + 1e-9);
      sim.step();
    }
    sim.untilGoal();
    assertEquals(2.0, sim.pose.getX(), 0.05);
  }

  @Test
  void roundsCornerAndStopsWithinTolerance() {
    var sim = new Simulation(0.020);
    sim.planner.activate(
        Trailblazer.segment("corner")
            .through(1.0, 0.0, 0.30)
            .stopAt(pose(1.0, 1.0, 0), g -> g.mark(Marker.FINISH))
            .build());
    sim.untilGoal();
    assertThat(sim.planner.passedMarker(Marker.FINISH)).isTrue();
    assertEquals(1.0, sim.pose.getX(), 0.05);
    assertEquals(1.0, sim.pose.getY(), 0.05);
  }

  @Test
  void seededStartOffsetsAndLocalizationResetRecoverOnSyntheticRobot() {
    var random = new Random(581);
    for (int sample = 0; sample < 20; sample++) {
      var sim = new Simulation(random.nextDouble(0.004, 0.004 + 0.016));
      sim.pose =
          pose(
              random.nextDouble(0.16) - 0.08,
              random.nextDouble(0.16) - 0.08,
              random.nextDouble(10) - 5);
      sim.planner.activate(
          Trailblazer.segment("perturbation")
              .addGoal(Goal.stop(pose(4, 0, 0)).mark(Marker.FINISH))
              .build());
      while (sim.time < 0.8) {
        sim.step();
      }
      sim.pose = pose(sim.pose.getX(), sim.pose.getY() + 0.25, sim.pose.getRotation().getDegrees());
      sim.planner.relatch();
      sim.step();
      assertEquals(2, sim.planner.lastResult().geometryGeneration());
      sim.untilGoal();
      assertThat(sim.planner.passedMarker(Marker.FINISH)).isTrue();
    }
  }

  @Test
  void stopPocketSettlesCleanlyUnderLaggedCoupledPlant() {
    var calibration = new EmpiricalDriveCalibration(8.5, 6.0, 0.055);
    var sim = new Simulation(0.020, calibration);
    sim.pose = pose(6.0, 2.57, 180.0);
    var targetPose = pose(3.35, 2.57, -131.1);
    sim.planner.activate(
        Trailblazer.segment("stop-pocket")
            .withLimits(MotionLimits.robotMaximum())
            .through(4.65, 2.57, g -> g.faceDegrees(180.0))
            .stopAt(targetPose)
            .build());

    // First-order lagged drivetrain with azimuth-coupling kick when reversing small commands.
    double alpha = sim.dt / 0.065;
    double prevCmdVx = 0.0;
    double prevCmdVy = 0.0;
    boolean inPocket = false;
    double latchedVxSign = 0.0;
    double latchedVySign = 0.0;
    double latchedOmegaSign = 0.0;
    int postPocketCommandReversals = 0;

    double deadline = sim.time + 8.0;
    while (!sim.planner.atGoal() && sim.time < deadline) {
      var res = sim.planner.update(sim.time, sim.pose, sim.speed);
      assertThat(res.infeasible())
          .withFailMessage(
              () -> "Stop pocket flagged infeasible at t=" + sim.time + " pose=" + sim.pose)
          .isFalse();

      var cmd = res.commandedFieldSpeeds();
      boolean insideHalfTolerance =
          sim.pose.getTranslation().getDistance(targetPose.getTranslation()) <= 0.015
              && Math.abs(sim.pose.getRotation().minus(targetPose.getRotation()).getDegrees())
                  <= 1.0;
      if (insideHalfTolerance) {
        inPocket = true;
      }
      if (inPocket && res.status() == RuntimeStatus.RUNNING) {
        double vxSign =
            Math.abs(cmd.vxMetersPerSecond) > 1e-4 ? Math.signum(cmd.vxMetersPerSecond) : 0.0;
        double vySign =
            Math.abs(cmd.vyMetersPerSecond) > 1e-4 ? Math.signum(cmd.vyMetersPerSecond) : 0.0;
        double omSign =
            Math.abs(cmd.omegaRadiansPerSecond) > 1e-4
                ? Math.signum(cmd.omegaRadiansPerSecond)
                : 0.0;
        if (vxSign != 0.0 && latchedVxSign != 0.0 && vxSign != latchedVxSign) {
          postPocketCommandReversals++;
        }
        if (vySign != 0.0 && latchedVySign != 0.0 && vySign != latchedVySign) {
          postPocketCommandReversals++;
        }
        if (omSign != 0.0 && latchedOmegaSign != 0.0 && omSign != latchedOmegaSign) {
          postPocketCommandReversals++;
        }
        if (vxSign != 0.0) {
          latchedVxSign = vxSign;
        }
        if (vySign != 0.0) {
          latchedVySign = vySign;
        }
        if (omSign != 0.0) {
          latchedOmegaSign = omSign;
        }
      }

      double coupleKick =
          (Math.abs(cmd.vxMetersPerSecond - prevCmdVx)
                  + Math.abs(cmd.vyMetersPerSecond - prevCmdVy))
              * (Math.hypot(cmd.vxMetersPerSecond, cmd.vyMetersPerSecond) > 1e-4 ? 0.12 : 0.0);
      double nextVx =
          sim.speed.vxMetersPerSecond
              + alpha * (cmd.vxMetersPerSecond - sim.speed.vxMetersPerSecond);
      double nextVy =
          sim.speed.vyMetersPerSecond
              + alpha * (cmd.vyMetersPerSecond - sim.speed.vyMetersPerSecond);
      double nextOmega =
          sim.speed.omegaRadiansPerSecond
              + alpha * (cmd.omegaRadiansPerSecond - sim.speed.omegaRadiansPerSecond)
              + Math.copySign(coupleKick, cmd.omegaRadiansPerSecond);
      prevCmdVx = cmd.vxMetersPerSecond;
      prevCmdVy = cmd.vyMetersPerSecond;

      sim.speed = new ChassisSpeeds(nextVx, nextVy, nextOmega);
      sim.pose =
          new Pose2d(
              sim.pose.getX() + sim.speed.vxMetersPerSecond * sim.dt,
              sim.pose.getY() + sim.speed.vyMetersPerSecond * sim.dt,
              sim.pose
                  .getRotation()
                  .plus(Rotation2d.fromRadians(sim.speed.omegaRadiansPerSecond * sim.dt)));
      sim.time += sim.dt;
    }

    assertThat(sim.planner.atGoal())
        .withFailMessage(() -> "Planner failed to settle at " + sim.pose)
        .isTrue();
    assertEquals(0, postPocketCommandReversals);
    assertThat(sim.pose.getTranslation().getDistance(targetPose.getTranslation()))
        .isLessThanOrEqualTo(0.03);
    assertThat(Math.abs(sim.pose.getRotation().minus(targetPose.getRotation()).getDegrees()))
        .isLessThanOrEqualTo(2.0);
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
    assertThat(sim.speed.omegaRadiansPerSecond).isNegative();
    sim.untilGoal();
    assertEquals(90, sim.pose.getRotation().getDegrees(), 2);
  }
}
