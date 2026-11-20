package frc.robot.autos;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assertions.assertEquals;

import com.google.common.collect.ImmutableList;
import com.team581.autos.Point;
import com.team581.mechanisms.imu.BumpCrossingState;
import com.team581.mechanisms.imu.BumpCrossingTracker;
import com.team581.trailblazer3.math.PathBuilder;
import com.team581.trailblazer3.model.AutoSegment;
import com.team581.trailblazer3.model.TickResult;
import com.team581.trailblazer3.model.TickResult.LimitingFactor;
import com.team581.trailblazer3.model.TickResult.RuntimeStatus;
import com.team581.util.FieldUtil;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.config.CompAutoField;
import frc.robot.config.CompTrailblazer;
import frc.robot.power_manager.PowerManagerState;
import frc.robot.robot_manager.RobotState;
import frc.robot.robot_manager.hopper_manager.HopperState;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.junit.jupiter.params.provider.EnumSource;

final class RightTrenchAutoTest {
  @Test
  void allSegmentsBuildWithExpectedGeometryWarnings() {
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
            switch (i) {
              case 2 -> ImmutableList.of("CORNER_FILLET_CLAMPED goal-2");
              default -> ImmutableList.of();
            },
            built.warnings(),
            alliance + " cycle " + (i + 1) + " warnings: " + built.warnings());
        assertThat(built.length() > 1.0)
            .withFailMessage("%s cycle %d length", alliance, i + 1)
            .isTrue();
      }
    }
  }

  @CsvSource({"Blue, false", "Blue, true", "Red, false", "Red, true"})
  @ParameterizedTest
  void crossesResetsAndScoresForThreeCycles(Alliance alliance, boolean right) {
    assertThat(HAL.initialize(500, 0)).isTrue();
    double dt = 0.020;
    double[] time = {0};
    var planner = CompTrailblazer.create(() -> time[0]);
    var context = TrailblazerTrenchAutoTest.testContext(planner, alliance);
    var robot = context.robot();
    var imu = robot.localization.imu;
    var tracker = imu.bumpCrossingTracker;
    var cycle = right ? RightTrenchAuto.CYCLE_1_SEGMENT : LeftTrenchAuto.CYCLE_1_SEGMENT;
    var goals = CompAutoField.segment(cycle, alliance).goals();
    var clearance = CompAutoField.clearance(alliance);
    var landing = goals.get(goals.size() - 2).pose().getTranslation();
    var shoot = goals.get(goals.size() - 1).pose().getTranslation();
    double direction = alliance == Alliance.Red ? 1 : -1;
    TrailblazerAuto<?> auto = right ? new RightTrenchAuto(context) : new LeftTrenchAuto(context);
    Pose2d pose = auto.startPose();
    auto.prepareWhileDisabled(pose, false, false);
    robot.hopperManager.idleRequest();
    assertThat(robot.hopperManager.getState()).isNotEqualTo(HopperState.INTAKING);
    assertThat(robot.powerManager.getState()).isEqualTo(PowerManagerState.IDLE);
    auto.prepareWhileDisabled(pose, true, true);
    robot.hopperManager.idleRequest();
    assertThat(robot.hopperManager.getState()).isEqualTo(HopperState.INTAKING);
    assertThat(robot.powerManager.getState()).isEqualTo(PowerManagerState.AUTO_FIRST_SEGMENT);
    ChassisSpeeds speeds = new ChassisSpeeds();
    String step = "";
    double stepStart = 0;
    int landings = 0;
    boolean displacedDuringFinalShot = false;
    boolean correctedFinalShotPosition = false;
    var visited = new ArrayList<String>();
    var shootingTimes = new ArrayList<Double>();
    var trenchReturns = new HashSet<String>();

    SimHooks.pauseTiming();
    DriverStation.silenceJoystickConnectionWarning(true);
    try {
      for (; time[0] < 40.0; time[0] += dt) {
        robot.localization.resetPose(pose);
        robot.swerve.beforePeriodic();
        imu.beforePeriodic();
        tracker.beforePeriodic();

        auto.periodic();
        if (time[0] == 0) {
          robot.hopperManager.idleRequest();
          assertThat(robot.hopperManager.getState()).isEqualTo(HopperState.INTAKING);
          assertThat(robot.powerManager.getState()).isEqualTo(PowerManagerState.AUTO_FIRST_SEGMENT);
        }
        String nextStep = auto.currentStepName();
        if (!step.equals(nextStep)) {
          if (step.startsWith("SHOOT_")) {
            shootingTimes.add(time[0] - stepStart);
          }
          if (nextStep.startsWith("SHOOT_")) {
            assertEquals(BumpCrossingState.FLAT_NOT_CROSSING, tracker.getState());
            assertThat(landings).isEqualTo(Integer.parseInt(nextStep.substring("SHOOT_".length())));
            robot.hopperManager.idleRequest();
          }
          step = nextStep;
          stepStart = time[0];
          visited.add(step);
        }

        if (step.equals("SHOOT_1") && time[0] - stepStart > 0.1) {
          // The simulated hopper finishes 1.5 s after feeding begins. The auto waits 2 s.
          robot.hopperManager.scoreRequest();
        }
        if (step.equals("SHOOT_3") && time[0] - stepStart > 3.5) {
          break;
        }

        var result = planner.update(time[0], pose, speeds);
        var crossingState = tracker.getState();
        var command = AutoActions.bumpCrossingCommand(result, crossingState, alliance);
        if (step.startsWith("CROSS_BUMP")) {
          assertEquals(direction * 4.0, command.vxMetersPerSecond);
          assertEquals(result.commandedFieldSpeeds().vyMetersPerSecond, command.vyMetersPerSecond);
          assertEquals(
              result.commandedFieldSpeeds().omegaRadiansPerSecond, command.omegaRadiansPerSecond);
        }
        if (step.startsWith("SHOOT_") && pose.getTranslation().getDistance(shoot) < 0.3) {
          assertEquals(0.0, command.vxMetersPerSecond);
          assertEquals(0.0, command.vyMetersPerSecond);
        }
        if (displacedDuringFinalShot && command.vxMetersPerSecond * direction < 0) {
          correctedFinalShotPosition = true;
        }
        double translationResponse =
            -Math.expm1(-dt / CompTrailblazer.CALIBRATION.driveResponseLagSeconds());
        double angularResponse =
            -Math.expm1(-dt / (2 * CompTrailblazer.CALIBRATION.driveResponseLagSeconds()));
        speeds =
            new ChassisSpeeds(
                speeds.vxMetersPerSecond
                    + translationResponse * (command.vxMetersPerSecond - speeds.vxMetersPerSecond),
                speeds.vyMetersPerSecond
                    + translationResponse * (command.vyMetersPerSecond - speeds.vyMetersPerSecond),
                speeds.omegaRadiansPerSecond
                    + angularResponse
                        * (command.omegaRadiansPerSecond - speeds.omegaRadiansPerSecond));
        var nextPose =
            new Pose2d(
                pose.getX() + speeds.vxMetersPerSecond * dt,
                pose.getY() + speeds.vyMetersPerSecond * dt,
                pose.getRotation().plus(Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * dt)));

        if (step.startsWith("CROSS_BUMP")) {
          tracker.periodic();
          if (tracker.getState() == BumpCrossingState.FLAT_NOT_CROSSING) {
            assertEquals(BumpCrossingState.CROSSING_DOWNHILL, crossingState);
            assertThat(direction * (pose.getX() - landing.getX()))
                .as("%s %s landing error before odometry reset", alliance, step)
                .isBetween(-0.10, 0.10);
            var resetPose = robot.swerve.drivetrain.getState().Pose;
            assertThat(resetPose.getTranslation().getDistance(landing)).isLessThan(0.01);
            assertThat(Math.abs(resetPose.getRotation().minus(pose.getRotation()).getDegrees()))
                .isLessThan(0.1);
            landings++;
            nextPose = resetPose;
          }
        }
        pose = nextPose;
        assertThat(clearance.clearance(pose)).as("%s %s at %s", alliance, step, pose).isPositive();
        if (right
            && (step.equals("INTAKE_CYCLE_2") || step.equals("INTAKE_CYCLE_3"))
            && FieldUtil.inTrench(pose.getTranslation())) {
          var bluePose = CompAutoField.pose(pose, alliance);
          var trenchGoals =
              (step.equals("INTAKE_CYCLE_2")
                      ? RightTrenchAuto.CYCLE_2_SEGMENT
                      : RightTrenchAuto.CYCLE_3_SEGMENT)
                  .goals();
          var entryHeading = trenchGoals.get(1).pose().getRotation();
          double exitOffset =
              trenchGoals.get(2).pose().getRotation().minus(entryHeading).getDegrees();
          double yawOffset = bluePose.getRotation().minus(entryHeading).getDegrees();
          assertThat(yawOffset)
              .as("%s %s heading inside the trench at %s", alliance, step, pose)
              .isBetween(Math.min(0.0, exitOffset) - 5.0, Math.max(0.0, exitOffset) + 5.0);
          trenchReturns.add(step);
        }
        if (step.equals("SHOOT_3") && time[0] - stepStart > 0.5 && !displacedDuringFinalShot) {
          pose = new Pose2d(shoot.getX() + 0.35 * direction, shoot.getY(), pose.getRotation());
          displacedDuringFinalShot = true;
        }
        SimHooks.stepTiming(dt);
      }

      assertThat(visited)
          .containsExactly(
              "INTAKE_CYCLE_1",
              "CROSS_BUMP_TO_SHOOT_1",
              "SHOOT_1",
              "INTAKE_CYCLE_2",
              "CROSS_BUMP_TO_SHOOT_2",
              "SHOOT_2",
              "INTAKE_CYCLE_3",
              "CROSS_BUMP_TO_SHOOT_3",
              "SHOOT_3");
      assertThat(shootingTimes).hasSize(2);
      if (right) {
        assertThat(trenchReturns).contains("INTAKE_CYCLE_2", "INTAKE_CYCLE_3");
      }
      assertThat(shootingTimes.get(0)).isBetween(2.0, 2.0 + dt + 1e-6);
      assertThat(shootingTimes.get(1)).isBetween(3.25, 3.25 + dt + 1e-6);
      assertThat(time[0] - stepStart).isGreaterThan(3.5);
      assertThat(correctedFinalShotPosition).isTrue();
      assertThat(pose.getTranslation().getDistance(shoot)).isLessThan(0.3);
      assertEquals(RobotState.PREPARE_SCORE, robot.getState());
    } finally {
      tracker.cancelCrossing();
      imu.setPitch(0);
      imu.setRoll(0);
      DriverStation.silenceJoystickConnectionWarning(false);
      SimHooks.resumeTiming();
    }
  }

  @EnumSource(Alliance.class)
  @ParameterizedTest
  void crossingOverridesOnlyFieldX(Alliance alliance) {
    var planned = new ChassisSpeeds(2.0, -0.2, 0.45);
    var result =
        new TickResult(planned, RuntimeStatus.RUNNING, false, LimitingFactor.NONE, 1, 0, "");
    for (var state : BumpCrossingState.values()) {
      var command = AutoActions.bumpCrossingCommand(result, state, alliance);
      double expectedX =
          state == BumpCrossingState.FLAT_NOT_CROSSING
              ? planned.vxMetersPerSecond
              : (alliance == Alliance.Red ? 4.0 : -4.0);
      assertEquals(expectedX, command.vxMetersPerSecond);
      assertEquals(planned.vyMetersPerSecond, command.vyMetersPerSecond);
      assertEquals(planned.omegaRadiansPerSecond, command.omegaRadiansPerSecond);
    }
  }

  @Test
  void trackerFlatFallbackResetsOdometry() {
    assertThat(HAL.initialize(500, 0)).isTrue();
    var reset = new AtomicReference<Translation2d>();
    double[] pitch = {0};
    var tracker = new BumpCrossingTracker(() -> pitch[0], () -> 0, () -> 0, reset::set);
    var landing = new Pose2d(3.25, 2.5, Rotation2d.kZero);
    tracker.bumpCrossRequest(new Point(landing, landing), Rotation2d.k180deg);

    SimHooks.pauseTiming();
    try {
      pitch[0] = -12;
      tracker.beforePeriodic();
      tracker.periodic();
      assertEquals(BumpCrossingState.CROSSING_UPHILL, tracker.getState());
      assertThat(reset.get()).isNull();

      pitch[0] = 0;
      tracker.beforePeriodic();
      SimHooks.stepTiming(0.76);
      tracker.beforePeriodic();
      tracker.periodic();
      assertEquals(BumpCrossingState.FLAT_NOT_CROSSING, tracker.getState());
      assertEquals(landing.getTranslation(), reset.get());
    } finally {
      SimHooks.resumeTiming();
    }
  }
}
