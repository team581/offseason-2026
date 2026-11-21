package frc.robot.autos;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import com.google.common.collect.ImmutableList;
import com.team581.trailblazer3.model.AutoSegment;
import com.team581.trailblazer3.model.TickResult;
import com.team581.trailblazer3.runtime.Trailblazer;
import com.team581.util.FieldUtil;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Hardware;
import frc.robot.config.CompAutoField;
import frc.robot.conveyor.Conveyor;
import frc.robot.deploy.Deploy;
import frc.robot.deploy.DeployState;
import frc.robot.feeder.Feeder;
import frc.robot.health.HealthManager;
import frc.robot.hub_activity.HubActivity;
import frc.robot.imu.Imu;
import frc.robot.intake.Intake;
import frc.robot.localization.Localization;
import frc.robot.power_manager.PowerManager;
import frc.robot.robot_manager.RobotManager;
import frc.robot.robot_manager.hopper_manager.HopperManager;
import frc.robot.shooter.Shooter;
import frc.robot.shooter_hood.ShooterHood;
import frc.robot.shooter_hood.ShooterHoodState;
import frc.robot.swerve.Swerve;
import frc.robot.vision.CameraConfigs;
import frc.robot.vision.Vision;
import frc.robot.vision.limelight.Limelight;
import frc.robot.vision.limelight.LimelightState;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

final class TrailblazerTrenchAutoTest {
  private static final class Holder {
    static final AtomicInteger HOOD_HOMING_REQUESTS = new AtomicInteger();
    static ShooterHood shooterHood;
    static final RobotManager ROBOT_MANAGER = createRobotManager();

    private static RobotManager createRobotManager() {
      assertThat(HAL.initialize(500, 0)).isTrue();
      var hardware = new Hardware();
      var shooterLimelight = new Limelight("shooter", LimelightState.TAGS, CameraConfigs.SHOOTER);
      var leftLimelight = new Limelight("left", LimelightState.TAGS, CameraConfigs.LEFT);
      var rightLimelight = new Limelight("right", LimelightState.TAGS, CameraConfigs.RIGHT);
      var health = new HealthManager(shooterLimelight, leftLimelight, rightLimelight);
      var imu = new Imu(hardware.drivetrain);
      var swerve = new Swerve(hardware.drivetrain, health, hardware.driverController);
      imu.setDriveStateSupplier(swerve::getDriveState);
      shooterHood =
          new ShooterHood(hardware.shooterHoodMotor) {
            @Override
            public void homingRequest() {
              HOOD_HOMING_REQUESTS.incrementAndGet();
              super.homingRequest();
            }
          };
      var shooter =
          new Shooter(
              hardware.shooterTopLeftMotor,
              hardware.shooterTopRightMotor,
              hardware.shooterBottomLeftMotor,
              hardware.shooterBottomRightMotor);
      var intake = new Intake(hardware.intakeLeftMotor, hardware.intakeRightMotor);
      var deploy = new Deploy(hardware.deployDifferentialMechanism);
      var vision = new Vision(imu, shooterLimelight, leftLimelight, rightLimelight);
      var localization = new Localization(swerve, hardware.drivetrain, vision, imu);
      var feeder = new Feeder(hardware.feederTopMotor, hardware.feederBottomMotor);
      var conveyor = new Conveyor(hardware.conveyorTopMotor, hardware.conveyorBottomMotor);
      var hubActivity = new HubActivity();
      var powerManager =
          new PowerManager(shooter, intake, deploy, shooterHood, feeder, conveyor, swerve);
      var hopperManager =
          new HopperManager(
              deploy, intake, conveyor, feeder, hardware.hopperCANRange, hardware.towerSensor);
      return new RobotManager(
          hopperManager,
          shooterHood,
          localization,
          swerve,
          shooter,
          vision,
          hardware.driverController,
          health,
          hubActivity,
          hardware,
          powerManager);
    }
  }

  private enum SequenceState {
    DRIVE_LEG,
    WAIT_READY,
    SHORT_PAUSE,
    DONE
  }

  private enum SingleDriveState {
    DRIVE,
    DONE
  }

  private enum TestMarker {
    MIDPOINT
  }

  private static void assertLaunchMatches(ChassisSpeeds staged, ChassisSpeeds live) {
    assertEquals(live.vxMetersPerSecond, staged.vxMetersPerSecond, 1e-9);
    assertEquals(live.vyMetersPerSecond, staged.vyMetersPerSecond, 1e-9);
    assertEquals(live.omegaRadiansPerSecond, staged.omegaRadiansPerSecond, 1e-9);
  }

  private static TickResult stepKinematics(
      Trailblazer planner, double[] time, Pose2d[] pose, ChassisSpeeds[] speeds) {
    var result = planner.update(time[0], pose[0], speeds[0]);
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

  static AutoContext testContext(Trailblazer planner, Alliance alliance) {
    var robot = Holder.ROBOT_MANAGER;
    robot.idleRequest();
    robot.cancelIntakeRequest();
    robot.hopperManager.setWantsSafeStow(false);
    robot.powerManager.idleRequest();
    return new AutoContext(robot, planner, alliance, Runnable::run);
  }

  @Test
  void autoPreparedWhileDisabledCommandsTowardAllianceWallAfterWaiting() {
    assertThat(HAL.initialize(500, 0)).isTrue();
    double[] time = {0};
    var planner = TestTrailblazer.create(() -> time[0]);
    for (var alliance : Alliance.values()) {
      var rightAuto = new TrailblazerTrenchAuto(testContext(planner, alliance));
      var rightStart = rightAuto.startPose();
      var deploy = rightAuto.robot.hopperManager.deploy;
      var hood = Holder.shooterHood;
      deploy.cancelOutwardHomingRequest();
      var hoodHomingRequests = Holder.HOOD_HOMING_REQUESTS.get();
      assertThat(rightAuto.isNearStartPose(rightStart)).isTrue();
      assertThat(rightAuto.isNearStartPose(Pose2d.kZero)).isFalse();

      rightAuto.prepareWhileDisabled(rightStart, false, false);
      assertThat(deploy.getState()).isNotEqualTo(DeployState.HOME_OUTWARD);
      assertThat(Holder.HOOD_HOMING_REQUESTS.get()).isEqualTo(hoodHomingRequests);

      rightAuto.prepareWhileDisabled(rightStart, true, true);
      var rightLaunch = rightAuto.stagedLaunch(rightStart);
      assertThat(rightLaunch).isNotEqualTo(new ChassisSpeeds());
      assertThat(rightAuto.stagedLaunch(Pose2d.kZero)).isEqualTo(new ChassisSpeeds());
      assertThat(rightAuto.hasStartedRunning()).isFalse();
      assertThat(Holder.HOOD_HOMING_REQUESTS.get()).isEqualTo(hoodHomingRequests + 1);
      deploy.simulationPeriodic();
      hood.simulationPeriodic();
      assertThat(deploy.getState()).isEqualTo(DeployState.HOME_OUTWARD);
      assertThat(hood.getState()).isEqualTo(ShooterHoodState.HOMING);
      var stagedPath = planner.sampledPathPoses(Trailblazer.PATH_LOG_SPACING);
      var stagedGeneration = planner.pathGeneration();
      assertThat(stagedPath).isNotEmpty();
      assertThat(planner.lastResult().commandedFieldSpeeds()).isEqualTo(new ChassisSpeeds());

      time[0] += 30;
      rightAuto.prepareWhileDisabled(rightStart, true, true);
      assertThat(planner.pathGeneration()).isEqualTo(stagedGeneration);
      rightAuto.robot.localization.resetPose(rightStart);
      rightAuto.periodic();
      assertThat(Holder.HOOD_HOMING_REQUESTS.get()).isEqualTo(hoodHomingRequests + 1);
      assertThat(deploy.getState()).isEqualTo(DeployState.HOME_OUTWARD);
      assertThat(hood.getState()).isEqualTo(ShooterHoodState.HOMING);
      var rightResult = planner.update(time[0], rightStart, new ChassisSpeeds());
      assertThat(rightResult.geometryGeneration()).isEqualTo(stagedGeneration);
      assertThat(planner.sampledPathPoses(Trailblazer.PATH_LOG_SPACING))
          .containsExactly(stagedPath);
      assertEquals(TickResult.RuntimeStatus.RUNNING, rightResult.status());
      assertLaunchMatches(rightLaunch, rightResult.commandedFieldSpeeds());
      assertThat(
              rightResult.commandedFieldSpeeds().vxMetersPerSecond
                  * rightStart.getRotation().getCos())
          .isPositive();
      time[0] += 0.02;
      planner.cancel();
      time[0] += 0.02;

      var leftAuto =
          new TrailblazerAuto<>(
              SingleDriveState.DRIVE,
              CompAutoField.LEFT_TRENCH_START,
              testContext(planner, alliance),
              LeftTrenchLeaveAuto.LEAVE_TRENCH_SEGMENT) {
            @Override
            protected void afterTransition(SingleDriveState newState) {
              if (newState == SingleDriveState.DRIVE) {
                drive(LeftTrenchLeaveAuto.LEAVE_TRENCH_SEGMENT);
              } else {
                stopDriving();
              }
            }

            @Override
            protected SingleDriveState getNextState(SingleDriveState currentState) {
              return currentState == SingleDriveState.DRIVE && atGoal()
                  ? SingleDriveState.DONE
                  : currentState;
            }
          };
      var leftStart = leftAuto.startPose();
      assertThat(leftAuto.isNearStartPose(leftStart)).isTrue();
      leftAuto.prepareWhileDisabled(leftStart, true, true);
      var leftLaunch = leftAuto.stagedLaunch(leftStart);
      assertThat(leftLaunch).isNotEqualTo(new ChassisSpeeds());

      leftAuto.robot.localization.resetPose(leftStart);
      leftAuto.periodic();
      var leftResult = planner.update(time[0], leftStart, new ChassisSpeeds());
      assertEquals(TickResult.RuntimeStatus.RUNNING, leftResult.status());
      assertLaunchMatches(leftLaunch, leftResult.commandedFieldSpeeds());
      assertThat(
              leftResult.commandedFieldSpeeds().vxMetersPerSecond
                  * leftStart.getRotation().getCos())
          .isPositive();
      time[0] += 0.02;
      planner.cancel();
      time[0] += 0.02;
    }
  }

  @Test
  void doNothingProvidesTheAllianceStartingPoseForDisabledReset() {
    var planner = TestTrailblazer.create();
    var blueAuto = new DoNothingAuto(testContext(planner, Alliance.Blue));
    var redAuto = new DoNothingAuto(testContext(planner, Alliance.Red));

    assertThat(blueAuto.startPose()).isEqualTo(Pose2d.kZero);
    assertThat(redAuto.startPose()).isEqualTo(FieldUtil.pathflip(Pose2d.kZero));
  }

  @Test
  void failedLivePlanStopsTheAutoWithoutSubmittingMoreWork() {
    assertThat(HAL.initialize(500, 0)).isTrue();
    var work = new ArrayDeque<Runnable>();
    var planner = TestTrailblazer.create();
    var robot = testContext(planner, Alliance.Blue).robot();
    var start = new Pose2d(2, 2, Rotation2d.kZero);
    var reversal = Trailblazer.segment("reversal").through(3, 2, 0).stopAt(start).build();
    var auto =
        new TrailblazerAuto<>(
            SingleDriveState.DRIVE,
            start,
            new AutoContext(robot, planner, Alliance.Blue, work::add),
            reversal) {
          @Override
          protected void afterTransition(SingleDriveState state) {
            drive(reversal);
          }
        };

    robot.localization.resetPose(start);
    auto.periodic();
    work.removeFirst().run();
    auto.periodic();
    auto.periodic();

    assertThat(auto.canDrive()).isFalse();
    assertThat(work).isEmpty();
    assertThat(planner.update(0, start, new ChassisSpeeds()).status())
        .isEqualTo(TickResult.RuntimeStatus.IDLE);
  }

  @Test
  void mixedRedAndBlueAuthoringKeepsTheSelectedAllianceCoordinates() {
    var blue = CompAutoField.fromBlue(2.0, 3.0, 45);
    var red = CompAutoField.fromRed(12.0, 6.0, -90);
    var segment = Trailblazer.segment("mixed-frames").through(blue).stopAt(red).build();

    assertEquals(blue, CompAutoField.segment(segment, Alliance.Blue).goals().get(0).pose());
    var redGoal = CompAutoField.segment(segment, Alliance.Red).goals().get(1).pose();
    assertEquals(12.0, redGoal.getX(), 1e-12);
    assertEquals(6.0, redGoal.getY(), 1e-12);
    assertEquals(-90, redGoal.getRotation().getDegrees(), 1e-12);
  }

  @Test
  void shiftedStartWaitsForWorkerAndUsesPreparedGeometryOnFirstUpdate() {
    assertThat(HAL.initialize(500, 0)).isTrue();
    var work = new ArrayDeque<Runnable>();
    var planner = TestTrailblazer.create();
    var robot = testContext(planner, Alliance.Blue).robot();
    var auto = new TrailblazerTrenchAuto(new AutoContext(robot, planner, Alliance.Blue, work::add));
    var nominal = auto.startPose();
    auto.prepareWhileDisabled(nominal, true, true);
    work.removeFirst().run();
    auto.prepareWhileDisabled(nominal, true, true);
    var stagedGeneration = planner.pathGeneration();

    var shifted = CompAutoField.offset(nominal, 0.12, 0);
    robot.localization.resetPose(shifted);
    auto.periodic();
    auto.periodic();
    assertThat(work).hasSize(1);
    assertThat(planner.update(0.0, shifted, new ChassisSpeeds()).status())
        .isEqualTo(TickResult.RuntimeStatus.IDLE);
    assertThat(planner.pathGeneration()).isEqualTo(-1);

    work.removeFirst().run();
    var measured = CompAutoField.offset(shifted, 0.01, 0);
    robot.localization.resetPose(measured);
    auto.periodic();
    assertThat(work).isEmpty();
    var shiftedGeneration = planner.pathGeneration();
    assertThat(shiftedGeneration).isGreaterThan(stagedGeneration);
    assertThat(planner.sampledPathPoses(Trailblazer.PATH_LOG_SPACING)[0]).isEqualTo(shifted);

    var first = planner.update(0.02, measured, new ChassisSpeeds());
    assertThat(first.status()).isEqualTo(TickResult.RuntimeStatus.RUNNING);
    assertThat(first.geometryGeneration()).isEqualTo(shiftedGeneration);
    assertThat(planner.sampledPathPoses(Trailblazer.PATH_LOG_SPACING)[0]).isEqualTo(shifted);
  }

  @Test
  void stagesFirstSegmentWhileLaterSegmentsAreStillQueued() {
    assertThat(HAL.initialize(500, 0)).isTrue();
    var work = new ArrayDeque<Runnable>();
    var planner = TestTrailblazer.create();
    var robot = testContext(planner, Alliance.Blue).robot();
    var auto = new RightTrenchAuto(new AutoContext(robot, planner, Alliance.Blue, work::add));
    var start = auto.startPose();

    auto.prepareWhileDisabled(start, true, true);
    assertThat(auto.stagedLaunch(start)).isEqualTo(new ChassisSpeeds());
    work.removeFirst().run();
    assertThat(work).isNotEmpty();
    auto.prepareWhileDisabled(start, true, true);
    assertThat(planner.sampledPathPoses(Trailblazer.PATH_LOG_SPACING)).isNotEmpty();

    var launch = auto.stagedLaunch(start);
    assertThat(launch).isNotEqualTo(new ChassisSpeeds());
    robot.localization.resetPose(start);
    auto.periodic();
    var enabled = planner.update(0, start, new ChassisSpeeds());
    assertThat(enabled.status()).isEqualTo(TickResult.RuntimeStatus.RUNNING);
    assertLaunchMatches(launch, enabled.commandedFieldSpeeds());
  }

  @Test
  void stateMachineAutoRunsTransitionsMarkersAndTimeoutsInOrder() {
    assertThat(HAL.initialize(500, 0)).isTrue();
    double[] time = {10.0};
    var planner = TestTrailblazer.create(() -> time[0]);
    Pose2d[] pose = {CompAutoField.pose(1.0, 0.30, 0.0)};
    ChassisSpeeds[] speeds = {new ChassisSpeeds()};
    List<String> log = new ArrayList<>();
    boolean[] ready = {false};
    AtomicInteger waitLoops = new AtomicInteger(0);

    AutoSegment leg1 =
        Trailblazer.segment("leg1")
            .maxSpeed(3.0)
            .through(2.0, 0.30, 0.20, g -> g.mark(TestMarker.MIDPOINT))
            .stopAt(2.5, 0.30, 0.0)
            .build();

    var auto =
        new TrailblazerAuto<SequenceState>(
            SequenceState.DRIVE_LEG,
            CompAutoField.pose(1.0, 0.30, 0.0),
            testContext(planner, Alliance.Blue),
            leg1) {
          boolean loggedMidpoint = false;

          @Override
          protected void afterTransition(SequenceState newState) {
            switch (newState) {
              case DRIVE_LEG -> {
                log.add("drive_start");
                driveWithoutClearance(leg1);
              }
              case WAIT_READY -> {
                log.add("wait_start");
                stopDriving();
                ready[0] = true;
              }
              case SHORT_PAUSE -> stopDriving();
              case DONE -> log.add("done");
            }
          }

          @Override
          protected SequenceState getNextState(SequenceState currentState) {
            return switch (currentState) {
              case DRIVE_LEG -> atGoal() ? SequenceState.WAIT_READY : currentState;
              case WAIT_READY -> ready[0] ? SequenceState.SHORT_PAUSE : currentState;
              case SHORT_PAUSE -> timeout(0.04) ? SequenceState.DONE : currentState;
              case DONE -> currentState;
            };
          }

          @Override
          protected void whileInState(SequenceState state) {
            if (state == SequenceState.DRIVE_LEG
                && !loggedMidpoint
                && passedMarker(TestMarker.MIDPOINT)) {
              loggedMidpoint = true;
              log.add("midpoint");
            } else if (state == SequenceState.WAIT_READY) {
              waitLoops.incrementAndGet();
            }
          }
        };

    auto.periodic();
    assertEquals(ImmutableList.of("drive_start"), log);
    assertEquals("DRIVE_LEG", auto.currentStepName());
    stepKinematics(planner, time, pose, speeds);
    assertNotEquals("KEEPOUT_VIOLATION", planner.lastResult().diagnostic());

    int steps = 0;
    while (auto.getState() != SequenceState.WAIT_READY && steps++ < 250) {
      auto.periodic();
      if (auto.getState() != SequenceState.WAIT_READY) {
        stepKinematics(planner, time, pose, speeds);
        assertNotEquals("KEEPOUT_VIOLATION", planner.lastResult().diagnostic());
      }
    }

    assertEquals(SequenceState.WAIT_READY, auto.getState());
    assertEquals(1, waitLoops.get());
    assertThat(log).contains("midpoint");
    stepKinematics(planner, time, pose, speeds);

    auto.periodic();
    assertEquals(SequenceState.SHORT_PAUSE, auto.getState());
    stepKinematics(planner, time, pose, speeds);

    for (int i = 0; i < 6 && auto.getState() != SequenceState.DONE; i++) {
      auto.periodic();
      stepKinematics(planner, time, pose, speeds);
    }
    assertEquals(SequenceState.DONE, auto.getState());
    assertEquals("DONE", auto.currentStepName());
    assertEquals(ImmutableList.of("drive_start", "midpoint", "wait_start", "done"), log);
    assertThat(auto.isNearStartPose(CompAutoField.pose(1.1, 0.30, 5.0))).isTrue();
    assertThat(auto.isNearStartPose(Pose2d.kZero)).isFalse();
  }
}
