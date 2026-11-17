package frc.robot.autos;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;

import com.google.common.collect.ImmutableList;
import com.team581.trailblazer3.model.AutoSegment;
import com.team581.trailblazer3.model.TickResult;
import com.team581.trailblazer3.runtime.Trailblazer;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Hardware;
import frc.robot.config.CompAutoField;
import frc.robot.config.CompTrailblazer;
import frc.robot.conveyor.Conveyor;
import frc.robot.deploy.Deploy;
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
import frc.robot.swerve.Swerve;
import frc.robot.vision.CameraConfigs;
import frc.robot.vision.Vision;
import frc.robot.vision.limelight.Limelight;
import frc.robot.vision.limelight.LimelightState;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

final class TrailblazerTrenchAutoTest {
  private static final class Holder {
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
      var shooterHood = new ShooterHood(hardware.shooterHoodMotor);
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
    return new AutoContext(robot, planner, alliance);
  }

  @Test
  void autoPreparedWhileDisabledCommandsTowardAllianceWallAfterWaiting() {
    assertThat(HAL.initialize(500, 0)).isTrue();
    double[] time = {0};
    var planner = CompTrailblazer.create(() -> time[0]);
    for (var alliance : Alliance.values()) {
      var rightAuto = new TrailblazerTrenchAuto(testContext(planner, alliance));
      var rightStart = rightAuto.startPose().orElseThrow();
      assertThat(rightAuto.isNearStartPose(rightStart)).isTrue();
      assertThat(rightAuto.isNearStartPose(Pose2d.kZero)).isFalse();

      var rightWheelTarget = rightAuto.initialDisabledWheelTarget(rightStart);
      assertThat(rightWheelTarget).isPresent();
      double expectedDir = alliance == Alliance.Red ? 1.0 : -1.0;
      assertEquals(expectedDir, rightWheelTarget.orElseThrow().vxMetersPerSecond, 1e-6);
      assertEquals(0.0, rightWheelTarget.orElseThrow().vyMetersPerSecond, 1e-6);

      rightAuto.periodic();
      time[0] += 30;
      var rightResult = planner.update(time[0], rightStart, new ChassisSpeeds());
      assertEquals(TickResult.RuntimeStatus.RUNNING, rightResult.status());
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
      var leftStart = leftAuto.startPose().orElseThrow();
      assertThat(leftAuto.isNearStartPose(leftStart)).isTrue();
      var leftWheelTarget = leftAuto.initialDisabledWheelTarget(leftStart);
      assertThat(leftWheelTarget).isPresent();
      assertEquals(expectedDir, leftWheelTarget.orElseThrow().vxMetersPerSecond, 1e-6);
      assertEquals(0.0, leftWheelTarget.orElseThrow().vyMetersPerSecond, 1e-6);

      leftAuto.periodic();
      var leftResult = planner.update(time[0], leftStart, new ChassisSpeeds());
      assertEquals(TickResult.RuntimeStatus.RUNNING, leftResult.status());
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
  void stateMachineAutoRunsTransitionsMarkersAndTimeoutsInOrder() {
    assertThat(HAL.initialize(500, 0)).isTrue();
    double[] time = {10.0};
    var planner = CompTrailblazer.create(() -> time[0]);
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
