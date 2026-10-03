package frc.robot;

import static org.assertj.core.api.Assertions.assertThat;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.config.FeatureFlags;
import frc.robot.feeder.FeederState;
import frc.robot.robot_manager.RobotManager;
import frc.robot.robot_manager.RobotState;
import frc.robot.shooter.ShooterConfig;
import frc.robot.shooter_hood.ShooterHoodConfig;
import frc.robot.util.AimParameterUtil;
import frc.robot.util.ShotMotion;
import org.junit.jupiter.api.Test;

/**
 * Boots the actual robot with desktop Phoenix devices and drives the normal controller bindings.
 */
final class ShotCoordinatorSimulationTest {
  private static void exerciseCircleFeeding(Robot robot, RobotManager manager) throws Exception {
    assertThat(FeatureFlags.CANCEL_IN_PROGRESS_SHOT.getAsBoolean()).isTrue();
    manager.idleRequest();
    DriverStationSim.setJoystickAxis(0, 3, 0);
    DriverStationSim.setJoystickAxis(0, 1, 0);
    DriverStationSim.setJoystickAxis(0, 0, 0);
    DriverStationSim.setJoystickAxis(0, 4, 0);
    manager.localization.resetPose(new Pose2d(8.5, 2.5, Rotation2d.fromDegrees(20)));
    runCycles(robot, 10);
    DriverStationSim.setJoystickAxis(0, 3, 1);
    runCycles(robot, 120);
    System.out.println(
        "Feed warmup state: "
            + manager.getState()
            + " hood="
            + manager.hardware.shooterHoodMotor.getPosition().getValueAsDouble() * 360
            + " shooter="
            + manager.hardware.shooterTopRightMotor.getVelocity().getValueAsDouble() * 60);
    int interruptions = 0;
    double maxError = 0;
    double maxRatio = 0;
    String firstFailure = "";
    var startPose = manager.localization.getPose();
    var previousPose = startPose;
    double traveled = 0;
    double rotation = 0;
    double maxRpmRatio = 0;
    double maxHoodRatio = 0;
    var feedField = RobotManager.class.getDeclaredField("feedingParameters");
    feedField.setAccessible(true);
    double translationStick = 0.05 + 0.9025 * Math.sqrt(1.0 / 4.75);
    double yawStick = 0.05 + 0.9025 * Math.pow(2.8 / Units.rotationsToRadians(4), 2.0 / 3);
    for (int i = 0; i < 120; i++) {
      double phase = 2 * Math.PI * i / 120;
      DriverStationSim.setJoystickAxis(0, 0, -translationStick * Math.cos(phase));
      DriverStationSim.setJoystickAxis(0, 1, translationStick * Math.sin(phase));
      DriverStationSim.setJoystickAxis(0, 4, yawStick);
      runCycles(robot, 1);
      var pose = manager.localization.getPose();
      traveled += pose.getTranslation().getDistance(previousPose.getTranslation());
      rotation += pose.getRotation().minus(previousPose.getRotation()).getDegrees();
      previousPose = pose;
      double wantedRpm =
          ((VelocityTorqueCurrentFOC) manager.hardware.shooterTopRightMotor.getAppliedControl())
                  .Velocity
              * 60;
      double actualRpm =
          manager.hardware.shooterTopRightMotor.getVelocity().getValueAsDouble() * 60;
      maxRpmRatio =
          Math.max(
              maxRpmRatio, Math.abs(wantedRpm - actualRpm) / ShooterConfig.RPM_TOLERANCE_FEEDING);
      double wantedHood =
          ((PositionVoltage) manager.hardware.shooterHoodMotor.getAppliedControl()).Position * 360;
      double actualHood = manager.hardware.shooterHoodMotor.getPosition().getValueAsDouble() * 360;
      maxHoodRatio =
          Math.max(
              maxHoodRatio,
              Math.abs(wantedHood - actualHood) / ShooterHoodConfig.FEEDING_TOLERANCE);
      var parameters = (AimParameterUtil.AimingParameters) feedField.get(manager);
      double error =
          Math.abs(
              Rotation2d.fromDegrees(manager.turret.getAngle())
                  .minus(Rotation2d.fromDegrees(parameters.turretAngle()))
                  .getDegrees());
      maxError = Math.max(maxError, error);
      maxRatio = Math.max(maxRatio, error / parameters.turretTolerance());
      if (manager.getState() != RobotState.FEED
          || manager.hopperManager.feeder.getState() != FeederState.FEED) {
        interruptions++;
        if (firstFailure.isEmpty()) {
          firstFailure =
              "step="
                  + i
                  + " state="
                  + manager.getState()
                  + " turret="
                  + manager.turret.atGoal(parameters)
                  + " shooter="
                  + manager.shooter.atGoalDebounced()
                  + " error="
                  + error
                  + " tolerance="
                  + parameters.turretTolerance()
                  + " pose="
                  + manager.localization.getPose()
                  + " upcoming="
                  + parameters.upcomingTurretAngle()
                  + " nearTrench="
                  + readField(manager, "nearTrench")
                  + " safeLocation="
                  + readField(manager, "isInSafeFeedingLocation")
                  + " allianceZone="
                  + readField(manager, "isInAllianceZone")
                  + " hoodReady="
                  + ((frc.robot.shooter_hood.ShooterHood) readField(manager, "shooterHood"))
                      .atGoal()
                  + " hoodAngle="
                  + manager.hardware.shooterHoodMotor.getPosition().getValueAsDouble() * 360
                  + " hoodGoal="
                  + ((PositionVoltage) manager.hardware.shooterHoodMotor.getAppliedControl())
                          .Position
                      * 360
                  + " speedSafe="
                  + !manager.swerve.isMovingBeyondSafeSpeed();
        }
      }
    }
    System.out.println(
        "Circle feeding interruptions="
            + interruptions
            + " max angular error="
            + maxError
            + " max tolerance ratio="
            + maxRatio
            + " first="
            + firstFailure);
    double returnDistance = previousPose.getTranslation().getDistance(startPose.getTranslation());
    System.out.println(
        "Circle travel meters="
            + traveled
            + " rotation degrees="
            + rotation
            + " return distance="
            + returnDistance
            + " max RPM tolerance ratio="
            + maxRpmRatio
            + " max hood tolerance ratio="
            + maxHoodRatio);
    assertThat(traveled).isGreaterThan(1.5);
    assertThat(Math.abs(rotation)).isGreaterThan(360);
    assertThat(returnDistance).isLessThan(0.6);
    assertThat(maxRpmRatio).isLessThanOrEqualTo(1);
    assertThat(maxHoodRatio).isLessThanOrEqualTo(1);
    assertThat(interruptions).isEqualTo(0);
    assertThat(maxRatio).isLessThanOrEqualTo(1);
    DriverStationSim.setJoystickAxis(0, 3, 0);
  }

  private static Object readField(Object object, String name) throws Exception {
    var field = object.getClass().getDeclaredField(name);
    field.setAccessible(true);
    return field.get(object);
  }

  private static void runCycles(Robot robot, int cycles) throws InterruptedException {
    for (int i = 0; i < cycles; i++) {
      Thread.sleep(20);
      DriverStationSim.notifyNewData();
      robot.robotPeriodic();
    }
  }

  @Test
  void driverStepUpdatesAllShotActuatorsInTheSameLoop() throws Exception {
    HAL.initialize(500, 0);
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setEnabled(false);
    DriverStationSim.setJoystickAxisCount(0, 6);
    DriverStationSim.setJoystickButtonCount(0, 10);
    DriverStationSim.notifyNewData();
    try (var robot = new Robot()) {
      robot.robotInit();
      var field = Robot.class.getDeclaredField("robotManager");
      field.setAccessible(true);
      var manager = (RobotManager) field.get(robot);
      for (int i = 0; i < 5; i++) {
        robot.robotPeriodic();
      }
      manager.localization.resetPose(new Pose2d(3, 4, Rotation2d.kZero));
      DriverStationSim.setEnabled(true);
      DriverStationSim.notifyNewData();
      manager.homeShooterHoodRequest();
      robot.robotPeriodic();
      manager.prepareScoreRequest();
      robot.robotPeriodic();
      var stationary = (PositionVoltage) manager.hardware.turretMotor.getAppliedControl();
      double stationaryAngle = stationary.Position;
      double stationaryRpm =
          ((VelocityTorqueCurrentFOC) manager.hardware.shooterTopRightMotor.getAppliedControl())
              .Velocity;
      double stationaryHood =
          ((PositionVoltage) manager.hardware.shooterHoodMotor.getAppliedControl()).Position;

      // Keep measured velocity at the pre-step value: the very next loop must anticipate input.
      DriverStationSim.setJoystickAxis(0, 1, -0.6);
      DriverStationSim.setJoystickAxis(0, 4, -0.4);
      DriverStationSim.notifyNewData();
      robot.robotPeriodic();

      var turret = (PositionVoltage) manager.hardware.turretMotor.getAppliedControl();
      var shooter =
          (VelocityTorqueCurrentFOC) manager.hardware.shooterTopRightMotor.getAppliedControl();
      var hood = (PositionVoltage) manager.hardware.shooterHoodMotor.getAppliedControl();
      assertThat(turret.Position).isNotEqualTo(stationaryAngle);
      assertThat(turret.Velocity).isNotEqualTo(0);
      assertThat(shooter.Velocity).isNotEqualTo(stationaryRpm);
      assertThat(hood.Position).isNotEqualTo(stationaryHood);
      assertThat(manager.swerve.driverStillDecidingSotm()).isFalse();

      var blueCommand = manager.swerve.prepareMotionCommand();
      assertThat(blueCommand.vxMetersPerSecond).isPositive();
      DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
      DriverStationSim.notifyNewData();
      robot.robotPeriodic();
      var redCommand = manager.swerve.prepareMotionCommand();
      assertThat(redCommand.vxMetersPerSecond).isNegative();
      assertThat(redCommand.omegaRadiansPerSecond).isEqualTo(blueCommand.omegaRadiansPerSecond);

      DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
      DriverStationSim.setJoystickAxis(0, 1, 0);
      DriverStationSim.setJoystickAxis(0, 4, 0);
      manager.prepareScoreRequest();
      runCycles(robot, 80);
      var startingHeading = manager.localization.getPose().getRotation();
      DriverStationSim.setJoystickAxis(0, 4, -0.15);
      double errorSum = 0;
      int samples = 0;
      for (int i = 0; i < 100; i++) {
        runCycles(robot, 1);
        if (i >= 40) {
          var pose = manager.localization.getPose();
          var release =
              ShotMotion.predict(
                  pose,
                  manager.swerve.getFieldRelativeSpeeds(),
                  manager.swerve.prepareMotionCommand(),
                  0.1);
          var shot = AimParameterUtil.getScoringParameters(release.pose(), release.speeds());
          var desiredFieldAngle =
              Rotation2d.fromDegrees(shot.turretAngle()).plus(release.pose().getRotation());
          var actualFieldAngle =
              Rotation2d.fromDegrees(manager.turret.getAngle()).plus(pose.getRotation());
          errorSum += Math.abs(actualFieldAngle.minus(desiredFieldAngle).getDegrees());
          samples++;
        }
      }
      double meanError = errorSum / samples;
      System.out.println(
          "Native sim mean field aiming error during rotation (degrees): " + meanError);
      assertThat(
              Math.abs(
                  manager.localization.getPose().getRotation().minus(startingHeading).getDegrees()))
          .isGreaterThan(15);
      assertThat(meanError).isLessThan(5);
      DriverStationSim.setJoystickAxis(0, 4, 0);
      runCycles(robot, 60);
      var stoppedPose = manager.localization.getPose();
      var stoppedShot =
          AimParameterUtil.getScoringParameters(
              stoppedPose, manager.swerve.getFieldRelativeSpeeds());
      double stoppedError =
          Math.abs(
              Rotation2d.fromDegrees(manager.turret.getAngle())
                  .minus(Rotation2d.fromDegrees(stoppedShot.turretAngle()))
                  .getDegrees());
      System.out.println("Native sim stopped aiming error (degrees): " + stoppedError);
      assertThat(stoppedError).isLessThan(2);

      exerciseCircleFeeding(robot, manager);

      DriverStationSim.setEnabled(false);
      DriverStationSim.notifyNewData();
      robot.robotPeriodic();
      var disabledCommand = manager.swerve.prepareMotionCommand();
      assertThat(disabledCommand.vxMetersPerSecond).isEqualTo(0);
      assertThat(disabledCommand.vyMetersPerSecond).isEqualTo(0);
      assertThat(disabledCommand.omegaRadiansPerSecond).isEqualTo(0);
    } finally {
      DriverStationSim.resetData();
    }
  }
}
