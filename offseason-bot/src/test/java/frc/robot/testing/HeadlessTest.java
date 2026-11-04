package frc.robot.testing;

import static com.google.common.base.Preconditions.checkArgument;
import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.data.Offset.offset;

import dev.doglog.DogLog;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.robot.HeadlessRunner;
import frc.robot.config.DSOptions;
import frc.robot.deploy.DeployState;
import frc.robot.robot_manager.RobotState;
import frc.robot.robot_manager.hopper_manager.HopperState;
import frc.robot.shooter.ShooterState;
import frc.robot.shooter_hood.ShooterHoodConfig;
import frc.robot.shooter_hood.ShooterHoodState;
import frc.robot.turret.TurretCalculator;
import frc.robot.turret.TurretConfig;
import frc.robot.turret.TurretState;
import frc.robot.util.AimParameterUtil;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Timeout;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

@Tag("headless")
final class HeadlessTest {
  private static HeadlessRunner robot;

  /** Holds an impact position through real 20 ms scheduler ticks and refreshed motor signals. */
  private static void holdDeploy(double angleDegrees, double seconds, DeployState expected) {
    double until = Timer.getFPGATimestamp() + seconds;
    while (Timer.getFPGATimestamp() < until) {
      robot.seedDeployAngle(angleDegrees);
      robot.tick();
      assertThat(robot.deploy().getState()).isEqualTo(expected);
    }
  }

  private static void initializeRobot() throws IOException {
    assertThat(HAL.initialize(500, 0)).as("Initialize desktop HAL").isTrue();
    DriverStation.silenceJoystickConnectionWarning(true);
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    double voltage = Double.parseDouble(System.getProperty("sim.batteryVoltage", "12.0"));
    checkArgument(Double.isFinite(voltage) && voltage > 0, "Battery voltage must be positive");
    RoboRioSim.setVInVoltage(voltage);
    if (robot == null) {
      Path logs = Path.of(System.getProperty("sim.logDir", "build/reports/headlessTest/logs"));
      Files.createDirectories(logs);
      DataLogManager.start(logs.toString());
      robot = new HeadlessRunner();
    }
  }

  private static boolean logFlushed(String runId) throws IOException {
    try (var paths = Files.list(Path.of(DataLogManager.getLogDir()))) {
      for (var path : paths.filter(p -> p.toString().endsWith(".wpilog")).toList()) {
        int marker = -1;
        for (var record : new DataLogReader(path.toString())) {
          if (record.isStart()
              && record.getStartData().name.equals("/Robot/Tests/Headless/CompletedRun")) {
            marker = record.getStartData().entry;
          } else if (!record.isControl()
              && record.getEntry() == marker
              && record.getString().equals(runId)) {
            return true;
          }
        }
      }
    }
    return false;
  }

  @AfterAll
  static void closeRobot() throws IOException, InterruptedException {
    if (robot != null) {
      robot.close();
      // Stop DogLog's background producers before flushing and closing their data log.
      DogLog.setOptions(DogLog.getOptions().withLogExtras(false).withCaptureNt(false));
      String runId = robot.manager().getRunId();
      DogLog.log("Tests/Headless/CompletedRun", runId);
      DogLog.setEnabled(false);
      var log = DataLogManager.getLog();
      log.flush();
      try {
        // flush() is asynchronous; verify the final entries reached disk before stopping the
        // writer.
        long deadline = System.nanoTime() + 5_000_000_000L;
        while (!logFlushed(runId) && System.nanoTime() < deadline) {
          Thread.sleep(10);
        }
        assertThat(logFlushed(runId)).as("Normal robot log includes the completed run").isTrue();
      } finally {
        DataLogManager.stop();
        log.close();
      }
    }
  }

  @Test
  @Timeout(60)
  void deployCollapsesAndRecoversThroughTheRobotScheduler() throws IOException {
    org.junit.jupiter.api.Assumptions.assumeTrue(
        System.getProperty("sim.test", "DEPLOY_COLLAPSE").equals("DEPLOY_COLLAPSE"));
    initializeRobot();
    try (var testMode =
        NetworkTableInstance.getDefault()
            .getTable("DSOptions")
            .getBooleanTopic("TestMode")
            .publish()) {
      testMode.set(false);
      robot.tick();
      DriverStationSim.setEnabled(true);
      robot.deploy().homingRequest();
      robot.runUntil(() -> robot.deploy().getState() == DeployState.INTAKE_COAST, 5);
      // Staying down for longer than the delay must never restore PID.
      holdDeploy(0, 3.3, DeployState.INTAKE_COAST);
      robot.seedDeployAngle(20);
      robot.runUntil(() -> robot.deploy().getState() == DeployState.INTAKE_COLLAPSED, 1);
      holdDeploy(20, 1.5, DeployState.INTAKE_COLLAPSED);
      robot.seedDeployAngle(8);
      robot.runUntil(() -> robot.deploy().getState() == DeployState.INTAKE_COAST, 1);
      robot.seedDeployAngle(20);
      robot.runUntil(() -> robot.deploy().getState() == DeployState.INTAKE_COLLAPSED, 1);
      // The first bump must not contribute to the new three-second delay.
      holdDeploy(20, 2.0, DeployState.INTAKE_COLLAPSED);
      robot.runUntil(() -> robot.deploy().getState() == DeployState.INTAKE_RECOVERING, 2);
      robot.runUntil(() -> robot.deploy().getState() == DeployState.INTAKE_COAST, 3);
      assertThat(robot.deploy().getAngleDegrees()).isBetween(-5.0, 5.0);
      holdDeploy(0, 0.3, DeployState.INTAKE_COAST);
    } finally {
      DriverStationSim.setEnabled(false);
      robot.tick();
    }
  }

  @Test
  @Timeout(60)
  void presetFeedShotTransitionsToFallbackFeedAndRunsFeeder() throws IOException {
    org.junit.jupiter.api.Assumptions.assumeTrue(
        System.getProperty("sim.test", "PRESET_FEED").equals("PRESET_FEED"));
    initializeRobot();
    var useVision = (BooleanEntry) DSOptions.USE_VISION;
    boolean previousVision = useVision.get();
    try (var resetPose =
            NetworkTableInstance.getDefault()
                .getTable("DSOptions")
                .getBooleanTopic("ResetPoseForAuto")
                .publish();
        var testMode =
            NetworkTableInstance.getDefault()
                .getTable("DSOptions")
                .getBooleanTopic("TestMode")
                .publish()) {
      resetPose.set(false);
      testMode.set(false);
      useVision.set(false);

      DriverStationSim.setEnabled(false);
      robot.tick();
      // Set pose away from trenches, heading 0 degrees
      robot.robotManager.localization.resetPose(new Pose2d(2.0, 4.0, Rotation2d.kZero));
      DriverStationSim.setEnabled(true);
      robot.tick();

      // Home the hood so it can accept feed requests
      robot.robotManager.homeShooterHoodRequest();
      robot.runUntil(() -> robot.shooterHood().getState() == ShooterHoodState.IDLE, 2.0);

      // Verify localization is reported unhealthy because UseVision is disabled
      assertThat(robot.health().isLocalizationHealthy()).isFalse();

      // Request feed; should enter PREPARE_FALLBACK_FEED
      robot.robotManager.prepareFeedRequest();
      robot.tick();
      assertThat(robot.robotManager.getState()).isEqualTo(RobotState.PREPARE_FALLBACK_FEED);
      assertThat(robot.robotManager.shooter.getState()).isEqualTo(ShooterState.PREPARE_FEED);
      assertThat(robot.shooterHood().getState()).isEqualTo(ShooterHoodState.FEEDING);
      assertThat(robot.turret.getState()).isEqualTo(TurretState.FEED);
      assertThat(robot.robotManager.hopperManager.getState()).isEqualTo(HopperState.IDLE_DEPLOYED);

      // Run until turret, shooter, and hood reach their setpoints and it transitions to
      // FALLBACK_FEED
      robot.runUntil(() -> robot.robotManager.getState() == RobotState.FALLBACK_FEED, 5.0);
      assertThat(robot.robotManager.getState()).isEqualTo(RobotState.FALLBACK_FEED);
      assertThat(robot.robotManager.shooter.getState()).isEqualTo(ShooterState.FEED);
      assertThat(robot.robotManager.hopperManager.getState()).isEqualTo(HopperState.FEED);

      // Release feed and return to idle
      robot.robotManager.idleRequest();
      robot.runUntil(() -> robot.robotManager.getState() == RobotState.IDLE, 1.0);
      assertThat(robot.robotManager.getState()).isEqualTo(RobotState.IDLE);
    } finally {
      useVision.set(previousVision);
      robot.robotManager.idleRequest();
      DriverStationSim.setEnabled(false);
      robot.tick();
    }
  }

  @Test
  @Timeout(30)
  void repeatedHoodHomingSettlesAtIdleThroughOperatorButton() throws IOException {
    org.junit.jupiter.api.Assumptions.assumeTrue(
        System.getProperty("sim.test", "HOOD_HOMING").equals("HOOD_HOMING"));
    initializeRobot();
    try (var testMode =
        NetworkTableInstance.getDefault()
            .getTable("DSOptions")
            .getBooleanTopic("TestMode")
            .publish()) {
      testMode.set(false);
      DriverStationSim.setJoystickButtonCount(1, 10);
      DriverStationSim.setJoystickButtons(1, 0);
      DriverStationSim.setEnabled(true);
      robot.tick();
      for (int press = 1; press <= 3; press++) {
        DriverStationSim.setJoystickButtons(1, 1 << 6);
        robot.tick();
        DriverStationSim.setJoystickButtons(1, 0);
        robot.tick();
        double settleUntil = Timer.getFPGATimestamp() + 1.0;
        robot.runUntil(() -> Timer.getFPGATimestamp() >= settleUntil, 2.0);
        assertThat(robot.shooterHood().getState()).isEqualTo(ShooterHoodState.IDLE);
        assertThat(robot.shooterHood().getAngle())
            .as("Hood settles at idle after operator homing press %s", press)
            .isCloseTo(ShooterHoodConfig.IDLE_ANGLE, offset(0.5));
      }
    } finally {
      DriverStationSim.setJoystickButtons(1, 0);
      DriverStationSim.setEnabled(false);
      robot.tick();
    }
  }

  @EnumSource(value = TestSelection.class, names = "NONE", mode = EnumSource.Mode.EXCLUDE)
  @ParameterizedTest
  @Timeout(120)
  void runsRegisteredTest(TestSelection selection) throws IOException {
    org.junit.jupiter.api.Assumptions.assumeTrue(
        System.getProperty("sim.test", selection.name()).equals(selection.name()));
    initializeRobot();
    double heading = Double.parseDouble(System.getProperty("sim.startHeading", "0.0"));
    checkArgument(Double.isFinite(heading), "Start heading must be finite");
    // Avoid the auto chooser resetting the test's starting pose while disabled.
    try (var resetPose =
            NetworkTableInstance.getDefault()
                .getTable("DSOptions")
                .getBooleanTopic("ResetPoseForAuto")
                .publish();
        var testMode =
            NetworkTableInstance.getDefault()
                .getTable("DSOptions")
                .getBooleanTopic("TestMode")
                .publish()) {
      testMode.set(false);
      resetPose.set(false);
      robot.select(selection, new Pose2d(2, 2, Rotation2d.fromDegrees(heading)));
      var manager = robot.manager();
      DriverStationSim.setEnabled(true);
      robot.tick();
      testMode.set(true);
      try {
        // Process the activation before checking a result retained from the previous run.
        robot.tick();
        robot.runUntil(() -> manager.getResult().filter(TestResult::finished).isPresent(), 30.0);
      } finally {
        testMode.set(false);
        DriverStationSim.setEnabled(false);
        robot.tick();
      }
      var result = manager.getResult().orElseThrow();
      assertThat(result.status()).as(result.reason()).isEqualTo(TestStatus.PASSED);
    }
  }

  @Test
  @Timeout(60)
  void turretTracksSimulatedPoseWithoutCameraFrames() throws IOException {
    org.junit.jupiter.api.Assumptions.assumeTrue(
        System.getProperty("sim.test", "TURRET_AIM").equals("TURRET_AIM"));
    initializeRobot();
    try (var resetPose =
            NetworkTableInstance.getDefault()
                .getTable("DSOptions")
                .getBooleanTopic("ResetPoseForAuto")
                .publish();
        var testMode =
            NetworkTableInstance.getDefault()
                .getTable("DSOptions")
                .getBooleanTopic("TestMode")
                .publish()) {
      resetPose.set(false);
      testMode.set(false);
      // Face away from the hub so both targets are inside the physical rear-facing travel.
      for (double y : new double[] {2.0, 6.0}) {
        DriverStationSim.setEnabled(false);
        robot.tick();
        robot.robotManager.localization.resetPose(new Pose2d(2.0, y, Rotation2d.k180deg));
        DriverStationSim.setEnabled(true);
        robot.robotManager.warmupScoreRequest();
        double until = Timer.getFPGATimestamp() + 2.0;
        robot.runUntil(() -> Timer.getFPGATimestamp() >= until, 3.0);
        var pose = robot.robotManager.localization.getPose();
        var parameters =
            AimParameterUtil.getStaticScoringParameters(
                pose, new ChassisSpeeds(), new ChassisSpeeds());
        double expected = TurretCalculator.getOptimalAngle(parameters.turretAngle(), -180.0);
        assertThat(expected).isStrictlyBetween(TurretConfig.MIN_ANGLE, TurretConfig.MAX_ANGLE);
        assertThat(robot.turret.getAngle())
            .as("Turret aims from the simulated robot pose at y=%s", y)
            .isCloseTo(expected, offset(1.0));
      }
    } finally {
      robot.robotManager.idleRequest();
      DriverStationSim.setEnabled(false);
      robot.tick();
    }
  }
}
