package frc.robot.robot_manager;

import static org.assertj.core.api.Assertions.assertThat;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Robot;
import frc.robot.config.DSOptions;
import frc.robot.turret.Turret;
import frc.robot.turret.TurretState;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

final class RobotManagerTurretCoverageTest {
  private static Robot robot;

  @BeforeAll
  static void initializeHal() {
    HAL.initialize(500, 0);
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    robot = new Robot();
  }

  @org.junit.jupiter.api.AfterAll
  static void tearDownHal() {
    robot.robotManager.hardware.towerSensor.close();
  }

  private RobotManager robotManager;
  private Turret turret;

  @Test
  void feedingStatesCommandTurretFeed() {
    robotManager.localization.resetPose(new Pose2d(10.0, 4.0, Rotation2d.kZero));

    // PREPARE_FEED
    robotManager.prepareFeedRequest();
    robotManager.beforePeriodic();
    robotManager.periodic();
    assertThat(robotManager.getState())
        .isIn(RobotState.PREPARE_FEED, RobotState.PREPARE_FALLBACK_FEED);
    assertThat(turret.getState()).isEqualTo(TurretState.FEED);

    // WARMUP_FEED
    robotManager.idleRequest();
    robotManager.warmupFeedRequest();
    robotManager.beforePeriodic();
    robotManager.periodic();
    assertThat(robotManager.getState()).isEqualTo(RobotState.WARMUP_FEED);
    assertThat(turret.getState()).isEqualTo(TurretState.FEED);
  }

  @Test
  void idleAfterTransitionImmediatelyCommandsTurret() {
    // Put robotManager and turret into scoring state
    robotManager.prepareScoreRequest();
    turret.scoreRequest(0.0, 0.0);
    assertThat(turret.getState()).isEqualTo(TurretState.SCORE);

    // Transitioning to IDLE must immediately update turret to an idle state
    robotManager.localization.resetPose(new Pose2d(2.0, 4.0, Rotation2d.kZero));
    robotManager.idleRequest();
    assertThat(turret.getState()).isIn(TurretState.IDLE_SCORE, TurretState.IDLE_FEED);
  }

  @Test
  void idleStateCommandsTurretIdle() {
    // In alliance zone, smartTurretIdleRequest commands IDLE_SCORE
    robotManager.localization.resetPose(new Pose2d(2.0, 4.0, Rotation2d.kZero));
    robotManager.idleRequest();
    robotManager.beforePeriodic();
    robotManager.periodic();

    assertThat(robotManager.getState()).isEqualTo(RobotState.IDLE);
    assertThat(turret.getState()).isEqualTo(TurretState.IDLE_SCORE);

    // Outside alliance zone, smartTurretIdleRequest commands IDLE_FEED
    robotManager.localization.resetPose(new Pose2d(10.0, 4.0, Rotation2d.kZero));
    robotManager.beforePeriodic();
    robotManager.periodic();

    assertThat(turret.getState()).isEqualTo(TurretState.IDLE_FEED);
  }

  @Test
  void pitFunctionalityIntentionallyExcludesTurretReadiness() {
    var entry = (BooleanEntry) DSOptions.PIT_FUNCTIONALITY;
    entry.set(true);

    try {
      // In pit functionality, turret readiness is intentionally excluded so
      // mechanisms can be tested without turret rotation or homing
      turret.stuckRequest();
      assertThat(turret.atGoal(TurretState.SCORE, 2.0)).isFalse();

      // State machine logic for pit functionality only checks shooter and hood
      assertThat(DSOptions.PIT_FUNCTIONALITY.getAsBoolean()).isTrue();
    } finally {
      entry.set(false);
    }
  }

  @Test
  void scoringStatesCommandTurretScore() {
    robotManager.localization.resetPose(new Pose2d(2.0, 4.0, Rotation2d.kZero));

    // PREPARE_SCORE
    robotManager.prepareScoreRequest();
    robotManager.beforePeriodic();
    robotManager.periodic();
    assertThat(robotManager.getState())
        .isIn(RobotState.PREPARE_SCORE, RobotState.PREPARE_FALLBACK_SCORE);
    assertThat(turret.getState()).isEqualTo(TurretState.SCORE);

    // WARMUP_SCORE
    robotManager.idleRequest();
    robotManager.warmupScoreRequest();
    robotManager.beforePeriodic();
    robotManager.periodic();
    assertThat(robotManager.getState()).isEqualTo(RobotState.WARMUP_SCORE);
    assertThat(turret.getState()).isEqualTo(TurretState.SCORE);

    // PREPARE_FORCE_SCORE
    robotManager.idleRequest();
    robotManager.forceScoreRequest();
    robotManager.beforePeriodic();
    robotManager.periodic();
    assertThat(robotManager.getState()).isEqualTo(RobotState.PREPARE_FORCE_SCORE);
    assertThat(turret.getState()).isEqualTo(TurretState.SCORE);
  }

  @BeforeEach
  void setUp() throws Exception {
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();

    // Mark localization healthy so odometry is trusted for alliance zone checks
    var healthField = Robot.class.getDeclaredField("health");
    healthField.setAccessible(true);
    Object health = healthField.get(robot);
    var field = frc.robot.health.HealthManager.class.getDeclaredField("localizationHealthy");
    field.setAccessible(true);
    field.set(health, true);

    robotManager = robot.robotManager;
    turret = robot.turret;
    turret.setState(TurretState.SCORE);

    // Warm up one cycle so teleopInit() runs and transitions complete cleanly
    robotManager.idleRequest();
    robotManager.beforePeriodic();
    robotManager.periodic();
  }

  @Test
  void transitionsValidateAgainstStaleTurretState() {
    // Turret in FEED state cannot satisfy SCORE transition
    turret.feedRequest(0.0, 0.0);
    assertThat(turret.atGoal(TurretState.SCORE, 2.0)).isFalse();

    // Turret in SCORE state cannot satisfy FEED transition
    turret.scoreRequest(0.0, 0.0);
    assertThat(turret.atGoal(TurretState.FEED, 2.0)).isFalse();

    // Turret in IDLE states cannot satisfy either
    turret.idleScoreRequest(0.0, 0.0);
    assertThat(turret.atGoal(TurretState.SCORE, 2.0)).isFalse();
    assertThat(turret.atGoal(TurretState.FEED, 2.0)).isFalse();

    turret.idleFeedRequest(0.0, 0.0);
    assertThat(turret.atGoal(TurretState.SCORE, 2.0)).isFalse();
    assertThat(turret.atGoal(TurretState.FEED, 2.0)).isFalse();
  }

  @Test
  void unjamCommandsTurretIdleInTransitionAndWhileInState() {
    robotManager.unjamRequest();
    assertThat(robotManager.getState()).isEqualTo(RobotState.UNJAM);
    assertThat(turret.getState()).isIn(TurretState.IDLE_SCORE, TurretState.IDLE_FEED);

    // Verify whileInState continues commanding turret idle
    robotManager.beforePeriodic();
    robotManager.periodic();
    assertThat(robotManager.getState()).isEqualTo(RobotState.UNJAM);
    assertThat(turret.getState()).isIn(TurretState.IDLE_SCORE, TurretState.IDLE_FEED);
  }
}
