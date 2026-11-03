package frc.robot.robot_manager;

import static org.assertj.core.api.Assertions.assertThat;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Robot;
import frc.robot.power_manager.PowerManager;
import frc.robot.power_manager.PowerManagerState;
import frc.robot.robot_manager.hopper_manager.HopperManager;
import frc.robot.robot_manager.hopper_manager.HopperState;
import frc.robot.shooter.ShooterState;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

final class RobotManagerRequestCoverageTest {
  private static Robot robot;

  @BeforeAll
  static void initializeHal() {
    HAL.initialize(500, 0);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.setTest(false);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    robot = new Robot();
  }

  private RobotManager robotManager;
  private HopperManager hopperManager;
  private PowerManager powerManager;

  @Test
  void afterTransitionImmediatelyCommandsHopperManager() {
    // Transitioning from IDLE to UNJAM immediately requests unjam on hopper
    robotManager.unjamRequest();
    assertThat(robotManager.getState()).isEqualTo(RobotState.UNJAM);
    assertThat(hopperManager.getState()).isEqualTo(HopperState.UNJAMMING);

    // Transitioning back to IDLE immediately requests idle on hopper
    robotManager.idleRequest();
    assertThat(robotManager.getState()).isEqualTo(RobotState.IDLE);
    assertThat(hopperManager.getState()).isEqualTo(HopperState.IDLE_DEPLOYED);

    // Transitioning to PREPARE_FORCE_SCORE immediately requests idle on hopper
    robotManager.forceScoreRequest();
    assertThat(robotManager.getState()).isEqualTo(RobotState.PREPARE_FORCE_SCORE);
    assertThat(hopperManager.getState()).isEqualTo(HopperState.IDLE_DEPLOYED);

    // WARMUP_SCORE requests idle on hopper
    robotManager.warmupScoreRequest();
    assertThat(robotManager.getState()).isEqualTo(RobotState.WARMUP_SCORE);
    assertThat(hopperManager.getState()).isEqualTo(HopperState.IDLE_DEPLOYED);

    // WARMUP_FEED requests idle on hopper
    robotManager.warmupFeedRequest();
    assertThat(robotManager.getState()).isEqualTo(RobotState.WARMUP_FEED);
    assertThat(hopperManager.getState()).isEqualTo(HopperState.IDLE_DEPLOYED);
  }

  @Test
  void idleWhileInStatePreservesPowerManagerOverride() {
    robotManager.idleRequest();
    robotManager.beforePeriodic();
    robotManager.periodic();
    assertThat(robotManager.getState()).isEqualTo(RobotState.IDLE);

    // Driver/operator can request temporary intake prioritization
    powerManager.prioritizeIntakeRequest();
    assertThat(powerManager.getState()).isEqualTo(PowerManagerState.PRIORITIZE_INTAKE);

    // whileInState in IDLE must not overwrite the temporary override
    robotManager.beforePeriodic();
    robotManager.periodic();
    assertThat(powerManager.getState()).isEqualTo(PowerManagerState.PRIORITIZE_INTAKE);
  }

  @Test
  void losingLocalizationMovesNormalShotsBackToFallbackPreparation() throws Exception {
    var healthField = Robot.class.getDeclaredField("health");
    healthField.setAccessible(true);
    Object health = healthField.get(robot);
    var readyField = frc.robot.health.HealthManager.class.getDeclaredField("localizationHealthy");
    readyField.setAccessible(true);
    readyField.set(health, false);
    assertThat(robotManager.getNextState(RobotState.PREPARE_SCORE))
        .isEqualTo(RobotState.PREPARE_FALLBACK_SCORE);
    assertThat(robotManager.getNextState(RobotState.SCORE))
        .isEqualTo(RobotState.PREPARE_FALLBACK_SCORE);
    assertThat(robotManager.getNextState(RobotState.PREPARE_FEED))
        .isEqualTo(RobotState.PREPARE_FALLBACK_FEED);
    assertThat(robotManager.getNextState(RobotState.FEED))
        .isEqualTo(RobotState.PREPARE_FALLBACK_FEED);
  }

  @BeforeEach
  void setUp() throws Exception {
    robotManager = robot.robotManager;
    hopperManager = robotManager.hopperManager;
    powerManager = robotManager.powerManager;

    // Mark localization healthy so robot states resolve consistently
    var healthField = Robot.class.getDeclaredField("health");
    healthField.setAccessible(true);
    Object health = healthField.get(robot);
    var field = frc.robot.health.HealthManager.class.getDeclaredField("localizationHealthy");
    field.setAccessible(true);
    field.set(health, true);

    robotManager.localization.resetPose(new Pose2d(2.0, 4.0, Rotation2d.kZero));
    robotManager.beforePeriodic();
    robotManager.idleRequest();
    robotManager.periodic();
  }

  @Test
  void unjamStateMaintainsRequestsInWhileInState() {
    robotManager.unjamRequest();
    assertThat(robotManager.getState()).isEqualTo(RobotState.UNJAM);
    assertThat(hopperManager.getState()).isEqualTo(HopperState.UNJAMMING);
    assertThat(robotManager.shooter.getState()).isEqualTo(ShooterState.SCORE);

    // A stale subsystem request must be replaced while unjam remains held.
    robotManager.shooter.idleRequest();

    // Verify whileInState continues commanding unjam on hopperManager and idle on powerManager
    robotManager.beforePeriodic();
    robotManager.periodic();
    assertThat(robotManager.getState()).isEqualTo(RobotState.UNJAM);
    assertThat(hopperManager.getState()).isEqualTo(HopperState.UNJAMMING);
    assertThat(powerManager.getState()).isEqualTo(PowerManagerState.IDLE);
    assertThat(robotManager.shooter.getState()).isEqualTo(ShooterState.SCORE);

    robotManager.idleRequest();
    assertThat(robotManager.shooter.getState()).isEqualTo(ShooterState.IDLE);
    assertThat(hopperManager.getState()).isEqualTo(HopperState.IDLE_DEPLOYED);
  }

  @Test
  void warmupAndFallbackScoreMaintainPowerManagerRequests() {
    robotManager.warmupScoreRequest();
    assertThat(robotManager.getState()).isEqualTo(RobotState.WARMUP_SCORE);

    robotManager.beforePeriodic();
    robotManager.periodic();
    assertThat(powerManager.getState())
        .isIn(
            PowerManagerState.SCORING, PowerManagerState.SCORING_FAR, PowerManagerState.BEAST_MODE);
  }
}
