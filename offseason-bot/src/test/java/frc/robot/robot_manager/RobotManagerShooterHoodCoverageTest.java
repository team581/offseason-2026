package frc.robot.robot_manager;

import static org.assertj.core.api.Assertions.assertThat;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Robot;
import frc.robot.shooter_hood.ShooterHood;
import frc.robot.shooter_hood.ShooterHoodState;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

final class RobotManagerShooterHoodCoverageTest {
  private static Robot robot;

  private static ShooterHoodState expectedHoodState(RobotState state) {
    return switch (state) {
      case IDLE, WARMUP_SCORE, WARMUP_FEED -> ShooterHoodState.IDLE;
      case PREPARE_SCORE,
          SCORE,
          PREPARE_FORCE_SCORE,
          FORCE_SCORE,
          PREPARE_FALLBACK_SCORE,
          FALLBACK_SCORE,
          UNJAM ->
          ShooterHoodState.SCORING;
      case PREPARE_FEED, FEED, PREPARE_FALLBACK_FEED, FALLBACK_FEED -> ShooterHoodState.FEEDING;
    };
  }

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
  private ShooterHood shooterHood;

  @EnumSource(RobotState.class)
  @ParameterizedTest
  void entryAndHeldStateReplaceStaleShooterHoodRequest(RobotState state) {
    var expected = expectedHoodState(state);
    for (boolean entry : new boolean[] {true, false}) {
      for (ShooterHoodState stale :
          new ShooterHoodState[] {
            ShooterHoodState.IDLE, ShooterHoodState.FEEDING, ShooterHoodState.SCORING
          }) {
        switch (stale) {
          case IDLE -> shooterHood.idleRequest();
          case FEEDING -> shooterHood.feedRequest(5.0);
          case SCORING -> shooterHood.scoreRequest(5.0);
          default -> {}
        }
        if (entry) {
          robotManager.afterTransition(state);
        } else {
          robotManager.whileInState(state);
        }
        assertThat(shooterHood.getState())
            .as("%s entry=%s stale=%s", state, entry, stale)
            .isEqualTo(expected);
      }
    }
  }

  @BeforeEach
  void setUp() throws Exception {
    robotManager = robot.robotManager;
    var hoodField = RobotManager.class.getDeclaredField("shooterHood");
    hoodField.setAccessible(true);
    shooterHood = (ShooterHood) hoodField.get(robotManager);

    var healthField = Robot.class.getDeclaredField("health");
    healthField.setAccessible(true);
    Object health = healthField.get(robot);
    var readyField = frc.robot.health.HealthManager.class.getDeclaredField("localizationHealthy");
    readyField.setAccessible(true);
    readyField.set(health, true);

    var nearTrenchField = RobotManager.class.getDeclaredField("nearTrench");
    nearTrenchField.setAccessible(true);
    nearTrenchField.set(robotManager, false);

    var isMovingField = RobotManager.class.getDeclaredField("isMoving");
    isMovingField.setAccessible(true);
    isMovingField.set(robotManager, false);

    robotManager.localization.resetPose(new Pose2d(2.0, 4.0, Rotation2d.kZero));
    shooterHood.homingRequest();
    shooterHood.simulationPeriodic();
    robotManager.beforePeriodic();
    robotManager.idleRequest();
    robotManager.periodic();
  }
}
