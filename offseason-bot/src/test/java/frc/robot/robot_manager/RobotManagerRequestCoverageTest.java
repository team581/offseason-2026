package frc.robot.robot_manager;

import static org.assertj.core.api.Assertions.assertThat;

import com.team581.util.FieldUtil;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Robot;
import frc.robot.hub_activity.HubActivity;
import frc.robot.power_manager.PowerManager;
import frc.robot.power_manager.PowerManagerState;
import frc.robot.robot_manager.hopper_manager.HopperManager;
import frc.robot.robot_manager.hopper_manager.HopperState;
import frc.robot.shooter.ShooterState;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

final class RobotManagerRequestCoverageTest {
  private static Robot robot;

  private static void assertExpectedPowerManagerState(RobotState state, PowerManagerState actual) {
    switch (state) {
      case IDLE, UNJAM -> assertThat(actual).isEqualTo(PowerManagerState.IDLE);
      case PREPARE_SCORE, SCORE, PREPARE_FALLBACK_SCORE, FALLBACK_SCORE, WARMUP_SCORE ->
          assertThat(actual).isEqualTo(PowerManagerState.SCORING);
      case PREPARE_FORCE_SCORE, FORCE_SCORE ->
          assertThat(actual).isEqualTo(PowerManagerState.SCORING);
      case PREPARE_FEED, FEED, PREPARE_FALLBACK_FEED, FALLBACK_FEED, WARMUP_FEED ->
          assertThat(actual).isIn(PowerManagerState.FEEDING, PowerManagerState.FEEDING_FAR);
    }
  }

  private static HopperState expectedHopperState(RobotState state) {
    return switch (state) {
      case IDLE,
          WARMUP_SCORE,
          WARMUP_FEED,
          PREPARE_SCORE,
          PREPARE_FORCE_SCORE,
          PREPARE_FALLBACK_SCORE,
          PREPARE_FEED,
          PREPARE_FALLBACK_FEED ->
          HopperState.IDLE_DEPLOYED;
      case SCORE, FORCE_SCORE, FALLBACK_SCORE -> HopperState.SCORE;
      case FEED, FALLBACK_FEED -> HopperState.FEED;
      case UNJAM -> HopperState.UNJAMMING;
    };
  }

  private static void setLocalizationHealthy(boolean healthy) throws Exception {
    var healthField = Robot.class.getDeclaredField("health");
    healthField.setAccessible(true);
    Object health = healthField.get(robot);
    var readyField = frc.robot.health.HealthManager.class.getDeclaredField("localizationHealthy");
    readyField.setAccessible(true);
    readyField.set(health, healthy);
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

  private HopperManager hopperManager;

  private PowerManager powerManager;

  private HubActivity hubActivity;

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
  void autoKeepsNormalShotsWithoutLocalization() throws Exception {
    setLocalizationHealthy(false);
    DriverStationSim.setAutonomous(true);
    DriverStationSim.notifyNewData();
    try {
      robotManager.idleRequest();
      robotManager.prepareScoreRequest();
      assertThat(robotManager.getState()).isEqualTo(RobotState.PREPARE_SCORE);
      assertThat(robotManager.getNextState(RobotState.SCORE))
          .isNotEqualTo(RobotState.PREPARE_FALLBACK_SCORE);
    } finally {
      DriverStationSim.setAutonomous(false);
      DriverStationSim.notifyNewData();
      setLocalizationHealthy(true);
    }
  }

  @EnumSource(RobotState.class)
  @ParameterizedTest
  void entryAndHeldStateReplaceStaleHopperManagerRequest(RobotState state) {
    var expected = expectedHopperState(state);
    for (boolean entry : new boolean[] {true, false}) {
      // Set a stale state on hopper
      if (expected == HopperState.IDLE_DEPLOYED) {
        hopperManager.unjamRequest();
      } else {
        hopperManager.idleRequest();
      }
      if (entry) {
        robotManager.afterTransition(state);
      } else {
        robotManager.whileInState(state);
      }
      assertThat(hopperManager.getState()).as("%s entry=%s", state, entry).isEqualTo(expected);
    }
  }

  @EnumSource(RobotState.class)
  @ParameterizedTest
  void entryAndHeldStateReplaceStalePowerManagerRequest(RobotState state) {
    for (boolean entry : new boolean[] {true, false}) {
      // In IDLE whileInState, powerManager is intentionally excluded to preserve overrides
      if (!entry && state == RobotState.IDLE) {
        continue;
      }
      powerManager.prioritizeIntakeRequest();
      if (entry) {
        robotManager.afterTransition(state);
      } else {
        robotManager.whileInState(state);
      }
      assertExpectedPowerManagerState(state, powerManager.getState());
    }
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
  void losingLocalizationBlocksNormalShotsAndMovesFeedsToFallback() throws Exception {
    setLocalizationHealthy(false);
    assertThat(robotManager.getNextState(RobotState.PREPARE_SCORE))
        .isEqualTo(RobotState.PREPARE_SCORE);
    assertThat(robotManager.getNextState(RobotState.SCORE)).isEqualTo(RobotState.PREPARE_SCORE);
    assertThat(robotManager.getNextState(RobotState.PREPARE_FEED))
        .isEqualTo(RobotState.PREPARE_FALLBACK_FEED);
    assertThat(robotManager.getNextState(RobotState.FEED))
        .isEqualTo(RobotState.PREPARE_FALLBACK_FEED);
  }

  @EnumSource(
      value = RobotState.class,
      names = {
        "FALLBACK_SCORE",
        "PREPARE_FALLBACK_SCORE",
        "PREPARE_SCORE",
        "SCORE",
        "WARMUP_SCORE"
      })
  @ParameterizedTest
  void scoringStatesSelectExactPowerModeOnEntryAndWhileHeld(RobotState state) throws Exception {
    var beastModeField = HubActivity.class.getDeclaredField("shouldBeastMode");
    beastModeField.setAccessible(true);
    try {
      for (var expected :
          new PowerManagerState[] {
            PowerManagerState.SCORING, PowerManagerState.SCORING_FAR, PowerManagerState.BEAST_MODE
          }) {
        // Beast mode must take priority even when the shot also qualifies as far.
        double distance = expected == PowerManagerState.SCORING ? 2.0 : 6.0;
        robotManager.localization.resetPose(
            new Pose2d(
                FieldUtil.HUB_POSE.getTranslation().plus(new Translation2d(distance, 0.0)),
                Rotation2d.kZero));
        robotManager.beforePeriodic();
        beastModeField.setBoolean(hubActivity, expected == PowerManagerState.BEAST_MODE);

        for (boolean entry : new boolean[] {true, false}) {
          powerManager.prioritizeIntakeRequest();
          if (entry) {
            robotManager.afterTransition(state);
          } else {
            robotManager.whileInState(state);
          }
          assertThat(powerManager.getState())
              .as("%s entry=%s expected=%s", state, entry, expected)
              .isEqualTo(expected);
        }
      }
    } finally {
      beastModeField.setBoolean(hubActivity, false);
    }
  }

  @BeforeEach
  void setUp() throws Exception {
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();
    robotManager = robot.robotManager;
    hopperManager = robotManager.hopperManager;
    powerManager = robotManager.powerManager;

    var hubActivityField = RobotManager.class.getDeclaredField("hubActivity");
    hubActivityField.setAccessible(true);
    hubActivity = (HubActivity) hubActivityField.get(robotManager);
    var beastModeField = HubActivity.class.getDeclaredField("shouldBeastMode");
    beastModeField.setAccessible(true);
    beastModeField.setBoolean(hubActivity, false);

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
