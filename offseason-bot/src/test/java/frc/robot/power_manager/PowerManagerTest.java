package frc.robot.power_manager;

import static org.assertj.core.api.Assertions.assertThat;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Robot;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

final class PowerManagerTest {
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

  @AfterAll
  static void tearDownHal() {
    robot.robotManager.hardware.towerSensor.close();
  }

  private PowerManager powerManager;

  @Test
  void beastModeProvidesMaxHopperThroughput() {
    assertThat(PowerManagerState.BEAST_MODE.feederSupplyCurrent).isEqualTo(30.0);
    assertThat(PowerManagerState.BEAST_MODE.conveyorSupplyCurrent).isEqualTo(30.0);
    assertThat(PowerManagerState.BEAST_MODE.funnelerSupplyCurrent).isEqualTo(30.0);
  }

  @Test
  void currentLimitsArePositiveForAllStates() {
    for (PowerManagerState state : PowerManagerState.values()) {
      assertThat(state.shooterSupplyCurrent).isPositive();
      assertThat(state.intakeSupplyCurrent).isPositive();
      assertThat(state.deploySupplyCurrent).isPositive();
      assertThat(state.shooterHoodSupplyCurrent).isPositive();
      assertThat(state.feederSupplyCurrent).isPositive();
      assertThat(state.conveyorSupplyCurrent).isPositive();
      assertThat(state.funnelerSupplyCurrent).isPositive();
      assertThat(state.swerveSupplyCurrent).isPositive();
      assertThat(state.turretSupplyCurrent).isPositive();
    }
  }

  @Test
  void feedingLimitsPreserveSwerveAndHopperThroughput() {
    // Issue #111 raised swerve supply current to 30A in FEEDING
    assertThat(PowerManagerState.FEEDING.swerveSupplyCurrent).isEqualTo(30.0);
    // Hopper feeder, conveyor, and funneler should be 30A for high throughput
    assertThat(PowerManagerState.FEEDING.feederSupplyCurrent).isEqualTo(30.0);
    assertThat(PowerManagerState.FEEDING.conveyorSupplyCurrent).isEqualTo(30.0);
    assertThat(PowerManagerState.FEEDING.funnelerSupplyCurrent).isEqualTo(30.0);
  }

  @Test
  void initialStateIsIdle() {
    assertThat(powerManager.getState()).isEqualTo(PowerManagerState.IDLE);
  }

  @BeforeEach
  void setUp() {
    powerManager = robot.robotManager.powerManager;
    powerManager.idleRequest();
  }

  @EnumSource(PowerManagerState.class)
  @ParameterizedTest
  void transitionsUpdatePowerManagerState(PowerManagerState state) {
    if (state == PowerManagerState.IDLE) {
      powerManager.scoringRequest();
    } else {
      powerManager.idleRequest();
    }

    switch (state) {
      case AUTO_FIRST_SEGMENT -> powerManager.firstAutoSegmentRequest();
      case PRIORITIZE_INTAKE -> powerManager.prioritizeIntakeRequest();
      case IDLE -> powerManager.idleRequest();
      case SCORING -> powerManager.scoringRequest();
      case SCORING_FAR -> powerManager.scoringFarRequest();
      case BEAST_MODE -> powerManager.beastModeRequest();
      case FEEDING -> powerManager.feedingRequest();
      case FEEDING_FAR -> powerManager.feedingFarRequest();
      case TURBO_MODE -> powerManager.turboRequest();
    }

    assertThat(powerManager.getState()).isEqualTo(state);
  }

  @Test
  void turboModePrioritizesSwerve() {
    assertThat(PowerManagerState.TURBO_MODE.swerveSupplyCurrent).isEqualTo(70.0);
  }
}
