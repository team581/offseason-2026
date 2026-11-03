package frc.robot.shooter;

import static org.assertj.core.api.Assertions.assertThat;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.Hardware;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

final class ShooterReadinessTest {
  private static Hardware hardware;

  @BeforeAll
  static void initializeHal() {
    HAL.initialize(500, 0);
    hardware = new Hardware();
  }

  @Test
  void startupAndEveryTransitionRequireActualReadiness() {
    SimHooks.pauseTiming();
    try {
      var shooter =
          new Shooter(
              hardware.shooterTopLeftMotor,
              hardware.shooterTopRightMotor,
              hardware.shooterBottomLeftMotor,
              hardware.shooterBottomRightMotor);
      for (var state : ShooterState.values()) {
        switch (state) {
          case IDLE -> shooter.idleRequest();
          case PREPARE_SCORE -> shooter.prepareScoreRequest(3.0);
          case SCORE -> shooter.scoreRequest(3.0);
          case PREPARE_FEED -> shooter.prepareFeedRequest(3.0);
          case FEED -> shooter.feedRequest(3.0);
        }
        shooter.afterTransition(state);
        for (int tick = 0; tick < 5; tick++) {
          shooter.beforePeriodic();
          assertThat(shooter.atGoal()).as("raw readiness in %s", state).isFalse();
          assertThat(shooter.atGoalDebounced()).as("debounced readiness in %s", state).isFalse();
          SimHooks.stepTimingAsync(0.02);
        }
      }
    } finally {
      SimHooks.resumeTiming();
    }
  }
}
