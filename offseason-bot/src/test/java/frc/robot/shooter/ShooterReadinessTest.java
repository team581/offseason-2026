package frc.robot.shooter;

import static org.assertj.core.api.Assertions.assertThat;

import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.Hardware;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

final class ShooterReadinessTest {
  private static Hardware hardware;

  private static void assertFeedForward(Shooter shooter, double expected) {
    shooter.beforePeriodic();
    shooter.robotPeriodic();
    var request = (VelocityTorqueCurrentFOC) hardware.shooterTopRightMotor.getAppliedControl();
    assertThat(request.FeedForward).isEqualTo(expected);
  }

  @BeforeAll
  static void initializeHal() {
    HAL.initialize(500, 0);
    hardware = new Hardware();
  }

  @Test
  void initialFeedForwardExpiresAndClearsOutsideShootingStates() {
    SimHooks.pauseTiming();
    try {
      var shooter =
          new Shooter(
              hardware.shooterTopLeftMotor,
              hardware.shooterTopRightMotor,
              hardware.shooterBottomLeftMotor,
              hardware.shooterBottomRightMotor);
      shooter.robotPeriodic();
      for (boolean full : new boolean[] {false, true}) {
        shooter.updateHopperState(full);
        double boost =
            full
                ? ShooterConfig.FULL_HOPPER_INITIAL_FF.getAsDouble()
                : ShooterConfig.LOW_HOPPER_INITIAL_FFF.getAsDouble();
        for (var state : new ShooterState[] {ShooterState.SCORE, ShooterState.FEED}) {
          if (state == ShooterState.SCORE) {
            shooter.scoreRequest(3.0);
          } else {
            shooter.feedRequest(3.0);
          }
          assertFeedForward(shooter, boost);
          SimHooks.stepTimingAsync(0.11);
          assertFeedForward(shooter, full ? boost : 0.0);
          SimHooks.stepTimingAsync(0.60);
          assertFeedForward(shooter, 0.0);
        }
        shooter.scoreRequest(3.0);
        assertFeedForward(shooter, boost);
        shooter.prepareScoreRequest(3.0);
        assertFeedForward(shooter, 0.0);
        shooter.feedRequest(3.0);
        assertFeedForward(shooter, boost);
        shooter.prepareFeedRequest(3.0);
        assertFeedForward(shooter, 0.0);
        shooter.idleRequest();
        assertFeedForward(shooter, 0.0);
      }
    } finally {
      SimHooks.resumeTiming();
    }
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
