package frc.robot.shooter;

import static org.assertj.core.api.Assertions.assertThat;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
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
    var request = (VelocityTorqueCurrentFOC) hardware.shooterRightMotor.getAppliedControl();
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
      var shooter = new Shooter(hardware.shooterLeftMotor, hardware.shooterRightMotor);
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
  void leftMotorFollowsRightMotorOpposedThroughScoringAndIdle() {
    var shooter = new Shooter(hardware.shooterLeftMotor, hardware.shooterRightMotor);
    shooter.robotPeriodic();
    shooter.scoreRequest(3.0);
    shooter.robotPeriodic();
    assertThat(hardware.shooterRightMotor.getAppliedControl())
        .isInstanceOf(VelocityTorqueCurrentFOC.class);
    var follower = (Follower) hardware.shooterLeftMotor.getAppliedControl();
    assertThat(follower.LeaderID).isEqualTo(hardware.shooterRightMotor.getDeviceID());
    assertThat(follower.MotorAlignment).isEqualTo(MotorAlignmentValue.Opposed);

    shooter.idleRequest();
    shooter.robotPeriodic();
    assertThat(((VelocityTorqueCurrentFOC) hardware.shooterRightMotor.getAppliedControl()).Velocity)
        .isEqualTo(ShooterConfig.IDLE_RPM / 60.0);
    assertThat(hardware.shooterLeftMotor.getAppliedControl()).isInstanceOf(Follower.class);
  }

  @Test
  void startupAndEveryTransitionRequireActualReadiness() {
    SimHooks.pauseTiming();
    try {
      var shooter = new Shooter(hardware.shooterLeftMotor, hardware.shooterRightMotor);
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
