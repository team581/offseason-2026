package frc.robot.shooter;

import static org.assertj.core.api.Assertions.assertThat;

import org.junit.jupiter.api.Test;

final class ShooterTest {
  @Test
  void expectedStateMustMatchCurrentState() {
    double targetScoreRpm = 2000.0;
    double targetFeedRpm = 2300.0;

    // In PREPARE_SCORE, matching scoring RPM is at goal for PREPARE_SCORE but not PREPARE_FEED
    assertThat(
            Shooter.isAtGoal(
                ShooterState.PREPARE_SCORE,
                ShooterState.PREPARE_SCORE,
                targetScoreRpm,
                targetScoreRpm,
                targetFeedRpm))
        .isTrue();

    assertThat(
            Shooter.isAtGoal(
                ShooterState.PREPARE_SCORE,
                ShooterState.PREPARE_FEED,
                targetScoreRpm,
                targetScoreRpm,
                targetFeedRpm))
        .isFalse();

    // In PREPARE_FEED, matching feeding RPM is at goal for PREPARE_FEED but not PREPARE_SCORE
    assertThat(
            Shooter.isAtGoal(
                ShooterState.PREPARE_FEED,
                ShooterState.PREPARE_FEED,
                targetFeedRpm,
                targetScoreRpm,
                targetFeedRpm))
        .isTrue();

    assertThat(
            Shooter.isAtGoal(
                ShooterState.PREPARE_FEED,
                ShooterState.PREPARE_SCORE,
                targetFeedRpm,
                targetScoreRpm,
                targetFeedRpm))
        .isFalse();
  }

  @Test
  void idleNeverReportsAtGoal() {
    assertThat(Shooter.isAtGoal(ShooterState.IDLE, 750.0, 1500.0, 2000.0)).isFalse();
    assertThat(Shooter.isAtGoal(ShooterState.IDLE, ShooterState.IDLE, 750.0, 1500.0, 2000.0))
        .isFalse();
  }

  @Test
  void prepareFeedUsesFeedingTolerance() {
    double targetRpm = 2200.0;
    double nearRpm = targetRpm + ShooterConfig.RPM_TOLERANCE_FEEDING - 50.0;
    double farRpm = targetRpm + ShooterConfig.RPM_TOLERANCE_FEEDING + 50.0;

    assertThat(Shooter.isAtGoal(ShooterState.PREPARE_FEED, nearRpm, 1500.0, targetRpm)).isTrue();
    assertThat(Shooter.isAtGoal(ShooterState.PREPARE_FEED, farRpm, 1500.0, targetRpm)).isFalse();
    assertThat(Shooter.isAtGoal(ShooterState.FEED, nearRpm, 1500.0, targetRpm)).isTrue();
  }

  @Test
  void prepareScoreUsesTighterToleranceThanActiveScoring() {
    double targetRpm = 2000.0;
    double nearRpm = targetRpm + ShooterConfig.RPM_TOLERANCE - 5.0;
    double mediumRpm = targetRpm + ShooterConfig.RPM_TOLERANCE + 50.0;

    assertThat(Shooter.isAtGoal(ShooterState.PREPARE_SCORE, nearRpm, targetRpm, 1000.0)).isTrue();
    assertThat(Shooter.isAtGoal(ShooterState.PREPARE_SCORE, mediumRpm, targetRpm, 1000.0))
        .isFalse();

    assertThat(Shooter.isAtGoal(ShooterState.SCORE, mediumRpm, targetRpm, 1000.0)).isTrue();
  }
}
