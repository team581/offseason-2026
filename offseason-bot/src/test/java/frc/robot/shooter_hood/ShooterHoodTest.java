package frc.robot.shooter_hood;

import static org.assertj.core.api.Assertions.assertThat;

import org.junit.jupiter.api.Test;

final class ShooterHoodTest {
  @Test
  void expectedStateValidatesAgainstStaleRequests() {
    double angle = 25.0;
    // When state is SCORING, asking for FEEDING returns false
    assertThat(
            ShooterHood.isAtGoal(ShooterHoodState.SCORING, ShooterHoodState.FEEDING, angle, angle))
        .isFalse();
    // When state is FEEDING, asking for SCORING returns false
    assertThat(
            ShooterHood.isAtGoal(ShooterHoodState.FEEDING, ShooterHoodState.SCORING, angle, angle))
        .isFalse();
    // When state is IDLE, asking for SCORING or FEEDING returns false
    assertThat(ShooterHood.isAtGoal(ShooterHoodState.IDLE, ShooterHoodState.SCORING, angle, angle))
        .isFalse();
    assertThat(ShooterHood.isAtGoal(ShooterHoodState.IDLE, ShooterHoodState.FEEDING, angle, angle))
        .isFalse();

    // Matching states succeed
    assertThat(
            ShooterHood.isAtGoal(ShooterHoodState.SCORING, ShooterHoodState.SCORING, angle, angle))
        .isTrue();
    assertThat(
            ShooterHood.isAtGoal(ShooterHoodState.FEEDING, ShooterHoodState.FEEDING, angle, angle))
        .isTrue();
  }

  @Test
  void feedingUsesFeedingTolerance() {
    double goal = 35.0;
    assertThat(ShooterHood.isAtGoal(ShooterHoodState.FEEDING, goal, goal)).isTrue();
    assertThat(
            ShooterHood.isAtGoal(
                ShooterHoodState.FEEDING, goal, goal + ShooterHoodConfig.FEEDING_TOLERANCE - 0.1))
        .isTrue();
    assertThat(
            ShooterHood.isAtGoal(
                ShooterHoodState.FEEDING, goal, goal + ShooterHoodConfig.FEEDING_TOLERANCE + 0.1))
        .isFalse();
  }

  @Test
  void goalClampsWithinSafeOperatingRange() {
    double tooLow = ShooterHoodConfig.MIN_ANGLE - 10.0;
    double tooHigh = ShooterHoodConfig.MAX_ANGLE + 10.0;

    assertThat(ShooterHood.clamp(tooLow)).isEqualTo(ShooterHoodConfig.MIN_ANGLE);
    assertThat(ShooterHood.clamp(tooHigh)).isEqualTo(ShooterHoodConfig.MAX_ANGLE);

    // Goal angle outside safe bounds clamps so that robot at MIN_ANGLE is at goal
    assertThat(ShooterHood.isAtGoal(ShooterHoodState.SCORING, tooLow, ShooterHoodConfig.MIN_ANGLE))
        .isTrue();
  }

  @Test
  void scoringUsesScoringTolerance() {
    double goal = 25.0;
    assertThat(ShooterHood.isAtGoal(ShooterHoodState.SCORING, goal, goal)).isTrue();
    assertThat(
            ShooterHood.isAtGoal(
                ShooterHoodState.SCORING, goal, goal + ShooterHoodConfig.TOLERANCE - 0.1))
        .isTrue();
    assertThat(
            ShooterHood.isAtGoal(
                ShooterHoodState.SCORING, goal, goal + ShooterHoodConfig.TOLERANCE + 0.1))
        .isFalse();
    assertThat(
            ShooterHood.isAtGoal(
                ShooterHoodState.SCORING, goal, goal - ShooterHoodConfig.TOLERANCE + 0.1))
        .isTrue();
    assertThat(
            ShooterHood.isAtGoal(
                ShooterHoodState.SCORING, goal, goal - ShooterHoodConfig.TOLERANCE - 0.1))
        .isFalse();
  }

  @Test
  void unhomedHomingAndIdleNeverReportAtGoal() {
    assertThat(ShooterHood.isAtGoal(ShooterHoodState.UNHOMED, 25.0, 25.0)).isFalse();
    assertThat(ShooterHood.isAtGoal(ShooterHoodState.HOMING, 25.0, 25.0)).isFalse();
    assertThat(
            ShooterHood.isAtGoal(
                ShooterHoodState.IDLE, ShooterHoodConfig.IDLE_ANGLE, ShooterHoodConfig.IDLE_ANGLE))
        .isFalse();
  }
}
