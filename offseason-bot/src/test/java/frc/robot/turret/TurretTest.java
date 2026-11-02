package frc.robot.turret;

import static org.assertj.core.api.Assertions.assertThat;

import org.junit.jupiter.api.Test;

final class TurretTest {
  @Test
  void expectedStateValidatesAgainstStaleRequests() {
    double angle = -100.0;
    // When state is SCORE, asking for FEED returns false
    assertThat(Turret.isAtGoal(TurretState.SCORE, TurretState.FEED, angle, angle, 2.0)).isFalse();
    // When state is FEED, asking for SCORE returns false
    assertThat(Turret.isAtGoal(TurretState.FEED, TurretState.SCORE, angle, angle, 2.0)).isFalse();
    // When state is IDLE_SCORE or IDLE_FEED, asking for SCORE or FEED returns false
    assertThat(Turret.isAtGoal(TurretState.IDLE_SCORE, TurretState.SCORE, angle, angle, 2.0))
        .isFalse();
    assertThat(Turret.isAtGoal(TurretState.IDLE_FEED, TurretState.FEED, angle, angle, 2.0))
        .isFalse();

    // Matching states within tolerance succeed
    assertThat(Turret.isAtGoal(TurretState.SCORE, TurretState.SCORE, angle, angle, 2.0)).isTrue();
    assertThat(Turret.isAtGoal(TurretState.FEED, TurretState.FEED, angle, angle, 2.0)).isTrue();

    // Matching with upcoming angle check
    assertThat(Turret.isAtGoal(TurretState.SCORE, TurretState.SCORE, angle, angle, 2.0, angle))
        .isTrue();
    assertThat(Turret.isAtGoal(TurretState.SCORE, TurretState.FEED, angle, angle, 2.0, angle))
        .isFalse();
  }

  @Test
  void normalToleranceControlsReadiness() {
    assertThat(Turret.isAtGoal(TurretState.SCORE, -100.0, -101.0, 2.0)).isTrue();
    assertThat(Turret.isAtGoal(TurretState.SCORE, -100.0, -103.0, 2.0)).isFalse();
    assertThat(Turret.isAtGoal(TurretState.FEED, -100.0, -101.0, 2.0)).isTrue();
    assertThat(Turret.isAtGoal(TurretState.FEED, -100.0, -103.0, 2.0)).isFalse();
  }

  @Test
  void unhomedStuckAndIdleTurretNeverReportReady() {
    assertThat(Turret.isAtGoal(TurretState.UNHOMED, 10.0, 10.0, 1.0)).isFalse();
    assertThat(Turret.isAtGoal(TurretState.STUCK, 10.0, 10.0, 1.0)).isFalse();
    assertThat(Turret.isAtGoal(TurretState.IDLE_SCORE, 10.0, 10.0, 1.0)).isFalse();
    assertThat(Turret.isAtGoal(TurretState.IDLE_FEED, 10.0, 10.0, 1.0)).isFalse();
    assertThat(Turret.isAtGoal(TurretState.UNHOMED, 10.0, 10.0, 1.0, 10.0)).isFalse();
    assertThat(Turret.isAtGoal(TurretState.STUCK, 10.0, 10.0, 1.0, 10.0)).isFalse();
    assertThat(Turret.isAtGoal(TurretState.IDLE_SCORE, 10.0, 10.0, 1.0, 10.0)).isFalse();
    assertThat(Turret.isAtGoal(TurretState.IDLE_FEED, 10.0, 10.0, 1.0, 10.0)).isFalse();
  }

  @Test
  void upcomingAngleMustUseCompatibleUnwrap() {
    assertThat(Turret.isAtGoal(TurretState.SCORE, -10.0, -10.0, 2.0, -15.0)).isTrue();
    assertThat(Turret.isAtGoal(TurretState.SCORE, -10.0, -10.0, 2.0, -200.0)).isFalse();
  }
}
