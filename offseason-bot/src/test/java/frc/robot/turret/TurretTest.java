package frc.robot.turret;

import static org.assertj.core.api.Assertions.assertThat;

import org.junit.jupiter.api.Test;

final class TurretTest {
  @Test
  void normalToleranceControlsReadiness() {
    assertThat(Turret.isAtGoal(TurretState.SCORE, -100.0, -101.0, 2.0)).isTrue();
    assertThat(Turret.isAtGoal(TurretState.SCORE, -100.0, -103.0, 2.0)).isFalse();
  }

  @Test
  void unhomedAndStuckTurretNeverReportReady() {
    assertThat(Turret.isAtGoal(TurretState.UNHOMED, 10.0, 10.0, 1.0)).isFalse();
    assertThat(Turret.isAtGoal(TurretState.STUCK, 10.0, 10.0, 1.0)).isFalse();
    assertThat(Turret.isAtGoal(TurretState.UNHOMED, 10.0, 10.0, 1.0, 10.0)).isFalse();
    assertThat(Turret.isAtGoal(TurretState.STUCK, 10.0, 10.0, 1.0, 10.0)).isFalse();
  }

  @Test
  void upcomingAngleMustUseCompatibleUnwrap() {
    assertThat(Turret.isAtGoal(TurretState.SCORE, -10.0, -10.0, 2.0, -15.0)).isTrue();
    assertThat(Turret.isAtGoal(TurretState.SCORE, -10.0, -10.0, 2.0, -200.0)).isFalse();
  }
}
