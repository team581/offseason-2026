package frc.robot.turret;

import static org.assertj.core.api.Assertions.assertThat;

import frc.robot.util.AimParameterUtil.AimingParameters;
import org.junit.jupiter.api.Test;

final class TurretTest {
  @Test
  void anAccurateCurrentShotDoesNotPauseForAHypotheticalFutureUnwrap() {
    var parameters = new AimingParameters(270, 8, 2, 0, -30);
    assertThat(Turret.isAtGoal(TurretState.FEED, 269, parameters)).isTrue();
    assertThat(Turret.isAtGoal(TurretState.SCORE, 267, parameters)).isFalse();
    assertThat(Turret.isAtGoal(TurretState.FEED, -90, parameters)).isTrue();
    assertThat(Turret.isAtGoal(TurretState.UNHOMED, 270, parameters)).isFalse();
    assertThat(Turret.isAtGoal(TurretState.FEED, -319, new AimingParameters(45, 8, 2, 0, 45)))
        .isFalse();
  }

  @Test
  void equivalentBeamAngleOnAnotherUnwrapBranchIsNotReady() {
    assertThat(Turret.isAtGoal(TurretState.FEED, 60, -300, 2, 60)).isFalse();
  }

  @Test
  void largePredictedMotionDoesNotRejectAnAccurateTurretWithRoomToMove() {
    assertThat(Turret.isAtGoal(TurretState.FEED, -120, -121, 2, -20)).isTrue();
    assertThat(Turret.isAtGoal(TurretState.SCORE, 120, 121, 2, 20)).isTrue();
    assertThat(Turret.isAtGoal(TurretState.FEED, -120, -123, 2, -20)).isFalse();
  }

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
    assertThat(Turret.isAtGoal(TurretState.SCORE, -300.0, -300.0, 2.0, 30.0)).isFalse();
  }
}
