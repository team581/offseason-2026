package frc.robot.turret;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

final class TurretTest {
  @Test
  void normalToleranceControlsReadiness() {
    assertTrue(Turret.isAtGoal(TurretState.SCORE, -100.0, -101.0, 2.0));
    assertFalse(Turret.isAtGoal(TurretState.SCORE, -100.0, -103.0, 2.0));
  }

  @Test
  void unhomedAndStuckTurretNeverReportReady() {
    assertFalse(Turret.isAtGoal(TurretState.UNHOMED, 10.0, 10.0, 1.0));
    assertFalse(Turret.isAtGoal(TurretState.STUCK, 10.0, 10.0, 1.0));
    assertFalse(Turret.isAtGoal(TurretState.UNHOMED, 10.0, 10.0, 1.0, 10.0));
    assertFalse(Turret.isAtGoal(TurretState.STUCK, 10.0, 10.0, 1.0, 10.0));
  }

  @Test
  void upcomingAngleMustUseCompatibleUnwrap() {
    assertTrue(Turret.isAtGoal(TurretState.SCORE, -10.0, -10.0, 2.0, -15.0));
    assertFalse(Turret.isAtGoal(TurretState.SCORE, -10.0, -10.0, 2.0, -200.0));
  }
}
