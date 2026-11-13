package frc.robot.turret;

import static org.assertj.core.api.Assertions.assertThat;

import frc.robot.util.AimParameterUtil.AimingParameters;
import org.junit.jupiter.api.Test;

final class TurretTest {
  @Test
  void aimingParametersValidateCurrentTargetInvalidSolutionsAndSoftLimits() {
    var onTarget = new AimingParameters(175.0, 4.0, 2.0, -1.0, 170.0, true);
    assertThat(Turret.isAtGoal(TurretState.SCORE, TurretState.SCORE, -185.0, onTarget)).isTrue();
    assertThat(Turret.isAtGoal(TurretState.SCORE, TurretState.SCORE, -180.0, onTarget)).isFalse();

    var invalid = new AimingParameters(-180.0, 4.0, 2.0, 0.0, -180.0, false);
    assertThat(Turret.isAtGoal(TurretState.SCORE, TurretState.SCORE, -180.0, invalid)).isFalse();

    // Pinned at MAX_ANGLE (-90 deg) when the compensated target is out of travel (0 deg) is not
    // ready.
    var unreachable = new AimingParameters(0.0, 4.0, 2.0, 0.0, 0.0, true);
    assertThat(
            Turret.isAtGoal(
                TurretState.SCORE, TurretState.SCORE, TurretConfig.MAX_ANGLE, unreachable))
        .isFalse();
  }

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
  void homingStuckAndIdleTurretNeverReportReady() {
    assertThat(Turret.isAtGoal(TurretState.HOMING, 10.0, 10.0, 1.0)).isFalse();
    assertThat(Turret.isAtGoal(TurretState.STUCK, 10.0, 10.0, 1.0)).isFalse();
    assertThat(Turret.isAtGoal(TurretState.IDLE_SCORE, 10.0, 10.0, 1.0)).isFalse();
    assertThat(Turret.isAtGoal(TurretState.IDLE_FEED, 10.0, 10.0, 1.0)).isFalse();
    assertThat(Turret.isAtGoal(TurretState.HOMING, 10.0, 10.0, 1.0, 10.0)).isFalse();
    assertThat(Turret.isAtGoal(TurretState.STUCK, 10.0, 10.0, 1.0, 10.0)).isFalse();
    assertThat(Turret.isAtGoal(TurretState.IDLE_SCORE, 10.0, 10.0, 1.0, 10.0)).isFalse();
    assertThat(Turret.isAtGoal(TurretState.IDLE_FEED, 10.0, 10.0, 1.0, 10.0)).isFalse();
  }

  @Test
  void normalToleranceControlsReadiness() {
    assertThat(Turret.isAtGoal(TurretState.SCORE, -100.0, -101.0, 2.0)).isTrue();
    assertThat(Turret.isAtGoal(TurretState.SCORE, -100.0, -103.0, 2.0)).isFalse();
    assertThat(Turret.isAtGoal(TurretState.FEED, -100.0, -101.0, 2.0)).isTrue();
    assertThat(Turret.isAtGoal(TurretState.FEED, -100.0, -103.0, 2.0)).isFalse();
  }

  @Test
  void upcomingAngleMustUseCompatibleUnwrap() {
    assertThat(Turret.isAtGoal(TurretState.SCORE, -10.0, -10.0, 2.0, -15.0)).isTrue();
    assertThat(Turret.isAtGoal(TurretState.SCORE, -10.0, -10.0, 2.0, -200.0)).isFalse();
  }
}
