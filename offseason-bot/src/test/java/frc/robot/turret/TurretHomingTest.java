package frc.robot.turret;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.within;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

final class TurretHomingTest {
  @BeforeAll
  static void initializeHal() {
    HAL.initialize(500, 0);
  }

  private TurretHoming homing;
  private final List<Double> writes = new ArrayList<>();

  private boolean update(boolean disabled, boolean valid, double rotor, double angle) {
    return homing.update(
        disabled,
        valid,
        rotor,
        angle,
        position -> {
          writes.add(position);
          return true;
        });
  }

  @Test
  void debouncesInitialSeedAndContinuesCorrectingUntilEnable() {
    // Time spent booting must not count toward the first correction's debounce.
    SimHooks.stepTimingAsync(1.0);
    assertThat(update(true, true, 0.0, 0.0)).isFalse();
    SimHooks.stepTimingAsync(0.05);
    update(true, true, 0.0, 0.0);
    assertThat(writes).isEmpty();
    SimHooks.stepTimingAsync(0.06);
    assertThat(update(true, true, 0.0, 0.0)).isFalse();
    assertThat(writes).containsExactly(-0.5);

    // Moving the hand-set reference across a rotor revolution needs another correction.
    update(true, true, 1.0, -174.0);
    SimHooks.stepTimingAsync(0.11);
    update(true, true, 1.0, -174.0);
    assertThat(writes).containsExactly(-0.5, -0.5);
    update(true, true, 1.0, -180.0);
    assertThat(update(false, true, 1.0, -180.0)).isTrue();
    assertThat(writes).hasSize(2);
  }

  @Test
  void failedWritesDoNotAllowEnableAndRetryWhileDisabled() {
    homing.update(true, true, 0.15, 0.0, position -> false);
    SimHooks.stepTimingAsync(0.11);
    homing.update(true, true, 0.15, 0.0, position -> false);
    assertThat(update(false, true, 0.15, 0.0)).isFalse();
    assertThat(writes).isEmpty();
    update(true, true, 0.15, 0.0);
    SimHooks.stepTimingAsync(0.11);
    update(true, true, 0.15, 0.0);
    assertThat(writes).hasSize(1);
    assertThat(Units.rotationsToDegrees(writes.get(0))).isCloseTo(-179.1, within(1e-9));
    assertThat(update(false, true, 0.15, -179.1)).isTrue();
  }

  @Test
  void ignoresSubThresholdDifferencesAndDebouncesLargerErrors() {
    update(true, true, 0.0, -179.995);
    SimHooks.stepTimingAsync(0.2);
    update(true, true, 0.0, -179.995);
    assertThat(writes).isEmpty();
    update(true, true, 0.0, -179.98);
    assertThat(writes).isEmpty();
    SimHooks.stepTimingAsync(0.11);
    update(true, true, 0.0, -179.98);
    assertThat(writes).containsExactly(-0.5);
  }

  @Test
  void invalidSamplesAndBriefErrorsResetDebounce() {
    update(true, true, 0.0, 0.0);
    SimHooks.stepTimingAsync(0.06);
    update(true, false, 0.0, 0.0);
    SimHooks.stepTimingAsync(0.06);
    update(true, true, 0.0, 0.0);
    assertThat(writes).isEmpty();
    update(true, true, Double.NaN, 0.0);
    update(true, true, 0.0, Double.NaN);
    update(true, true, 0.0, -180.0);
    update(true, true, 0.0, -179.98);
    SimHooks.stepTimingAsync(0.06);
    update(true, true, 0.0, -180.0);
    update(true, true, 0.0, -179.98);
    SimHooks.stepTimingAsync(0.06);
    update(true, true, 0.0, -179.98);
    assertThat(writes).isEmpty();
    assertThat(update(false, true, 0.0, -179.98)).isFalse();
  }

  @Test
  void neverWritesWhileEnabledOrFinishesWithoutDisabledAlignment() {
    assertThat(update(false, true, 0.0, 0.0)).isFalse();
    SimHooks.stepTimingAsync(0.2);
    assertThat(update(false, true, 0.0, 0.0)).isFalse();
    assertThat(writes).isEmpty();
    update(true, true, 0.0, -180.0);
    assertThat(update(false, true, 0.0, -180.0)).isTrue();
    assertThat(writes).isEmpty();
  }

  @BeforeEach
  void pauseTiming() {
    SimHooks.pauseTiming();
    homing = new TurretHoming();
  }

  @AfterEach
  void resumeTiming() {
    SimHooks.resumeTiming();
  }
}
