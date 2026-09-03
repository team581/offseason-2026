package frc.robot.robot_manager;

import static org.assertj.core.api.Assertions.assertThat;
import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

final class ScoringAimReadinessTest {
  @BeforeAll
  static void initializeHal() {
    HAL.initialize(500, 0);
  }

  private ScoringAimReadiness scoringAimReadiness;

  private void assertStrictAimSelected(AtomicInteger strictCalls, AtomicInteger lookaheadCalls) {
    var lookaheadCallsBefore = lookaheadCalls.get();
    assertThat(scoringAimReadiness.isLookaheadPermitted()).isFalse();
    assertThat(
            scoringAimReadiness.isAtGoal(
                () -> {
                  strictCalls.incrementAndGet();
                  return true;
                },
                () -> {
                  lookaheadCalls.incrementAndGet();
                  return false;
                }))
        .isTrue();
    assertEquals(lookaheadCallsBefore, lookaheadCalls.get());
  }

  @BeforeEach
  void setUp() {
    SimHooks.pauseTiming();
    scoringAimReadiness = new ScoringAimReadiness(1.0);
  }

  @AfterEach
  void tearDown() {
    SimHooks.resumeTiming();
  }

  @Test
  void usesStrictAimUntilTrustIsContinuousForOneSecondAndResetsImmediately() {
    var strictCalls = new AtomicInteger();
    var lookaheadCalls = new AtomicInteger();

    scoringAimReadiness.updateLocalizationTrustworthiness(true);
    assertStrictAimSelected(strictCalls, lookaheadCalls);

    SimHooks.stepTiming(0.99);
    scoringAimReadiness.updateLocalizationTrustworthiness(true);
    assertStrictAimSelected(strictCalls, lookaheadCalls);

    SimHooks.stepTiming(0.02);
    scoringAimReadiness.updateLocalizationTrustworthiness(true);
    assertThat(scoringAimReadiness.isLookaheadPermitted()).isTrue();
    assertThat(
            scoringAimReadiness.isAtGoal(
                () -> {
                  strictCalls.incrementAndGet();
                  return true;
                },
                () -> {
                  lookaheadCalls.incrementAndGet();
                  return false;
                }))
        .isFalse();
    assertEquals(2, strictCalls.get());
    assertEquals(1, lookaheadCalls.get());

    scoringAimReadiness.updateLocalizationTrustworthiness(false);
    assertStrictAimSelected(strictCalls, lookaheadCalls);

    SimHooks.stepTiming(1.01);
    scoringAimReadiness.updateLocalizationTrustworthiness(false);
    scoringAimReadiness.updateLocalizationTrustworthiness(true);
    assertStrictAimSelected(strictCalls, lookaheadCalls);

    SimHooks.stepTiming(1.01);
    scoringAimReadiness.updateLocalizationTrustworthiness(true);
    assertThat(scoringAimReadiness.isLookaheadPermitted()).isTrue();
  }
}
