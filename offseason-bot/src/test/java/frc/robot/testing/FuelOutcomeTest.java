package frc.robot.testing;

import static org.assertj.core.api.Assertions.assertThat;
import static org.assertj.core.api.Assertions.assertThatIllegalArgumentException;

import org.junit.jupiter.api.Test;

final class FuelOutcomeTest {
  @Test
  void impossibleOrUnreconciledCountsAreRejected() {
    assertThatIllegalArgumentException().isThrownBy(() -> new FuelOutcome(5, 4, 5, 1));
    assertThatIllegalArgumentException().isThrownBy(() -> new FuelOutcome(5, 4, 4, 0));
    assertThatIllegalArgumentException().isThrownBy(() -> new FuelOutcome(0, 0, 0, 0));
    assertThatIllegalArgumentException()
        .isThrownBy(() -> new FuelOutcome(5, 5, 5, 0).evaluate("score", 0));
  }

  @Test
  void scoringUsesAvailableFuelAndCannotPassByRetainingMostOfIt() {
    assertThat(new FuelOutcome(5, 1, 1, 4).evaluate("score", 4).status())
        .isEqualTo(DiagnosticRoutine.Status.FAILED);
    assertThat(new FuelOutcome(5, 5, 4, 0).evaluate("score", 4).status())
        .isEqualTo(DiagnosticRoutine.Status.PASSED);
    assertThat(new FuelOutcome(5, 0, 0, 5).evaluate("score", 4).status())
        .isEqualTo(DiagnosticRoutine.Status.FAILED);
  }
}
