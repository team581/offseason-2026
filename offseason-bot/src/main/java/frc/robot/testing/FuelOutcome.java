package frc.robot.testing;

import static com.google.common.base.Preconditions.checkArgument;

/** Independent counted outcome. Readiness/state transitions are deliberately not inputs. */
public record FuelOutcome(int available, int launched, int hits, int retained) {
  public FuelOutcome {
    checkArgument(
        available > 0
            && launched >= 0
            && hits >= 0
            && retained >= 0
            && hits <= launched
            && launched <= available
            && retained == available - launched,
        "Fuel counts must be nonnegative and reconcile with available fuel");
  }

  public DiagnosticRoutine.Result evaluate(String name, int requiredHits) {
    checkArgument(
        requiredHits > 0 && requiredHits <= available,
        "Required hits must be between one and available fuel");
    return new DiagnosticRoutine.Result(
        name,
        hits >= requiredHits ? DiagnosticRoutine.Status.PASSED : DiagnosticRoutine.Status.FAILED,
        hits
            + "/"
            + available
            + " available fuel scored; "
            + launched
            + " launched; "
            + retained
            + " retained");
  }
}
