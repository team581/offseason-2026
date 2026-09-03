package frc.robot.robot_manager;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import java.util.function.BooleanSupplier;

final class ScoringAimReadiness {
  private final Debouncer localizationTrustworthinessDebouncer;
  private boolean lookaheadPermitted = false;

  ScoringAimReadiness(double trustworthyDelaySeconds) {
    localizationTrustworthinessDebouncer =
        new Debouncer(trustworthyDelaySeconds, DebounceType.kRising);
  }

  boolean isAtGoal(BooleanSupplier strictAtGoal, BooleanSupplier lookaheadAtGoal) {
    return lookaheadPermitted ? lookaheadAtGoal.getAsBoolean() : strictAtGoal.getAsBoolean();
  }

  boolean isLookaheadPermitted() {
    return lookaheadPermitted;
  }

  void updateLocalizationTrustworthiness(boolean localizationTrustworthy) {
    lookaheadPermitted = localizationTrustworthinessDebouncer.calculate(localizationTrustworthy);
  }
}
