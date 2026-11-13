package frc.robot.turret;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import java.util.function.DoublePredicate;

/** Tracks the hand-set reference while disabled before the first enable. */
final class TurretHoming {
  private final Debouncer correctionDebouncer = new Debouncer(TurretConfig.HOMING_DEBOUNCE_SECONDS);
  private boolean aligned = false;
  private boolean correctionPending = false;

  /** Returns true on enable only after a valid, aligned disabled sample. */
  boolean update(
      boolean disabled,
      boolean inputsValid,
      double rotorPosition,
      double measuredAngle,
      DoublePredicate setPosition) {
    if (!inputsValid || !Double.isFinite(rotorPosition) || !Double.isFinite(measuredAngle)) {
      correctionDebouncer.calculate(false);
      correctionPending = false;
      aligned = false;
      return false;
    }
    if (!disabled) {
      correctionDebouncer.calculate(false);
      correctionPending = false;
      return aligned;
    }

    double homedPosition = TurretCalculator.calculateHomedPosition(rotorPosition);
    double errorDegrees = Math.abs(Units.rotationsToDegrees(homedPosition) - measuredAngle);
    boolean needsCorrection = errorDegrees > TurretConfig.HOMING_CORRECTION_THRESHOLD_DEGREES;
    aligned = !needsCorrection;
    if (!correctionPending) {
      correctionDebouncer.calculate(false);
    }
    correctionPending = needsCorrection;
    if (correctionDebouncer.calculate(needsCorrection) && setPosition.test(homedPosition)) {
      aligned = true;
      correctionPending = false;
      correctionDebouncer.calculate(false);
    }
    return false;
  }
}
