package com.team581.trailblazer3.math;

import edu.wpi.first.math.MathUtil;

public final class RotationBudgetEstimator {
  public record Budget(double speed, double traction) {}

  public static Budget estimate(
      DrivetrainLimits limits,
      double derivative,
      double distanceAhead,
      double referenceOmega,
      double referenceAlpha,
      double angleRemaining) {
    double radius = limits.moduleRadius();
    double coupled =
        Math.min(limits.maxSpeed(), limits.maxWheelSpeed() / (1 + Math.abs(derivative) * radius));
    double referenceDecayDistance =
        limits.maxSpeed() * Math.abs(referenceOmega) / limits.maxAlpha();
    double decay =
        referenceDecayDistance > 1e-9
            ? MathUtil.clamp(1 - distanceAhead / referenceDecayDistance, 0, 1)
            : (distanceAhead < 1e-9 ? 1 : 0);
    double omega = Math.abs(referenceOmega) * decay;
    double alpha = Math.abs(referenceAlpha) * decay;
    double omegaShare = limits.rotationSpeed(coupled);
    double alphaShare = limits.rotationAccel(coupled);
    double turnDistance =
        coupled * (Math.abs(angleRemaining) / omegaShare + omegaShare / alphaShare);
    if (Math.abs(angleRemaining) > 0.01 && distanceAhead < turnDistance) {
      omega =
          Math.max(
              omega, Math.min(omegaShare, Math.sqrt(2 * alphaShare * Math.abs(angleRemaining))));
      alpha = Math.max(alpha, alphaShare);
    }
    return new Budget(
        Math.max(0, Math.min(coupled, limits.maxWheelSpeed() - omega * radius)),
        limits.availableTraction(omega, alpha));
  }

  private RotationBudgetEstimator() {}
}
