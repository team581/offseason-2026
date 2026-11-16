package com.team581.trailblazer3.math;

import com.google.common.collect.ImmutableList;
import edu.wpi.first.math.MathUtil;
import java.util.List;

public final class RotationBudgetEstimator {
  public record Budget(double speed, double traction) {}

  public record HeadingSpan(double startProgress, double endProgress, double deltaRadians) {}

  public static Budget estimate(
      DrivetrainLimits limits,
      double derivative,
      double distanceAhead,
      double referenceOmega,
      double referenceAlpha,
      double angleRemaining) {
    return estimate(
        limits,
        Math.max(0, distanceAhead),
        0,
        derivative,
        referenceOmega,
        referenceAlpha,
        angleRemaining,
        ImmutableList.of());
  }

  public static Budget estimate(
      DrivetrainLimits limits,
      double nodeProgress,
      double measuredProgress,
      double derivative,
      double referenceOmega,
      double referenceAlpha,
      double angleRemaining,
      List<HeadingSpan> downstreamSpans) {
    double[] out = new double[2];
    estimateInto(
        limits,
        nodeProgress,
        measuredProgress,
        derivative,
        referenceOmega,
        referenceAlpha,
        angleRemaining,
        downstreamSpans,
        out);
    return new Budget(out[0], out[1]);
  }

  /**
   * Zero-allocation overload writing {@code [speed, traction]} into {@code outSpeedAndTraction}.
   */
  public static void estimateInto(
      DrivetrainLimits limits,
      double nodeProgress,
      double measuredProgress,
      double derivative,
      double referenceOmega,
      double referenceAlpha,
      double angleRemaining,
      List<HeadingSpan> downstreamSpans,
      double[] outSpeedAndTraction) {
    double distanceAhead = nodeProgress - measuredProgress;
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
    for (int i = 0; i < downstreamSpans.size(); i++) {
      var span = downstreamSpans.get(i);
      double absDelta = Math.abs(span.deltaRadians());
      if (absDelta <= 0.01) {
        continue;
      }
      double spanTurnDistance =
          coupled
              * (absDelta / Math.max(1e-3, omegaShare) + omegaShare / Math.max(1e-3, alphaShare));
      double spanStart = span.startProgress();
      double spanActiveEnd = Math.min(span.endProgress(), spanStart + spanTurnDistance);
      if (nodeProgress >= spanStart - 1e-6 && nodeProgress <= spanActiveEnd + 1e-6) {
        double windowTime =
            Math.max(0.15, (span.endProgress() - spanStart) / Math.max(0.5, coupled));
        double requiredOmega =
            Math.min(
                omegaShare, Math.max(absDelta / windowTime, Math.sqrt(2 * alphaShare * absDelta)));
        omega = Math.max(omega, requiredOmega);
        alpha = Math.max(alpha, alphaShare);
      }
    }
    outSpeedAndTraction[0] =
        Math.max(0, Math.min(coupled, limits.maxWheelSpeed() - omega * radius));
    outSpeedAndTraction[1] = limits.availableTraction(omega, alpha);
  }

  private RotationBudgetEstimator() {}
}
