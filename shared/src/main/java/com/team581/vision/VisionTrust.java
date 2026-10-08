package com.team581.vision;

import static com.google.common.base.Preconditions.checkArgument;

import java.util.List;

/**
 * Reviewed-policy candidate. A heuristic in metres, never a probability. Public robot behavior uses
 * this only behind the revised-trust feature flag.
 */
public final class VisionTrust {
  private double score = Double.POSITIVE_INFINITY;
  private double evidenceTimestamp = Double.NEGATIVE_INFINITY;
  private boolean collisionActive;

  public double evidenceAge(double now) {
    return now - evidenceTimestamp;
  }

  public double get() {
    return score;
  }

  public void ingest(List<VisionMeasurement> accepted) {
    if (accepted.isEmpty()) {
      return;
    }
    // Use each pre-fusion capture-time residual. A conservative worst evidence
    // score prevents additional correlated views from artificially improving it.
    double evidence =
        accepted.stream()
            .mapToDouble(value -> value.innovationM() + Math.hypot(value.stdX(), value.stdY()))
            .filter(Double::isFinite)
            .max()
            .orElse(Double.NaN);
    if (!Double.isFinite(evidence)) {
      return;
    }
    score = evidence;
    evidenceTimestamp =
        accepted.stream().mapToDouble(VisionMeasurement::timestamp).max().orElseThrow();
  }

  public boolean isLost() {
    return score >= 1.0;
  }

  public boolean isTrustworthy() {
    return score <= .254;
  }

  public void reset(double now, boolean explicitlySeeded) {
    score = explicitlySeeded ? .254 : Double.POSITIVE_INFINITY;
    evidenceTimestamp = Double.NEGATIVE_INFINITY;
    collisionActive = false;
  }

  public void updateOdometry(double distanceM, double elapsedS, boolean collision) {
    checkArgument(
        Double.isFinite(distanceM) && Double.isFinite(elapsedS) && distanceM >= 0 && elapsedS >= 0,
        "nonnegative finite odometry distance and elapsed time required");
    // Robot trials must fit these heuristic degradation rates before enabling.
    score += distanceM * .05 + elapsedS * .01;
    if (collision && !collisionActive) {
      score += 2.;
    }
    collisionActive = collision;
  }
}
