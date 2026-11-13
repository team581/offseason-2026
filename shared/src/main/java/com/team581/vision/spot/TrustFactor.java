package com.team581.vision.spot;

import com.team581.vision.spot.results.VisionMeasurement;
import java.util.ArrayDeque;
import java.util.function.DoubleSupplier;

/** Capture-time evidence heuristic in metres, not covariance or probability. */
public final class TrustFactor {
  public record Policy(double trustworthyM, double lostM, double travelMPerM, double correlationS) {
    public static Policy provisional() {
      return new Policy(.254, 1., .05, .02);
    }
  }

  private record Penalty(double time, double value) {}

  private final Policy policy = Policy.provisional();
  private final DoubleSupplier trustworthyMeters;

  private final ArrayDeque<Penalty> history = new ArrayDeque<>();

  private double score = Double.POSITIVE_INFINITY;
  private double evidenceTimestamp = Double.NEGATIVE_INFINITY;
  private double groupStart = Double.NEGATIVE_INFINITY;
  private double groupWorst;
  private double groupPenalty;
  private double penalty;
  private double time;

  public TrustFactor() {
    this(() -> Policy.provisional().trustworthyM());
  }

  public TrustFactor(DoubleSupplier trustworthyMeters) {
    this.trustworthyMeters = trustworthyMeters;
  }

  public double evidenceAge(double now) {
    return now - evidenceTimestamp;
  }

  public double get() {
    return score;
  }

  /** Caller supplies only measurements actually submitted to the estimator. */
  public void ingest(VisionMeasurement value) {
    if (value.timestamp() < evidenceTimestamp) return;
    // Disagreement with the estimate plus this measurement's own deviation, counted once.
    double evidence = value.innovationM() + Math.max(value.stdX(), value.stdY());
    if (!Double.isFinite(evidence) || value.innovationM() < 0) return;
    if (value.timestamp() - groupStart > policy.correlationS()) {
      groupStart = value.timestamp();
      groupWorst = evidence;
      groupPenalty = penaltyAt(groupStart);
    } else {
      groupWorst = Math.max(groupWorst, evidence);
    }
    score = groupWorst + Math.max(0., penalty - groupPenalty);
    evidenceTimestamp = value.timestamp();
  }

  public boolean isLost() {
    return score >= policy.lostM();
  }

  public boolean isTrustworthy() {
    return score <= trustworthyMeters.getAsDouble();
  }

  public void reset(double now, boolean explicitlySeeded) {
    if (!Double.isFinite(now)) throw new IllegalArgumentException("finite reset time required");
    score = explicitlySeeded ? trustworthyMeters.getAsDouble() : Double.POSITIVE_INFINITY;
    evidenceTimestamp = groupStart = Double.NEGATIVE_INFINITY;
    penalty = 0.;
    time = now;
    history.clear();
    history.addLast(new Penalty(now, 0.));
  }

  public void updateOdometry(double distanceM, double elapsedS) {
    if (!Double.isFinite(distanceM) || !Double.isFinite(elapsedS) || distanceM < 0 || elapsedS < 0)
      throw new IllegalArgumentException("nonnegative finite metres and seconds required");
    // Only driving makes odometry drift; a stationary robot keeps its trust indefinitely.
    double delta = distanceM * policy.travelMPerM();
    score += delta;
    penalty += delta;
    time += elapsedS;
    history.addLast(new Penalty(time, penalty));
    while (history.size() > 256) history.removeFirst();
  }

  private double penaltyAt(double timestamp) {
    if (history.isEmpty()) return penalty;
    Penalty previous = history.getFirst();
    for (var next : history) {
      if (next.time() >= timestamp) {
        double span = next.time() - previous.time();
        double fraction = span <= 0 ? 0 : Math.max(0, (timestamp - previous.time()) / span);
        return previous.value() + fraction * (next.value() - previous.value());
      }
      previous = next;
    }
    return penalty;
  }
}
