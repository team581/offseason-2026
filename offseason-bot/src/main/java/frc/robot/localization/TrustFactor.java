package frc.robot.localization;

import frc.robot.vision.VisionMeasurement;
import java.util.ArrayDeque;

/** Capture-time evidence heuristic in metres, not covariance or probability. */
public final class TrustFactor {
  private record Penalty(double time, double value) {}

  private static final double TRUSTWORTHY_M = .5;
  private static final double LOST_M = 1;
  // Odometry drift per meter driven.
  private static final double DRIFT_PER_M = .05;

  // Measurements this close in time are from the same moment, so they count as one observation.
  private static final double SAME_MOMENT_S = .02;

  private final ArrayDeque<Penalty> history = new ArrayDeque<>();

  private double score = Double.POSITIVE_INFINITY;
  private double evidenceTimestamp = Double.NEGATIVE_INFINITY;
  private double groupStart = Double.NEGATIVE_INFINITY;
  private double groupWorst;
  private double groupPenalty;
  private double penalty;
  private double time;

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
    double evidence = value.innovationM() + value.std();
    if (!Double.isFinite(evidence) || value.innovationM() < 0) return;
    if (value.timestamp() - groupStart > SAME_MOMENT_S) {
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
    return score >= LOST_M;
  }

  public boolean isTrustworthy() {
    return score <= TRUSTWORTHY_M;
  }

  public void reset(double now, boolean explicitlySeeded) {
    if (!Double.isFinite(now)) throw new IllegalArgumentException("finite reset time required");
    score = explicitlySeeded ? TRUSTWORTHY_M : Double.POSITIVE_INFINITY;
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
    double delta = distanceM * DRIFT_PER_M;
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
