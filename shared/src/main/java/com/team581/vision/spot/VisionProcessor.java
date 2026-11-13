package com.team581.vision.spot;

import com.team581.spot.SpotEstimate;
import com.team581.spot.TimestampedEstimate;
import com.team581.util.AprilTags;
import com.team581.vision.spot.results.VisionMeasurement;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/** Robot-owned candidate selection and conservative quality model for Spot estimates. */
public final class VisionProcessor {
  public record CaptureState(
      Pose2d pose,
      double speedMps,
      double angularRateRadPerS,
      double rollRad,
      double pitchRad,
      double zM) {}

  public record Decision(Reason reason, Optional<VisionMeasurement> measurement) {
    public static Decision reject(Reason reason) {
      return new Decision(reason, Optional.empty());
    }
  }

  public interface History {
    Optional<CaptureState> at(double fpgaSeconds);
  }

  public record Policy(
      double translationFloorM,
      double rangeSquaredCoefficient,
      double maxResidualPx,
      double maxHeadingDifferenceRad,
      double innovationSigma,
      // IMU roll/pitch magnitude at capture beyond which the estimate is distrusted.
      double maxTiltRad,
      // Residual capture-time error in Spot's timestamps; robot-only.
      double timestampUncertaintyS) {
    public static Policy conservative() {
      // Engineering defaults, requiring held-out robot trials. Heading remains
      // suppressed with a large finite deviation supported by CTRE's interface.
      return new Policy(.04, .015, 2.0, Math.toRadians(35), 10.0, Math.toRadians(15), .005);
    }
  }

  public enum Reason {
    ACCEPTED,
    NO_TAGS,
    CONFIGURATION,
    CALIBRATION,
    TIMESTAMP,
    STALE,
    FUTURE,
    DUPLICATE,
    RESET,
    MALFORMED,
    HISTORY,
    FIELD_BOUNDS,
    ROBOT_TILT,
    AMBIGUOUS,
    RESIDUAL,
    INNOVATION
  }

  private record RecoveryEvidence(Pose2d correction, double start, double last, int count) {}

  /** Estimates captured longer ago than this are rejected. */
  public static final double MAX_AGE_S = .25;

  /** Estimates timestamped further than this in the future are rejected. */
  public static final double FUTURE_TOLERANCE_S = .005;

  /**
   * Estimator deviation multiplier covering correlation between all configured cameras, even across
   * separate release batches. Measurements keep their uninflated deviations.
   */
  public static double correlationFactor(int cameraCount) {
    return Math.sqrt(Math.max(1, cameraCount));
  }

  private static double headingError(Pose3d candidate, Pose2d history) {
    return Math.abs(candidate.toPose2d().getRotation().minus(history.getRotation()).getRadians());
  }

  private static boolean validPose(Pose3d pose) {
    var q = pose.getRotation().getQuaternion();
    for (double value :
        new double[] {
          pose.getX(), pose.getY(), pose.getZ(), q.getW(), q.getX(), q.getY(), q.getZ()
        }) if (!Double.isFinite(value)) return false;
    return MathUtil.isNear(q.norm(), 1.0, 1e-6);
  }

  private final History history;
  private final Supplier<Policy> policy;
  private final BooleanSupplier recoveryAllowed;
  private final Map<String, RecoveryEvidence> recovery = new HashMap<>();
  private final Map<String, Double> lastTimestamp = new HashMap<>();

  private double resetTimestamp = Double.NEGATIVE_INFINITY;

  public VisionProcessor(
      History history, Supplier<Policy> policy, BooleanSupplier recoveryAllowed) {
    this.history = history;
    this.policy = policy;
    this.recoveryAllowed = recoveryAllowed;
  }

  public Decision process(String camera, TimestampedEstimate sample, double now) {
    var estimate = sample.estimate();
    double timestamp = sample.timestampSeconds();
    if (!Double.isFinite(now) || !Double.isFinite(timestamp) || timestamp <= 0) {
      return Decision.reject(Reason.TIMESTAMP);
    }
    if (timestamp <= resetTimestamp) {
      return Decision.reject(Reason.RESET);
    }
    if (timestamp > now + FUTURE_TOLERANCE_S) {
      return Decision.reject(Reason.FUTURE);
    }
    if (now - timestamp > MAX_AGE_S) {
      return Decision.reject(Reason.STALE);
    }
    if (timestamp <= lastTimestamp.getOrDefault(camera, Double.NEGATIVE_INFINITY)) {
      return Decision.reject(Reason.DUPLICATE);
    }
    lastTimestamp.put(camera, timestamp);
    if (estimate.tagIds().length == 0) {
      return Decision.reject(Reason.NO_TAGS);
    }
    double range = estimate.averageTagDistance();
    if (!Double.isFinite(range) || range <= 0) {
      return Decision.reject(Reason.MALFORMED);
    }
    var captured = history.at(timestamp);
    if (captured.isEmpty()) {
      return Decision.reject(Reason.HISTORY);
    }
    var policy = this.policy.get();
    var candidates = new ArrayList<SpotEstimate.Candidate>();
    candidates.add(estimate.primary());
    estimate.alternate().ifPresent(candidates::add);
    var fits = new ArrayList<SpotEstimate.Candidate>();
    for (var candidate : candidates) {
      if (!validPose(candidate.pose())
          || !Double.isFinite(candidate.error())
          || candidate.error() < 0) {
        return Decision.reject(Reason.MALFORMED);
      }
      // Spot's candidates are its plausible answers; one that fits poorly is not a contender.
      if (candidate.error() <= policy.maxResidualPx()) {
        fits.add(candidate);
      }
    }
    if (fits.isEmpty()) {
      return Decision.reject(Reason.RESIDUAL);
    }
    var state = captured.orElseThrow();
    // The robot's heading at capture decides which of Spot's answers is most likely.
    var selected = fits.get(0);
    for (var candidate : fits) {
      if (headingError(candidate.pose(), state.pose())
          < headingError(selected.pose(), state.pose())) {
        selected = candidate;
      }
    }
    if (headingError(selected.pose(), state.pose()) > policy.maxHeadingDifferenceRad()) {
      return Decision.reject(Reason.AMBIGUOUS);
    }
    var robot = selected.pose().toPose2d();
    if (robot.getX() < -.25
        || robot.getX() > AprilTags.FIELD_LAYOUT.getFieldLength() + .25
        || robot.getY() < -.25
        || robot.getY() > AprilTags.FIELD_LAYOUT.getFieldWidth() + .25) {
      return Decision.reject(Reason.FIELD_BOUNDS);
    }
    for (double value :
        new double[] {
          state.speedMps(), state.angularRateRadPerS(), state.rollRad(), state.pitchRad()
        }) {
      if (!Double.isFinite(value)) {
        return Decision.reject(Reason.HISTORY);
      }
    }
    if (Math.hypot(state.rollRad(), state.pitchRad()) > policy.maxTiltRad()) {
      return Decision.reject(Reason.ROBOT_TILT);
    }
    int tagCount = estimate.tagIds().length;
    // Count benefit capped at sqrt(3). Overlapping simultaneous cameras are
    // additionally inflated by correlationFactor at estimator ingestion.
    double sigma =
        Math.max(
                policy.translationFloorM(),
                policy.rangeSquaredCoefficient() * range * range / Math.sqrt(Math.min(3, tagCount)))
            * (1 + selected.error() / policy.maxResidualPx());
    sigma =
        Math.hypot(
            sigma,
            policy.timestampUncertaintyS()
                * (Math.abs(state.speedMps()) + Math.abs(state.angularRateRadPerS()) * range));
    double innovation = robot.getTranslation().getDistance(state.pose().getTranslation());
    double normalized = innovation / sigma;
    if (!Double.isFinite(normalized)) {
      return Decision.reject(Reason.INNOVATION);
    }
    if (normalized > policy.innovationSigma()) {
      // A wrong initial pose must not permanently veto good vision. While fusion evidence is
      // stale, require a stationary robot and repeated, precise multi-tag agreement before
      // allowing an estimator correction. Isolated outliers and single-tag ambiguity still fail.
      if (!recoveryAllowed.getAsBoolean()
          || tagCount < 2
          || sigma > .25
          || Math.abs(state.speedMps()) > .25
          || Math.abs(state.angularRateRadPerS()) > .25) {
        recovery.remove(camera);
        return Decision.reject(Reason.INNOVATION);
      }
      var correction =
          new Pose2d(
              robot.getTranslation().minus(state.pose().getTranslation()),
              robot.getRotation().minus(state.pose().getRotation()));
      var previous = recovery.get(camera);
      boolean consistent =
          previous != null
              && timestamp - previous.last() <= .5
              && correction.getTranslation().getDistance(previous.correction().getTranslation())
                  <= .25
              && Math.abs(
                      correction
                          .getRotation()
                          .minus(previous.correction().getRotation())
                          .getRadians())
                  <= Math.toRadians(10);
      var evidence =
          new RecoveryEvidence(
              consistent ? previous.correction() : correction,
              consistent ? previous.start() : timestamp,
              timestamp,
              consistent ? previous.count() + 1 : 1);
      recovery.put(camera, evidence);
      if (evidence.count() < 3 || timestamp - evidence.start() < .1) {
        return Decision.reject(Reason.INNOVATION);
      }
    } else {
      recovery.remove(camera);
    }
    var measurement =
        new VisionMeasurement(
            camera,
            camera + "/" + timestamp,
            robot,
            timestamp,
            sigma,
            sigma,
            // Heading comes from the gyro; vision only corrects translation.
            1e6,
            Arrays.stream(estimate.tagIds()).boxed().toList(),
            innovation,
            normalized);
    return new Decision(Reason.ACCEPTED, Optional.of(measurement));
  }

  public void reset(double timestamp) {
    resetTimestamp = timestamp;
    lastTimestamp.clear();
    recovery.clear();
  }
}
