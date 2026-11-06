package com.team581.vision.spot;

import com.team581.vision.spot.data.*;
import com.team581.vision.spot.results.VisionMeasurement;
import com.team581.vision.spot.structs.PoseEstimate;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/** Robot-owned geometry, candidate selection and conservative quality model. */
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
      double candidateHeadingMarginRad,
      double maxHeadingDifferenceRad,
      double innovationSigma,
      double maxZErrorM,
      double maxTiltErrorRad,
      // Residual capture-time error after Spot's latency correction; robot-only.
      double timestampUncertaintyS) {
    public static Policy conservative() {
      // Engineering defaults, requiring held-out robot trials. Heading remains
      // suppressed with a large finite deviation supported by CTRE's interface.
      return new Policy(
          .04,
          .015,
          2.0,
          Math.toRadians(15),
          Math.toRadians(35),
          10.0,
          .5,
          Math.toRadians(30),
          .005);
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
    INNOVATION,
    TAG_POLICY
  }

  private record RecoveryEvidence(Pose2d correction, double start, double last, int count) {}

  /** Use all configured cameras to cover correlation even across separate release batches. */
  public static List<VisionMeasurement> accountForCorrelation(
      List<VisionMeasurement> observations, int cameraCount) {
    var result = new ArrayList<VisionMeasurement>();
    double factor = Math.sqrt(Math.max(1, cameraCount));
    for (var observation : observations) {
      result.add(
          new VisionMeasurement(
              observation.source(),
              observation.frameId(),
              observation.pose(),
              observation.timestamp(),
              observation.stdX() * factor,
              observation.stdY() * factor,
              observation.stdHeading() * factor,
              observation.tagIds(),
              observation.innovationM(),
              observation.normalizedInnovation() / factor));
    }
    return List.copyOf(result);
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

  private final Profile profile;
  private final History history;
  private final Supplier<Policy> policy;
  private final BooleanSupplier recoveryAllowed;
  private final Map<String, RecoveryEvidence> recovery = new HashMap<>();
  private final long fieldTagMask;
  private final Map<String, Double> lastTimestamp = new HashMap<>();

  private double resetTimestamp = Double.NEGATIVE_INFINITY;

  private String generation = "";

  public VisionProcessor(Profile profile, History history, Policy policy) {
    this(profile, history, () -> policy);
  }

  public VisionProcessor(Profile profile, History history, Supplier<Policy> policy) {
    this(profile, history, policy, () -> false);
  }

  public VisionProcessor(
      Profile profile, History history, Supplier<Policy> policy, BooleanSupplier recoveryAllowed) {
    this.profile = profile;
    this.history = history;
    this.policy = policy;
    this.recoveryAllowed = recoveryAllowed;
    int tagCount = profile.field().tags().size();
    fieldTagMask = tagCount == 64 ? -1L : (1L << tagCount) - 1;
  }

  /** Observations from different tag policies must not corroborate one another. */
  public void clearRecoveryEvidence() {
    recovery.clear();
  }

  public void newGeneration(String identity) {
    if (!identity.equals(generation)) {
      generation = identity;
      lastTimestamp.clear();
      recovery.clear();
    }
  }

  public Decision process(VisionIO.Sample sample, double now) {
    var frame = sample.frame();
    double timestamp = sample.captureFpgaSeconds();
    if (!Double.isFinite(now) || !Double.isFinite(timestamp) || timestamp <= 0) {
      return Decision.reject(Reason.TIMESTAMP);
    }
    if (timestamp <= resetTimestamp) {
      return Decision.reject(Reason.RESET);
    }
    if (timestamp > now + profile.futureToleranceS()) {
      return Decision.reject(Reason.FUTURE);
    }
    if (now - timestamp > profile.maxAgeS()) {
      return Decision.reject(Reason.STALE);
    }
    if (frame.sequence() <= 0
        || frame.candidateCount() < 0
        || frame.candidateCount() > 2
        || (frame.observedTags() & ~fieldTagMask) != 0) return Decision.reject(Reason.MALFORMED);
    for (double value :
        new double[] {
          frame.captureToDetectMs(), frame.detectMs(), frame.solveMs(), frame.totalMs()
        }) if (!Double.isFinite(value) || value < 0) return Decision.reject(Reason.MALFORMED);
    if (timestamp <= lastTimestamp.getOrDefault(sample.cameraName(), Double.NEGATIVE_INFINITY)) {
      return Decision.reject(Reason.DUPLICATE);
    }
    lastTimestamp.put(sample.cameraName(), timestamp);
    if (!frame.calibrationValid()) return Decision.reject(Reason.CALIBRATION);
    var policy = this.policy.get();
    double timeUncertainty = policy.timestampUncertaintyS();
    if (frame.timestampSource() < 1 || frame.timestampSource() > 3) {
      return Decision.reject(Reason.TIMESTAMP);
    }
    if (frame.candidateCount() == 0) {
      return Decision.reject(Reason.NO_TAGS);
    }
    var captured = history.at(timestamp);
    if (captured.isEmpty()) {
      return Decision.reject(Reason.HISTORY);
    }
    var mount = frame.robotToCamera();
    var alternatives = new ArrayList<Pose3d>();
    for (int index = 0; index < frame.candidateCount(); index++) {
      var candidate = index == 0 ? frame.primary() : frame.alternate();
      if (!validPose(candidate.pose())
          || candidate.tagMask() == 0
          || (candidate.tagMask() & ~fieldTagMask) != 0
          || !Double.isFinite(candidate.averageDistance())
          || candidate.averageDistance() <= 0
          || !Double.isFinite(candidate.reprojectionError())
          || candidate.reprojectionError() < 0
          || !Double.isFinite(candidate.imageArea())
          || candidate.imageArea() <= 0
          || candidate.imageArea() > 1
          || !Double.isFinite(candidate.imageSpread())
          || candidate.imageSpread() < 0
          || candidate.imageSpread() > 1
          || (candidate.method() != PoseEstimate.MULTI_TAG_SQPNP
              && candidate.method() != PoseEstimate.SINGLE_TAG_IPPE_SQUARE)) {
        return Decision.reject(Reason.MALFORMED);
      }
      if ((candidate.tagMask() & ~frame.observedTags()) != 0
          || candidate.method() != frame.primary().method()
          || (candidate.method() == PoseEstimate.SINGLE_TAG_IPPE_SQUARE
              && (Long.bitCount(candidate.tagMask()) != 1
                  || candidate.tagMask() != frame.primary().tagMask())))
        return Decision.reject(Reason.MALFORMED);
      var fieldCamera = candidate.pose();
      for (int tag = 0; tag < profile.field().tags().size(); tag++) {
        if ((candidate.tagMask() & (1L << tag)) == 0) continue;
        var cameraToTag =
            new Transform3d(
                fieldCamera, SpotProtocol.pose(profile.field().tags().get(tag).fieldToTag()));
        if (cameraToTag.getX() <= .05) return Decision.reject(Reason.MALFORMED);
      }
      if (candidate.reprojectionError() > policy.maxResidualPx()) {
        return Decision.reject(Reason.RESIDUAL);
      }
      alternatives.add(candidate.pose().transformBy(mount.inverse()));
    }
    int selected = 0;
    if (frame.primary().method() == PoseEstimate.SINGLE_TAG_IPPE_SQUARE) {
      if (alternatives.size() != 2) {
        return Decision.reject(Reason.AMBIGUOUS);
      }
      double a = headingError(alternatives.get(0), captured.orElseThrow().pose());
      double b = headingError(alternatives.get(1), captured.orElseThrow().pose());
      if (MathUtil.isNear(a, b, policy.candidateHeadingMarginRad())
          || Math.min(a, b) > policy.maxHeadingDifferenceRad()) {
        return Decision.reject(Reason.AMBIGUOUS);
      }
      selected = a <= b ? 0 : 1;
    } else if (alternatives.size() != 1 || Long.bitCount(frame.primary().tagMask()) < 2) {
      return Decision.reject(Reason.MALFORMED);
    }
    var robot = alternatives.get(selected);
    if (robot.getX() < -.25
        || robot.getX() > profile.field().lengthM() + .25
        || robot.getY() < -.25
        || robot.getY() > profile.field().widthM() + .25) {
      return Decision.reject(Reason.FIELD_BOUNDS);
    }
    var state = captured.orElseThrow();
    for (double value :
        new double[] {
          state.speedMps(),
          state.angularRateRadPerS(),
          state.rollRad(),
          state.pitchRad(),
          state.zM()
        }) {
      if (!Double.isFinite(value)) {
        return Decision.reject(Reason.HISTORY);
      }
    }
    if (!MathUtil.isNear(robot.getZ(), state.zM(), policy.maxZErrorM())
        || !MathUtil.isNear(robot.getRotation().getX(), state.rollRad(), policy.maxTiltErrorRad())
        || !MathUtil.isNear(
            robot.getRotation().getY(), state.pitchRad(), policy.maxTiltErrorRad())) {
      return Decision.reject(Reason.ROBOT_TILT);
    }
    var candidate = selected == 0 ? frame.primary() : frame.alternate();
    double range = candidate.averageDistance();
    double conditioning = MathUtil.clamp(Math.sqrt(.02 / candidate.imageArea()), 1., 4.);
    // Count benefit capped at sqrt(3). Overlapping simultaneous cameras are
    // additionally inflated by correlation policy at source collection.
    double sigma =
        Math.max(
                policy.translationFloorM(),
                policy.rangeSquaredCoefficient()
                    * range
                    * range
                    / Math.sqrt(Math.min(3, Long.bitCount(candidate.tagMask()))))
            * conditioning
            * (1 + candidate.reprojectionError() / policy.maxResidualPx());
    sigma =
        Math.hypot(
            sigma,
            timeUncertainty
                * (Math.abs(state.speedMps()) + Math.abs(state.angularRateRadPerS()) * range));
    double innovation =
        robot.toPose2d().getTranslation().getDistance(state.pose().getTranslation());
    double normalized = innovation / sigma;
    if (!Double.isFinite(normalized)) {
      return Decision.reject(Reason.INNOVATION);
    }
    if (normalized > policy.innovationSigma()) {
      // A wrong initial pose must not permanently veto good vision. While fusion evidence is
      // stale, require a stationary robot and repeated, precise multi-tag agreement before
      // allowing an estimator correction. Isolated outliers and single-tag ambiguity still fail.
      if (!recoveryAllowed.getAsBoolean()
          || candidate.method() != PoseEstimate.MULTI_TAG_SQPNP
          || sigma > .25
          || Math.abs(state.speedMps()) > .25
          || Math.abs(state.angularRateRadPerS()) > .25) {
        recovery.remove(sample.cameraName());
        return Decision.reject(Reason.INNOVATION);
      }
      var correction =
          new Pose2d(
              robot.toPose2d().getTranslation().minus(state.pose().getTranslation()),
              robot.toPose2d().getRotation().minus(state.pose().getRotation()));
      var previous = recovery.get(sample.cameraName());
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
      recovery.put(sample.cameraName(), evidence);
      if (evidence.count() < 3 || timestamp - evidence.start() < .1) {
        return Decision.reject(Reason.INNOVATION);
      }
    } else {
      recovery.remove(sample.cameraName());
    }
    var measurement =
        new VisionMeasurement(
            sample.cameraName(),
            sample.cameraName() + "/" + sample.captureEpoch() + "/" + frame.sequence(),
            robot.toPose2d(),
            timestamp,
            sigma,
            sigma,
            profile.headingEnabled() ? Math.max(.2, sigma / range) : 1e6,
            SpotProtocol.tagIds(candidate.tagMask(), profile.field()),
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
