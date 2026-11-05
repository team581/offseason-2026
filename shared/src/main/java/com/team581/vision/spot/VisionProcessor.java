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
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
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
      double maxTimestampUncertaintyS) {
    public static Policy conservative() {
      // Engineering defaults, requiring held-out robot trials. Heading remains
      // suppressed with a large finite deviation supported by CTRE's interface.
      return new Policy(
          .04, .015, 2.0, Math.toRadians(15), Math.toRadians(35), 5.0, .5, Math.toRadians(30), .02);
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
    OLD_EPOCH,
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
  private final String profileHash;
  private final String fieldHash;
  private final Map<String, Transform3d> cameraMounts = new HashMap<>();
  private final History history;
  private final Supplier<Policy> policy;
  private final long fieldTagMask;
  private Acknowledgement contextAcknowledgement;
  private final Map<String, long[]> frameContexts = new HashMap<>();
  private final Map<String, Long> lastSequence = new HashMap<>();

  private final Map<String, String> currentEpoch = new HashMap<>();

  private final Set<String> retiredEpochs = new HashSet<>();

  private double resetTimestamp = Double.NEGATIVE_INFINITY;

  private String generation = "";

  public VisionProcessor(Profile profile, History history, Policy policy) {
    this(profile, history, () -> policy);
  }

  public VisionProcessor(Profile profile, History history, Supplier<Policy> policy) {
    this.profile = profile;
    this.profileHash = SpotProtocol.hash(profile);
    this.fieldHash = SpotProtocol.hash(profile.field());
    for (var camera : profile.cameras()) {
      cameraMounts.put(
          camera.name(), new Transform3d(Pose3d.kZero, SpotProtocol.pose(camera.robotToCamera())));
    }
    this.history = history;
    this.policy = policy;
    int tagCount = profile.field().tags().size();
    fieldTagMask = tagCount == 64 ? -1L : (1L << tagCount) - 1;
  }

  public void newGeneration(String identity) {
    if (!identity.equals(generation)) {
      generation = identity;
      lastSequence.clear();
      currentEpoch.clear();
      retiredEpochs.clear();
    }
  }

  public Decision process(VisionIO.Sample sample, Acknowledgement ack, double now) {
    var frame = sample.frame();
    double timestamp = sample.captureFpgaSeconds();
    if (!ack.applied()
        || !ack.manifestHash().equals(profileHash)
        || !ack.robotId().equals(profile.robotId())
        || !ack.fieldHash().equals(fieldHash)) return Decision.reject(Reason.CONFIGURATION);
    if (!ack.equals(contextAcknowledgement)) {
      frameContexts.clear();
      for (var camera : ack.cameras())
        frameContexts.put(camera.cameraName(), SpotProtocol.frameContext(ack, camera.cameraName()));
      contextAcknowledgement = ack;
    }
    var context = frameContexts.get(sample.cameraName());
    if (context == null || frame.contextHigh() != context[0] || frame.contextLow() != context[1])
      return Decision.reject(Reason.CONFIGURATION);
    var cameraGeneration =
        ack.cameras().stream()
            .filter(camera -> camera.cameraName().equals(sample.cameraName()))
            .findFirst();
    var definition =
        profile.cameras().stream()
            .filter(camera -> camera.name().equals(sample.cameraName()))
            .findFirst();
    if (cameraGeneration.isEmpty() || definition.isEmpty() || !definition.orElseThrow().enabled())
      return Decision.reject(Reason.CONFIGURATION);
    var applied = cameraGeneration.orElseThrow();
    if (definition.orElseThrow().allowedModes().stream()
        .noneMatch(mode ->
            mode.id().equals(applied.modeId())
                && mode.opticalSignature().equals(applied.opticalSignature())))
      return Decision.reject(Reason.CONFIGURATION);
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
    String epoch = ack.serviceSession() + "/" + sample.captureEpoch();
    String key = sample.cameraName() + "/" + epoch;
    if (retiredEpochs.contains(key)) {
      return Decision.reject(Reason.OLD_EPOCH);
    }
    String previousEpoch = currentEpoch.put(sample.cameraName(), epoch);
    if (previousEpoch != null && !previousEpoch.equals(epoch)) {
      retiredEpochs.add(sample.cameraName() + "/" + previousEpoch);
      if (retiredEpochs.size() > 128) {
        // Bounded memory: a wildly flapping camera remains rejected until a new
        // service/runtime handshake, rather than forgetting an old epoch.
        currentEpoch.put(sample.cameraName(), previousEpoch);
        return Decision.reject(Reason.OLD_EPOCH);
      }
    }
    long sequence = frame.sequence();
    if (sequence <= lastSequence.getOrDefault(key, 0L)) {
      return Decision.reject(Reason.DUPLICATE);
    }
    lastSequence.put(key, sequence);
    if (!applied.calibrationValid() || applied.calibrationHash().isEmpty()) {
      return Decision.reject(Reason.CALIBRATION);
    }
    var policy = this.policy.get();
    double timeUncertainty = frame.timestampUncertainty();
    if (!Double.isFinite(timeUncertainty)
        || timeUncertainty <= 0
        || timeUncertainty > policy.maxTimestampUncertaintyS()
        || frame.timestampSource() < 1
        || frame.timestampSource() > 3) {
      return Decision.reject(Reason.TIMESTAMP);
    }
    if (frame.candidateCount() == 0) {
      return Decision.reject(Reason.NO_TAGS);
    }
    var captured = history.at(timestamp);
    if (captured.isEmpty()) {
      return Decision.reject(Reason.HISTORY);
    }
    var mount = cameraMounts.get(sample.cameraName());
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
    if (!Double.isFinite(normalized) || normalized > policy.innovationSigma()) {
      return Decision.reject(Reason.INNOVATION);
    }
    var measurement =
        new VisionMeasurement(
            sample.cameraName(),
            key
                + "/"
                + ack.runtimeGeneration()
                + "/"
                + ack.connectionEpoch()
                + "/"
                + ack.syncEpoch()
                + "/"
                + frame.tagSelectionGeneration()
                + "/"
                + sequence,
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
    lastSequence.clear();
    currentEpoch.clear();
    retiredEpochs.clear();
  }
}
