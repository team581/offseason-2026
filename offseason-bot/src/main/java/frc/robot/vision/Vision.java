package frc.robot.vision;

import static java.util.Comparator.comparingDouble;

import com.google.common.collect.ImmutableList;
import com.google.common.collect.Sets;
import com.team581.spot.Spot;
import com.team581.spot.SpotCameraStatus;
import com.team581.spot.SpotEstimate;
import com.team581.spot.SpotStatus;
import com.team581.spot.TimestampedEstimate;
import com.team581.util.AprilTags;
import com.team581.util.state_machines.StateMachineSubsystem;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.util.scheduling.SubsystemPriority;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleFunction;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.stream.Stream;

/** Turns Spot's pose estimates into measurements for the pose estimator. */
public final class Vision extends StateMachineSubsystem<VisionState> implements AutoCloseable {
  /**
   * Robot motion now. Estimates are at most {@link #MAX_AGE_S} old, so this stands in for motion at
   * capture.
   */
  public record Motion(double speedMps, double angularRateRadPerS, double tiltRad) {}

  /** Why the latest estimate from a camera was accepted or rejected. */
  public enum Reason {
    ACCEPTED,
    SPOT_NOT_READY,
    UNCALIBRATED,
    TIMESTAMP,
    RESET,
    DUPLICATE,
    NO_TAGS,
    HISTORY,
    RESIDUAL,
    AMBIGUOUS,
    FIELD_BOUNDS,
    ROBOT_TILT
  }

  private record Arrival(String camera, TimestampedEstimate sample) {}

  private static final double MAX_AGE_S = .25;
  private static final double FUTURE_TOLERANCE_S = .005;
  // Candidates with more reprojection error than this aren't contenders.
  private static final double MAX_ERROR_PX = 20;
  private static final double MAX_HEADING_ERROR_RAD = Math.toRadians(35);
  private static final double MAX_TILT_RAD = Math.toRadians(15);
  private static final double FIELD_MARGIN_M = .25;
  // Deviations fit to mttd Q11 and practice-field logs: repeat readings agree within 1 cm, and
  // cameras disagree by ~0.1-0.25 m at 3.6 m, so accuracy, not noise, sets the scale.
  private static final double MIN_STD_M = .02;
  private static final double STD_PER_RANGE_SQUARED = .008;
  // Error in Spot's capture timestamps, which robot motion turns into position error.
  private static final double TIMESTAMP_ERROR_S = .02;
  private static final double NO_TAGS_WARNING_S = 2;
  private static final Set<SpotCameraStatus.State> STREAMING =
      Sets.immutableEnumSet(
          SpotCameraStatus.State.STREAMING, SpotCameraStatus.State.FALLING_BEHIND);

  private static double headingError(SpotEstimate.Candidate candidate, Rotation2d heading) {
    return Math.abs(candidate.pose().toPose2d().getRotation().minus(heading).getRadians());
  }

  private static boolean isOnField(Pose2d pose) {
    var field = AprilTags.FIELD_LAYOUT;
    return pose.getX() >= -FIELD_MARGIN_M
        && pose.getX() <= field.getFieldLength() + FIELD_MARGIN_M
        && pose.getY() >= -FIELD_MARGIN_M
        && pose.getY() <= field.getFieldWidth() + FIELD_MARGIN_M;
  }

  /** Whether an estimate captured at {@code timestamp} is neither stale nor from the future. */
  private static boolean isRecent(double timestamp, double now) {
    return timestamp > 0 && timestamp <= now + FUTURE_TOLERANCE_S && now - timestamp <= MAX_AGE_S;
  }

  /**
   * Spot may offer an alternate answer. Of the candidates that fit the tags well, the robot's
   * heading at capture picks the likelier one.
   */
  private static Optional<SpotEstimate.Candidate> pickCandidate(
      SpotEstimate estimate, Rotation2d heading) {
    return Stream.concat(Stream.of(estimate.primary()), estimate.alternate().stream())
        .filter(candidate -> candidate.error() <= MAX_ERROR_PX)
        .min(comparingDouble(candidate -> headingError(candidate, heading)));
  }

  /**
   * Error grows with distance squared, shrinks with more tags (up to 3), and grows with
   * reprojection error. Motion during the timestamp error adds to it.
   */
  private static double std(
      SpotEstimate estimate, SpotEstimate.Candidate candidate, Motion motion) {
    double range = estimate.averageTagDistance();
    int tags = estimate.tagIds().length;
    double std =
        Math.max(MIN_STD_M, STD_PER_RANGE_SQUARED * range * range / Math.sqrt(Math.min(3, tags)))
            * (1 + candidate.error() / MAX_ERROR_PX);
    return Math.hypot(
        std,
        TIMESTAMP_ERROR_S
            * (Math.abs(motion.speedMps()) + Math.abs(motion.angularRateRadPerS()) * range));
  }

  private final Spot spot;
  private final List<String> cameras;
  private final DoubleFunction<Optional<Pose2d>> poseAt;
  private final Supplier<Motion> motion;
  private final DoubleSupplier clock;
  private final BooleanSupplier fusionEnabled;
  // Filled on the NetworkTables listener thread, drained on the main loop.
  private final ConcurrentLinkedQueue<Arrival> incoming = new ConcurrentLinkedQueue<>();
  private final Map<String, Double> lastTimestamp = new HashMap<>();
  private final Map<String, Reason> reasons = new HashMap<>();
  private final Set<String> faults = new HashSet<>();
  private List<VisionMeasurement> measurements = ImmutableList.of();

  private Optional<SpotStatus> spotStatus = Optional.empty();

  private double now;

  private double resetTime = Double.NEGATIVE_INFINITY;

  private double lastTagSeen = Double.NEGATIVE_INFINITY;

  private boolean closed;

  /**
   * @param poseAt The robot's odometry pose at an FPGA timestamp.
   * @param fusionEnabled Whether accepted measurements should reach the pose estimator.
   */
  public Vision(
      Spot spot,
      List<String> cameras,
      DoubleFunction<Optional<Pose2d>> poseAt,
      Supplier<Motion> motion,
      DoubleSupplier clock,
      BooleanSupplier fusionEnabled) {
    super(SubsystemPriority.VISION, VisionState.DEFAULT_STATE);
    this.spot = spot;
    this.cameras = List.copyOf(cameras);
    this.poseAt = poseAt;
    this.motion = motion;
    this.clock = clock;
    this.fusionEnabled = fusionEnabled;
    for (var name : cameras) {
      spot.camera(name).onEstimate(sample -> incoming.add(new Arrival(name, sample)));
    }
  }

  @Override
  public void close() {
    if (closed) return;
    closed = true;
    measurements = ImmutableList.of();
    for (var fault : faults) DogLog.clearFault(fault);
    faults.clear();
    spot.close();
  }

  @Override
  public void collectInputs() {
    if (closed) return;
    var arrivals = new ArrayList<Arrival>();
    for (Arrival arrival; (arrival = incoming.poll()) != null; ) arrivals.add(arrival);
    // Read the clock after draining so estimates that just arrived don't appear to be from the
    // future.
    now = clock.getAsDouble();
    spotStatus = spot.status();
    var currentMotion = motion.get();
    var accepted = new ArrayList<VisionMeasurement>();
    for (var arrival : arrivals) {
      if (arrival.sample().estimate().tagIds().length > 0) lastTagSeen = now;
      reasons.put(
          arrival.camera(), evaluate(arrival.camera(), arrival.sample(), currentMotion, accepted));
    }
    accepted.sort(comparingDouble(VisionMeasurement::timestamp));
    measurements = fusionEnabled.getAsBoolean() ? List.copyOf(accepted) : ImmutableList.of();
  }

  /**
   * Multiplier for estimator deviations. The cameras see the same tags at the same time, so their
   * errors are correlated and more cameras must not add confidence.
   */
  public double correlationFactor() {
    return Math.sqrt(cameras.size());
  }

  /** Measurements accepted this loop, oldest first. Each is returned once. */
  public List<VisionMeasurement> drainMeasurements() {
    var result = measurements;
    measurements = ImmutableList.of();
    return result;
  }

  public boolean isSpotReady() {
    return spotStatus.filter(s -> s.state() == SpotStatus.State.READY).isPresent();
  }

  /** Discards estimates captured at or before {@code timestamp}, so they can't undo a reset. */
  public void reset(double timestamp) {
    measurements = ImmutableList.of();
    incoming.clear();
    lastTimestamp.clear();
    resetTime = timestamp;
  }

  @Override
  public void robotPeriodic() {
    if (!closed) super.robotPeriodic();
  }

  public double tickTime() {
    return now;
  }

  @Override
  public void whileInState(VisionState state) {
    if (closed) return;
    // Errors mean Spot is down or not ready. A camera that isn't streaming or not seeing a tag is
    // only a warning, since Spot reports missing required cameras itself.
    var errors = new HashSet<String>();
    var warnings = new HashSet<String>();
    if (spotStatus.isEmpty()) {
      errors.add("Spot not connected");
    } else {
      var status = spotStatus.orElseThrow();
      if (status.state() != SpotStatus.State.READY) {
        var error = spot.error();
        errors.add("Spot: " + (error.isEmpty() ? status.state().name() : error));
      }
      if (status.overridesActive()) warnings.add("Spot dashboard overrides active");
      boolean anyStreaming = false;
      for (var name : cameras) {
        if (spot.camera(name).status().filter(s -> STREAMING.contains(s.state())).isPresent()) {
          anyStreaming = true;
        } else {
          warnings.add("Spot camera not streaming: " + name);
        }
      }
      if (anyStreaming && now - lastTagSeen > NO_TAGS_WARNING_S)
        warnings.add("Spot cameras haven't seen a tag recently");
    }
    for (var fault : errors) if (faults.add(fault)) DogLog.logFault(fault, AlertType.kError);
    for (var fault : warnings) if (faults.add(fault)) DogLog.logFault(fault, AlertType.kWarning);
    var desired = Sets.union(errors, warnings);
    for (var fault : Set.copyOf(faults))
      if (!desired.contains(fault)) {
        DogLog.clearFault(fault);
        faults.remove(fault);
      }

    DogLog.log("Vision/Spot/FusionEnabled", fusionEnabled.getAsBoolean());
    DogLog.log("Vision/Spot/State", spotStatus.map(s -> s.state().name()).orElse("DISCONNECTED"));
    DogLog.log("Vision/Spot/Error", spot.error());
    DogLog.log("Vision/Spot/Version", spotStatus.map(SpotStatus::version).orElse(""));
    DogLog.log("Vision/Spot/TagSeenAgeSeconds", now - lastTagSeen);
    for (var name : cameras) {
      var path = "Vision/Spot/" + name;
      var status = spot.camera(name).status();
      DogLog.log(path + "/State", status.map(s -> s.state().name()).orElse("UNKNOWN"));
      DogLog.log(path + "/Fps", status.map(SpotCameraStatus::fps).orElse(0.0));
      DogLog.log(path + "/DroppedFrames", status.map(SpotCameraStatus::droppedFrames).orElse(0L));
      DogLog.log(path + "/Reason", reason(name).map(Reason::name).orElse("NONE"));
    }
  }

  /** Checks an estimate and, if it's trustworthy, adds a measurement of it to {@code accepted}. */
  private Reason evaluate(
      String camera, TimestampedEstimate sample, Motion motion, List<VisionMeasurement> accepted) {
    if (spotStatus.isEmpty()) return Reason.SPOT_NOT_READY;
    if (spot.camera(camera)
        .status()
        .filter(s -> s.state() == SpotCameraStatus.State.UNCALIBRATED)
        .isPresent()) {
      return Reason.UNCALIBRATED;
    }
    double timestamp = sample.timestampSeconds();
    if (!isRecent(timestamp, now)) return Reason.TIMESTAMP;
    if (timestamp <= resetTime) return Reason.RESET;
    if (timestamp <= lastTimestamp.getOrDefault(camera, Double.NEGATIVE_INFINITY)) {
      return Reason.DUPLICATE;
    }
    lastTimestamp.put(camera, timestamp);

    var estimate = sample.estimate();
    if (estimate.tagIds().length == 0) return Reason.NO_TAGS;
    var odometry = poseAt.apply(timestamp);
    if (odometry.isEmpty()) return Reason.HISTORY;
    var robotThen = odometry.orElseThrow();

    var best = pickCandidate(estimate, robotThen.getRotation());
    if (best.isEmpty()) return Reason.RESIDUAL;
    var candidate = best.orElseThrow();
    if (headingError(candidate, robotThen.getRotation()) > MAX_HEADING_ERROR_RAD) {
      return Reason.AMBIGUOUS;
    }
    var pose = candidate.pose().toPose2d();
    if (!isOnField(pose)) return Reason.FIELD_BOUNDS;
    if (motion.tiltRad() > MAX_TILT_RAD) return Reason.ROBOT_TILT;

    accepted.add(
        new VisionMeasurement(
            pose,
            timestamp,
            std(estimate, candidate, motion),
            pose.getTranslation().getDistance(robotThen.getTranslation())));
    return Reason.ACCEPTED;
  }

  Optional<Reason> reason(String camera) {
    return Optional.ofNullable(reasons.get(camera));
  }
}
