package frc.robot.vision;

import com.google.common.collect.ImmutableList;
import com.google.common.collect.ImmutableMap;
import com.google.common.collect.Sets;
import com.team581.spot.Spot;
import com.team581.spot.SpotCameraStatus;
import com.team581.spot.SpotStatus;
import com.team581.spot.TimestampedEstimate;
import com.team581.util.state_machines.StateMachineSubsystem;
import com.team581.vision.spot.CaptureHistory;
import com.team581.vision.spot.MeasurementQueue;
import com.team581.vision.spot.VisionProcessor;
import com.team581.vision.spot.results.VisionMeasurement;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
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
import java.util.function.DoubleSupplier;

/** Sole scheduled vision owner: collection, selection, ordering and cached observations. */
public final class Vision extends StateMachineSubsystem<VisionState> implements AutoCloseable {
  public record CameraStatus(
      double lastObservation,
      double lastAccepted,
      double lastFused,
      long accepted,
      long rejected,
      boolean usable,
      String reason) {}

  public interface CaptureSource {
    VisionProcessor.CaptureState current(double fpgaSeconds);

    Optional<Pose2d> poseAt(double fpgaSeconds);
  }

  public record Status(
      boolean available,
      String reason,
      int configured,
      int usable,
      boolean allRequiredHealthy,
      int queueDepth,
      long queueDrops,
      Map<String, CameraStatus> cameras) {}

  private record Arrival(String camera, TimestampedEstimate sample) {}

  private static final double REORDER_WINDOW_S = .02;
  private static final double NO_TAGS_WARNING_S = 2;
  private static final String NO_TAGS_WARNING = "Spot cameras haven't seen a tag recently";
  private static final Set<SpotCameraStatus.State> STREAMING =
      Set.of(SpotCameraStatus.State.STREAMING, SpotCameraStatus.State.FALLING_BEHIND);
  private static final Set<VisionProcessor.Reason> OPERATIONAL =
      Set.of(
          VisionProcessor.Reason.ACCEPTED,
          VisionProcessor.Reason.NO_TAGS,
          VisionProcessor.Reason.HISTORY,
          VisionProcessor.Reason.FIELD_BOUNDS,
          VisionProcessor.Reason.ROBOT_TILT,
          VisionProcessor.Reason.AMBIGUOUS,
          VisionProcessor.Reason.RESIDUAL,
          VisionProcessor.Reason.INNOVATION);
  private final Spot spot;
  private final List<String> cameraNames;
  private final CaptureSource capture;
  private final DoubleSupplier clock;
  private final BooleanSupplier operatorVision;
  private final CaptureHistory history;
  private final VisionProcessor processor;
  private final MeasurementQueue queue;
  // Filled on the NetworkTables listener thread, drained on the main loop.
  private final ConcurrentLinkedQueue<Arrival> incoming = new ConcurrentLinkedQueue<>();
  private final Map<String, CameraStatus> cameras = new HashMap<>();
  private double lastTagSeen = Double.NEGATIVE_INFINITY;
  private final Set<String> faults = new HashSet<>();
  private List<VisionMeasurement> prepared = ImmutableList.of();
  private final Map<String, VisionMeasurement> offered = new HashMap<>();
  private Optional<SpotStatus> spotStatus = Optional.empty();
  private double now;
  private double lastTag = Double.NEGATIVE_INFINITY;
  private double resetTime = Double.NEGATIVE_INFINITY;
  private boolean hasSeenTag;
  private boolean closed;

  public Vision(
      Spot spot,
      List<String> cameraNames,
      CaptureSource capture,
      DoubleSupplier clock,
      BooleanSupplier operatorVision) {
    super(SubsystemPriority.VISION, VisionState.DEFAULT_STATE);
    this.spot = spot;
    this.cameraNames = List.copyOf(cameraNames);
    this.capture = capture;
    this.clock = clock;
    this.operatorVision = operatorVision;
    history = new CaptureHistory(capture::poseAt);
    processor =
        new VisionProcessor(history, VisionTuning::policy, () -> !hasRecentFusedMeasurement());
    queue = new MeasurementQueue(REORDER_WINDOW_S, 128);
    for (var name : cameraNames) {
      spot.camera(name).onEstimate(sample -> incoming.add(new Arrival(name, sample)));
    }
  }

  @Override
  public void close() {
    if (closed) return;
    closed = true;
    prepared = ImmutableList.of();
    offered.clear();
    queue.clearPending();
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
    prepared = ImmutableList.of();
    offered.clear();
    spotStatus = spot.status();
    history.add(now, capture.current(now));
    boolean available = spotStatus.isPresent();
    boolean fusion = isFusionEnabled();
    if (!fusion) queue.clearPending();
    for (var arrival : arrivals) {
      var name = arrival.camera();
      var sample = arrival.sample();
      if (sample.estimate().tagIds().length > 0) lastTagSeen = now;
      boolean uncalibrated =
          spot.camera(name)
              .status()
              .filter(s -> s.state() == SpotCameraStatus.State.UNCALIBRATED)
              .isPresent();
      var decision =
          !available
              ? VisionProcessor.Decision.reject(VisionProcessor.Reason.CONFIGURATION)
              : uncalibrated
                  ? VisionProcessor.Decision.reject(VisionProcessor.Reason.CALIBRATION)
                  : processor.process(name, sample, now);
      boolean accepted = decision.reason() == VisionProcessor.Reason.ACCEPTED;
      boolean operational = OPERATIONAL.contains(decision.reason());
      var previous =
          cameras.getOrDefault(
              name,
              new CameraStatus(
                  Double.NEGATIVE_INFINITY,
                  Double.NEGATIVE_INFINITY,
                  Double.NEGATIVE_INFINITY,
                  0,
                  0,
                  false,
                  "Waiting"));
      cameras.put(
          name,
          new CameraStatus(
              operational ? sample.timestampSeconds() : previous.lastObservation(),
              accepted ? sample.timestampSeconds() : previous.lastAccepted(),
              previous.lastFused(),
              previous.accepted() + (accepted ? 1 : 0),
              previous.rejected() + (accepted ? 0 : 1),
              previous.usable(),
              decision.reason().name()));
      if (accepted) {
        var measurement = decision.measurement().orElseThrow();
        hasSeenTag = true;
        lastTag = now;
        if (fusion) queue.add(measurement);
      }
    }
    if (!available) {
      queue.clearPending();
      retireObservations();
    }
    for (var name : cameraNames) {
      var previous =
          cameras.getOrDefault(
              name,
              new CameraStatus(
                  Double.NEGATIVE_INFINITY,
                  Double.NEGATIVE_INFINITY,
                  Double.NEGATIVE_INFINITY,
                  0,
                  0,
                  false,
                  "No estimate"));
      // A camera is healthy while Spot is streaming it, whether or not it sees a tag.
      boolean usable =
          available
              && spot.camera(name).status().filter(s -> STREAMING.contains(s.state())).isPresent();
      cameras.put(
          name,
          new CameraStatus(
              previous.lastObservation(),
              previous.lastAccepted(),
              previous.lastFused(),
              previous.accepted(),
              previous.rejected(),
              usable,
              previous.reason()));
    }
    if (fusion) prepared = queue.release(now, 32);
  }

  /** Multiplier for estimator deviations; correlated cameras must not add confidence. */
  public double correlationFactor() {
    return VisionProcessor.correlationFactor(cameraNames.size());
  }

  public List<VisionMeasurement> drainMeasurements() {
    var result = prepared;
    prepared = ImmutableList.of();
    result.forEach(value -> offered.put(value.frameId(), value));
    return result;
  }

  /** Ingestion recheck; no old prepared result survives a reset. */
  public boolean eligible(VisionMeasurement value, double timestamp) {
    return value.equals(offered.get(value.frameId()))
        && value.timestamp() > resetTime
        && timestamp - value.timestamp() <= VisionProcessor.MAX_AGE_S
        && value.timestamp() <= timestamp + VisionProcessor.FUTURE_TOLERANCE_S
        && spotStatus.isPresent();
  }

  // The copy keeps status snapshots independent of later camera updates and resets.
  @SuppressWarnings("IdentityConversion")
  public Status getStatus() {
    int usable = (int) cameras.values().stream().filter(CameraStatus::usable).count();
    return new Status(
        spotStatus.isPresent(),
        getReason(usable),
        cameraNames.size(),
        usable,
        spotStatus.filter(s -> s.state() == SpotStatus.State.READY).isPresent(),
        queue.size(),
        queue.lateCount() + queue.overloadCount(),
        ImmutableMap.copyOf(cameras));
  }

  public boolean hasRecentFusedMeasurement() {
    return isFusionEnabled()
        && cameras.values().stream()
            .anyMatch(camera ->
                now - camera.lastFused() >= 0
                    && now - camera.lastFused() <= VisionProcessor.MAX_AGE_S);
  }

  public boolean hasSeenTag() {
    return hasSeenTag;
  }

  public boolean isFusionEnabled() {
    return spotStatus.isPresent() && operatorVision.getAsBoolean();
  }

  public void measurementFused(VisionMeasurement value) {
    if (!eligible(value, now)) return;
    offered.remove(value.frameId());
    var previous = cameras.get(value.source());
    if (previous != null)
      cameras.put(
          value.source(),
          new CameraStatus(
              previous.lastObservation(),
              previous.lastAccepted(),
              Math.max(previous.lastFused(), value.timestamp()),
              previous.accepted(),
              previous.rejected(),
              previous.usable(),
              previous.reason()));
  }

  public void reset(double timestamp) {
    prepared = ImmutableList.of();
    offered.clear();
    incoming.clear();
    queue.reset(timestamp);
    processor.reset(timestamp);
    history.reset();
    resetTime = timestamp;
    retireObservations();
  }

  @Override
  public void robotPeriodic() {
    if (!closed) super.robotPeriodic();
  }

  public boolean seeingTag() {
    return now - lastTag <= VisionProcessor.MAX_AGE_S;
  }

  public double tickTime() {
    return now;
  }

  @Override
  public void whileInState(VisionState state) {
    if (closed) return;
    var status = getStatus();
    // Errors mean Spot is down or not ready. A camera that isn't streaming or not seeing a tag is
    // only a warning, since Spot reports missing required cameras itself.
    var errors = new HashSet<String>();
    var warnings = new HashSet<String>();
    if (spotStatus.isEmpty()) {
      errors.add("Spot not connected");
    } else {
      var spotState = spotStatus.orElseThrow();
      if (spotState.state() != SpotStatus.State.READY) {
        var error = spot.error();
        errors.add("Spot: " + (error.isEmpty() ? spotState.state().name() : error));
      }
      if (spotState.overridesActive()) warnings.add("Spot dashboard overrides active");
      for (var name : cameraNames) {
        var observed = cameras.get(name);
        if (observed == null || !observed.usable())
          warnings.add("Spot camera not streaming: " + name);
      }
      if (status.usable() > 0 && now - lastTagSeen > NO_TAGS_WARNING_S)
        warnings.add(NO_TAGS_WARNING);
    }
    for (var fault : errors) if (faults.add(fault)) DogLog.logFault(fault, AlertType.kError);
    for (var fault : warnings) if (faults.add(fault)) DogLog.logFault(fault, AlertType.kWarning);
    var desired = Sets.union(errors, warnings);
    for (var fault : Set.copyOf(faults))
      if (!desired.contains(fault)) {
        DogLog.clearFault(fault);
        faults.remove(fault);
      }
    DogLog.log("Vision/Spot/FusionEnabled", isFusionEnabled());
    DogLog.log("Vision/Spot/RecentFusedEvidence", hasRecentFusedMeasurement());
    DogLog.log("Vision/Spot/Available", status.available());
    DogLog.log("Vision/Spot/Reason", status.reason());
    DogLog.log("Vision/Spot/UsableCameras", status.usable());
    DogLog.log("Vision/Spot/TagSeenAgeSeconds", now - lastTagSeen);
    DogLog.log("Vision/Spot/QueueDrops", status.queueDrops());
    DogLog.log("Vision/Spot/State", spotStatus.map(s -> s.state().name()).orElse("DISCONNECTED"));
    DogLog.log("Vision/Spot/Error", spot.error());
    DogLog.log("Vision/Spot/Version", spotStatus.map(SpotStatus::version).orElse(""));
    for (var entry : cameras.entrySet()) {
      String path = "Vision/Spot/" + entry.getKey();
      var value = entry.getValue();
      var cameraStatus = spot.camera(entry.getKey()).status();
      DogLog.log(path + "/State", cameraStatus.map(s -> s.state().name()).orElse("UNKNOWN"));
      DogLog.log(path + "/Fps", cameraStatus.map(SpotCameraStatus::fps).orElse(0.0));
      DogLog.log(
          path + "/DroppedFrames", cameraStatus.map(SpotCameraStatus::droppedFrames).orElse(0L));
      DogLog.log(path + "/Reason", value.reason());
      DogLog.log(path + "/Usable", value.usable());
      DogLog.log(path + "/ObservationAgeSeconds", now - value.lastObservation());
      DogLog.log(path + "/FusedAgeSeconds", now - value.lastFused());
      DogLog.log(path + "/Accepted", value.accepted());
      DogLog.log(path + "/Rejected", value.rejected());
      DogLog.log(path + "/LastFusedFpgaS", value.lastFused());
      DogLog.log(path + "/LastObservationFpgaS", value.lastObservation());
    }
  }

  private String getReason(int usable) {
    if (spotStatus.isEmpty()) return "Spot not connected";
    if (usable == 0) return "No cameras streaming";
    return "Ready";
  }

  private void retireObservations() {
    cameras.replaceAll((name, old) ->
        new CameraStatus(
            Double.NEGATIVE_INFINITY,
            Double.NEGATIVE_INFINITY,
            Double.NEGATIVE_INFINITY,
            old.accepted(),
            old.rejected(),
            false,
            "Retired"));
    lastTag = Double.NEGATIVE_INFINITY;
  }
}
