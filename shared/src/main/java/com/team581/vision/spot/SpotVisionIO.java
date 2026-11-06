package com.team581.vision.spot;

import com.team581.vision.spot.data.*;
import com.team581.vision.spot.structs.*;
import dev.doglog.DogLog;
import edu.wpi.first.networktables.*;
import edu.wpi.first.util.struct.StructBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.OptionalDouble;

/** Robot defaults go out as typed keys; observations and diagnostics use structs. */
public final class SpotVisionIO implements VisionIO {
  public static final String ROOT = "/Spot";

  public static OptionalDouble captureTimestamp(TimestampedRaw sample, boolean onRobotServer) {
    long timestamp = sample.serverTime > 1 ? sample.serverTime : sample.timestamp;
    return onRobotServer && timestamp > 1
        ? OptionalDouble.of(timestamp / 1e6)
        : OptionalDouble.empty();
  }

  private final NetworkTableInstance instance;
  private Profile profile;
  private final SpotConfigPublisher configPublisher;
  private final StructPublisher<RobotStatus> statePublisher;
  private final StringArraySubscriber cameraNames;
  private String[] registeredCameras = new String[0];
  private final RawSubscriber healthSubscriber;
  private final StringSubscriber errorSubscriber;
  private final StructBuffer<FrameResult> frameBuffer = StructBuffer.create(FrameResult.struct);
  private final StructBuffer<Heartbeat> healthBuffer = StructBuffer.create(Heartbeat.struct);
  private final Map<String, RawSubscriber> cameras = new HashMap<>();
  private final Map<String, StructSubscriber<CameraSettings>> settings = new HashMap<>();
  private Heartbeat heartbeat = new Heartbeat(0, 0, false);
  private double heartbeatTimestamp = Double.NEGATIVE_INFINITY;
  private String generation = "";
  private double resetTimestamp = Double.NEGATIVE_INFINITY;
  private long malformed;
  private long droppedObservations;
  private String lastRejection = "None";
  private TagSelection selection;
  private boolean closed;
  private double lastDiagnostics = Double.NEGATIVE_INFINITY;

  public SpotVisionIO(NetworkTableInstance instance, Profile profile, String cameraConfig) {
    this.instance = instance;
    this.profile = profile;
    configPublisher = new SpotConfigPublisher(instance, profile, cameraConfig);
    statePublisher =
        instance
            .getStructTopic(ROOT + "/robot/state", RobotStatus.struct)
            .publish(PubSubOption.periodic(.02));
    healthSubscriber =
        instance
            .getRawTopic(ROOT + "/health")
            .subscribe(
                Heartbeat.struct.getTypeString(), new byte[0], PubSubOption.keepDuplicates(true));
    errorSubscriber = instance.getStringTopic(ROOT + "/error").subscribe("");
    cameraNames = instance.getStringArrayTopic(ROOT + "/cameras").subscribe(new String[0]);
  }

  @Override
  public void close() {
    if (closed) return;
    closed = true;
    configPublisher.close();
    statePublisher.close();
    cameraNames.close();
    healthSubscriber.close();
    errorSubscriber.close();
    cameras.values().forEach(RawSubscriber::close);
    settings.values().forEach(StructSubscriber::close);
  }

  @Override
  public java.util.Optional<Profile> configuration() {
    return java.util.Optional.of(profile);
  }

  @Override
  public void logDiagnostics(double now) {
    if (now - lastDiagnostics < 1.) return;
    lastDiagnostics = now;
    DogLog.log("Vision/Spot/Transport/DroppedObservations", droppedObservations);
    DogLog.log("Vision/Spot/Transport/MalformedPackets", malformed);
    DogLog.log("Vision/Spot/Transport/LastRejection", lastRejection);
    DogLog.log("Vision/Spot/Transport/HealthReason", health(now).reason());
    DogLog.log("Vision/Spot/Transport/Error", errorSubscriber.get());
  }

  public long malformedCount() {
    return malformed;
  }

  @Override
  public Inputs readInputs(double now) {
    update();
    var result = new ArrayList<Sample>();
    var health = health(now);
    for (var camera : cameras.entrySet()) {
      for (var sample : camera.getValue().readQueue()) {
        if (!health.available()) {
          reject(health.reason());
          continue;
        }
        if (sample.value.length != FrameResult.struct.getSize()) {
          malformed++;
          reject("Incorrect frame struct size");
          continue;
        }
        var frame = frameBuffer.read(sample.value);
        var timestamp = captureTimestamp(sample, serverMode());
        if (timestamp.isEmpty()
            || timestamp.getAsDouble() <= resetTimestamp
            || frame.revisionHigh() != heartbeat.revisionHigh()
            || frame.revisionLow() != heartbeat.revisionLow()) {
          reject("Observation predates reset or configuration change");
          continue;
        }
        result.add(new Sample(camera.getKey(), frame, timestamp.getAsDouble()));
      }
    }
    return new Inputs(health, result);
  }

  @Override
  public void reset(double timestamp) {
    resetTimestamp = timestamp;
    cameras.values().forEach(RawSubscriber::readQueue);
  }

  @Override
  public void setTagSelection(TagSelection value) {
    selection = value;
  }

  private Health health(double now) {
    String reason =
        !serverMode()
            ? "Robot must be NT server"
            : now - heartbeatTimestamp > .5 || heartbeatTimestamp > now + .005
                ? "Spot heartbeat stale"
                : !heartbeat.ready() ? "Spot not ready" : "";
    return new Health(reason.isEmpty(), reason, generation);
  }

  private void registerCameras(String[] names) {
    cameras.values().forEach(RawSubscriber::close);
    settings.values().forEach(StructSubscriber::close);
    cameras.clear();
    settings.clear();
    var policies = new ArrayList<CameraPolicy>();
    for (var name : names) {
      policies.add(new CameraPolicy(name, false, false));
      String path = ROOT + "/" + name;
      cameras.put(
          name,
          instance
              .getRawTopic(path + "/observations")
              .subscribe(
                  FrameResult.struct.getTypeString(),
                  new byte[0],
                  PubSubOption.sendAll(true),
                  PubSubOption.keepDuplicates(true),
                  PubSubOption.pollStorage(16),
                  PubSubOption.periodic(.005)));
      settings.put(
          name,
          instance
              .getStructTopic(path + "/settings", CameraSettings.struct)
              .subscribe(new CameraSettings(false, false)));
    }
    profile =
        new Profile(
            policies,
            profile.field(),
            profile.fusionEnabled(),
            profile.headingEnabled(),
            profile.maxAgeS(),
            profile.futureToleranceS(),
            profile.reorderWindowS());
  }

  private void reject(String reason) {
    droppedObservations++;
    lastRejection = reason;
  }

  private boolean serverMode() {
    return instance.getNetworkMode().contains(NetworkTableInstance.NetworkMode.kServer);
  }

  private void update() {
    if (!serverMode()) return;
    statePublisher.set(
        new RobotStatus(
            selection == null ? 0 : selection.generation(),
            selection == null
                ? 0
                : SpotProtocol.tagMask(selection.allowedTagIds(), profile.field())));
    for (var sample : healthSubscriber.readQueue()) {
      if (sample.value.length != Heartbeat.struct.getSize()) {
        malformed++;
        continue;
      }
      var value = healthBuffer.read(sample.value);
      if (value.revisionHigh() != heartbeat.revisionHigh()
          || value.revisionLow() != heartbeat.revisionLow()) {
        generation =
            Long.toHexString(value.revisionHigh()) + "/" + Long.toHexString(value.revisionLow());
      }
      heartbeat = value;
      heartbeatTimestamp = captureTimestamp(sample, true).orElse(Double.NEGATIVE_INFINITY);
    }
    var names = cameraNames.get();
    if (!Arrays.equals(names, registeredCameras)) {
      registerCameras(names);
      registeredCameras = names;
    }
    ArrayList<CameraPolicy> updated = null;
    for (int index = 0; index < profile.cameras().size(); index++) {
      var camera = profile.cameras().get(index);
      for (var sample : settings.get(camera.name()).readQueue()) {
        var value = sample.value;
        if (value.enabled() != camera.enabled() || value.required() != camera.required()) {
          if (updated == null) updated = new ArrayList<>(profile.cameras());
          camera = new CameraPolicy(camera.name(), value.required(), value.enabled());
          updated.set(index, camera);
        }
      }
    }
    if (updated != null)
      profile =
          new Profile(
              updated,
              profile.field(),
              profile.fusionEnabled(),
              profile.headingEnabled(),
              profile.maxAgeS(),
              profile.futureToleranceS(),
              profile.reorderWindowS());
  }
}
