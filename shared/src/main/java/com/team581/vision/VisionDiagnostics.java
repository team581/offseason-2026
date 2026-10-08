package com.team581.vision;

import com.team581.vision.proto.SpotProtos;
import dev.doglog.DogLog;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.Map;

/** Bounded per-camera counters, one owner for robot faults, no per-frame arrays. */
public final class VisionDiagnostics {
  private final Map<String, EnumMap<VisionProcessor.Reason, Long>> counters = new HashMap<>();
  private final Map<String, VisionMeasurement> accepted = new HashMap<>();
  private final Map<String, Double> observed = new HashMap<>();
  private final java.util.Set<String> activeFaults = new java.util.HashSet<>();
  private final Map<String, Long> previousObserved = new HashMap<>();
  private final Map<String, Long> previousAccepted = new HashMap<>();
  private final Map<String, Double> latency = new HashMap<>();
  private String lastBuild = "";
  private double lastLogged = Double.NEGATIVE_INFINITY;

  public void log(double now, VisionIO.Health health, SpotProtos.Profile profile) {
    var desiredFaults = new java.util.HashSet<String>();
    if (profile.getCamerasList().stream().anyMatch(SpotProtos.CameraDefinition::getEnabled)
        && !health.available()) desiredFaults.add("Spot link/configuration unavailable");
    for (var camera : profile.getCamerasList()) {
      var generation =
          health.acknowledgement().getCamerasList().stream()
              .filter(item -> item.getCameraName().equals(camera.getName()))
              .findFirst();
      if (camera.getEnabled()
          && camera.getRequired()
          && (!health.available()
              || generation.isEmpty()
              || !generation.orElseThrow().getCalibrationValid()
              || now - observed.getOrDefault(camera.getName(), Double.NEGATIVE_INFINITY) > .5)) {
        desiredFaults.add(
            "Spot " + camera.getName() + " missing or calibration/configuration invalid");
      }
    }
    if (health.available())
      for (String serviceFault : health.service().getFaultsList())
        desiredFaults.add("Spot service: " + serviceFault);
    for (String fault : desiredFaults) if (activeFaults.add(fault)) DogLog.logFault(fault);
    for (String fault : java.util.Set.copyOf(activeFaults))
      if (!desiredFaults.contains(fault)) {
        DogLog.clearFault(fault);
        activeFaults.remove(fault);
      }
    if (now - lastLogged < .5) return;
    double interval = now - lastLogged;
    lastLogged = now;
    var service = health.service();
    String build = service.getReleaseId() + "/" + service.getNativeHash();
    if (!build.equals(lastBuild)) {
      lastBuild = build;
      DogLog.log("Vision/Spot/Release", service.getReleaseId());
      DogLog.log("Vision/Spot/NativeBuild", service.getNativeHash());
    }
    DogLog.log("Vision/Spot/SyncState", service.getSyncState());
    DogLog.log("Vision/Spot/ConnectionEpoch", service.getConnectionEpoch());
    DogLog.log("Vision/Spot/SyncEpoch", service.getSyncEpoch());
    DogLog.log("Vision/Spot/SyncOffsetUs", service.getOffsetUs());
    DogLog.log("Vision/Spot/SyncUpdateAgeS", service.getSyncUpdateAgeS());
    DogLog.log("Vision/Spot/Available", health.available());
    DogLog.log("Vision/Spot/State", health.reason());
    for (var camera : profile.getCamerasList()) {
      String path = "Vision/Spot/" + camera.getName();
      for (var count :
          counters
              .getOrDefault(camera.getName(), new EnumMap<>(VisionProcessor.Reason.class))
              .entrySet()) {
        DogLog.log(path + "/" + count.getKey().name(), count.getValue());
      }
      var evidence = accepted.get(camera.getName());
      DogLog.log(path + "/AcceptedAgeS", evidence == null ? -1. : now - evidence.timestamp());
      if (evidence != null) {
        DogLog.log(path + "/InnovationM", evidence.innovationM());
        DogLog.log(path + "/StdXM", evidence.stdX());
      }
      var values =
          counters.getOrDefault(camera.getName(), new EnumMap<>(VisionProcessor.Reason.class));
      long observations = values.values().stream().mapToLong(Long::longValue).sum();
      long acceptedCount = values.getOrDefault(VisionProcessor.Reason.ACCEPTED, 0L);
      DogLog.log(
          path + "/ObservationRateHz",
          Double.isFinite(interval)
              ? (observations - previousObserved.getOrDefault(camera.getName(), observations))
                  / interval
              : -1.);
      DogLog.log(
          path + "/AcceptedRateHz",
          Double.isFinite(interval)
              ? (acceptedCount - previousAccepted.getOrDefault(camera.getName(), acceptedCount))
                  / interval
              : -1.);
      DogLog.log(path + "/LatencyS", latency.getOrDefault(camera.getName(), -1.));
      previousObserved.put(camera.getName(), observations);
      previousAccepted.put(camera.getName(), acceptedCount);
    }
    DogLog.log("Vision/Spot/ManifestHash", health.acknowledgement().getManifestHash());
    DogLog.log("Vision/Spot/RuntimeGeneration", health.acknowledgement().getRuntimeGeneration());
  }

  public void record(
      String name, VisionProcessor.Decision decision, double captureTimestamp, double now) {
    latency.put(name, now - captureTimestamp);
    counters
        .computeIfAbsent(name, unused -> new EnumMap<>(VisionProcessor.Reason.class))
        .merge(decision.reason(), 1L, Long::sum);
    decision.measurement().ifPresent(value -> accepted.put(name, value));
    if (decision.reason() == VisionProcessor.Reason.ACCEPTED
        || decision.reason() == VisionProcessor.Reason.NO_TAGS) {
      observed.put(name, captureTimestamp);
    }
  }

  public void reset() {
    accepted.clear();
    observed.clear();
    latency.clear();
    previousObserved.clear();
    previousAccepted.clear();
  }
}
