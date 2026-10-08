package com.team581.vision;

import com.team581.vision.proto.SpotProtos;
import java.util.ArrayList;
import java.util.List;

/** Shared bounded collection, shadow validation and chronological release. */
public final class VisionPipeline implements AutoCloseable {
  private final VisionIO spot;
  private final SpotProtos.Profile profile;
  private final VisionProcessor processor;
  private final List<LimelightVisionIO> limelights;
  private final MeasurementQueue queue;
  private final VisionDiagnostics diagnostics;
  private String epoch = "";
  private final List<VisionMeasurement> shadow = new ArrayList<>();

  public VisionPipeline(
      VisionIO spot,
      SpotProtos.Profile profile,
      VisionProcessor processor,
      List<LimelightVisionIO> limelights,
      VisionDiagnostics diagnostics) {
    this.spot = spot;
    this.profile = profile;
    this.processor = processor;
    this.limelights = List.copyOf(limelights);
    this.queue = new MeasurementQueue(profile.getReorderWindowS(), 128);
    this.diagnostics = diagnostics;
  }

  @Override
  public void close() {
    spot.close();
  }

  public List<VisionMeasurement> collect(
      double now, boolean fusionEnabled, boolean initialization) {
    shadow.clear();
    var observations = spot.drain(now);
    var health = spot.health(now);
    var ack = health.acknowledgement();
    String current =
        ack.getServiceSession()
            + "/"
            + ack.getRuntimeGeneration()
            + "/"
            + ack.getConnectionEpoch()
            + "/"
            + ack.getSyncEpoch();
    if (!current.equals(epoch)) {
      epoch = current;
      queue.clearPending();
      processor.newGeneration(current);
      if (diagnostics != null) diagnostics.reset();
    }
    if (health.available()) {
      for (var sample : observations) {
        var decision = processor.process(sample, ack, now, initialization);
        spot.reportDecision(sample, decision);
        if (diagnostics != null) {
          diagnostics.record(
              sample.observation().getCameraName(), decision, sample.captureFpgaSeconds(), now);
        }
        decision
            .measurement()
            .ifPresent(
                value -> {
                  shadow.add(value);
                  if (fusionEnabled && profile.getFusionEnabled()) {
                    queue.add(value);
                  }
                });
      }
    }
    for (var limelight : limelights) {
      for (var measurement : limelight.drain(now)) {
        queue.add(measurement);
      }
    }
    if (diagnostics != null) {
      diagnostics.log(now, health, profile);
    }
    return VisionProcessor.accountForCorrelation(
        queue.release(now, 32), Math.max(1, profile.getCamerasCount() + limelights.size()));
  }

  /** Call only after the estimator actually ingests a released measurement. */
  public void measurementFused(VisionMeasurement value) {
    spot.measurementFused(value);
  }

  public void publishDecisions(double now) {
    spot.publishDecisions(now);
  }

  public void reset(double now) {
    queue.reset(now);
    processor.reset(now);
    spot.reset(now);
    if (diagnostics != null) diagnostics.reset();
    for (var limelight : limelights) {
      limelight.reset(now);
    }
  }

  public List<VisionMeasurement> shadowMeasurements() {
    return List.copyOf(shadow);
  }
}
