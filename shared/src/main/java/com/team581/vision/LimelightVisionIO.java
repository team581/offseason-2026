package com.team581.vision;

import com.google.common.collect.ImmutableList;
import com.team581.vision.results.OptionalTagResult;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/** Snapshot existing Limelight policy; targeting and cluster maps stay with their owners. */
public final class LimelightVisionIO {
  private final String name;
  private final Supplier<OptionalTagResult> source;
  private final DoubleSupplier latencyAdjustmentS;
  private double lastTimestamp = Double.NEGATIVE_INFINITY;

  public LimelightVisionIO(String name, Supplier<OptionalTagResult> source) {
    this(name, source, () -> 0.);
  }

  public LimelightVisionIO(
      String name, Supplier<OptionalTagResult> source, DoubleSupplier latencyAdjustmentS) {
    this.name = name;
    this.source = source;
    this.latencyAdjustmentS = latencyAdjustmentS;
  }

  public List<VisionMeasurement> drain(double now) {
    var result = source.get();
    var measurements = new java.util.ArrayList<VisionMeasurement>();
    result.ifPresent(
        value -> {
          double timestamp = value.timestamp() - latencyAdjustmentS.getAsDouble();
          if (value.timestamp() <= lastTimestamp
              || !Double.isFinite(value.timestamp())
              || !Double.isFinite(timestamp)
              || timestamp > now + .005
              || now - timestamp > .25) {
            return;
          }
          lastTimestamp = value.timestamp();
          var deviations = value.standardDevs();
          try {
            measurements.add(
                new VisionMeasurement(
                    name,
                    name + "/" + lastTimestamp,
                    value.pose(),
                    timestamp,
                    deviations.get(0, 0),
                    deviations.get(1, 0),
                    deviations.get(2, 0),
                    ImmutableList.of(),
                    Double.NaN,
                    Double.NaN));
          } catch (IllegalArgumentException exception) {
            // Adapter does not substitute stale data or mutate the reusable result.
          }
        });
    return List.copyOf(measurements);
  }

  public void reset(double timestamp) {
    lastTimestamp = timestamp;
  }
}
