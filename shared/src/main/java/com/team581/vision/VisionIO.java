package com.team581.vision;

import com.team581.vision.proto.SpotProtos;
import java.util.List;

/** All implementations drain bounded, owned observations; no latest mutable result. */
public interface VisionIO extends AutoCloseable {
  record Health(
      boolean available,
      String reason,
      SpotProtos.Acknowledgement acknowledgement,
      SpotProtos.Health service) {
    public Health(boolean available, String reason, SpotProtos.Acknowledgement acknowledgement) {
      this(available, reason, acknowledgement, SpotProtos.Health.getDefaultInstance());
    }
  }

  record Sample(SpotProtos.Observation observation, double captureFpgaSeconds) {}

  @Override
  default void close() {}

  List<Sample> drain(double nowFpgaSeconds);

  Health health(double nowFpgaSeconds);

  default void measurementFused(VisionMeasurement measurement) {}

  default void publishDecisions(double now) {}

  default void reportDecision(Sample sample, VisionProcessor.Decision decision) {}

  default void reset(double nowFpgaSeconds) {}
}
