package com.team581.vision.spot;

import com.team581.vision.spot.data.*;
import com.team581.vision.spot.structs.FrameResult;
import java.util.HexFormat;
import java.util.List;

/** A single owned snapshot per control tick. Getters never poll transport. */
public interface VisionIO extends AutoCloseable {
  record Health(boolean available, String reason, String generation) {}

  record Inputs(Health health, List<Sample> samples) {
    public Inputs {
      samples = List.copyOf(samples);
    }
  }

  record Sample(String cameraName, FrameResult frame, double captureFpgaSeconds) {
    public String captureEpoch() {
      var hex = HexFormat.of();
      return hex.toHexDigits(Long.reverseBytes(frame.captureHigh()))
          + hex.toHexDigits(Long.reverseBytes(frame.captureLow()));
    }
  }

  record TagSelection(long generation, List<Integer> allowedTagIds) {
    public TagSelection {
      allowedTagIds = List.copyOf(allowedTagIds);
      if (generation <= 0
          || !allowedTagIds.equals(allowedTagIds.stream().distinct().sorted().toList()))
        throw new IllegalArgumentException("positive generation and sorted unique IDs required");
    }
  }

  @Override
  default void close() {}

  /** Effective camera availability flags from Spot. */
  default java.util.Optional<Profile> configuration() {
    return java.util.Optional.empty();
  }

  default void logDiagnostics(double now) {}

  Inputs readInputs(double nowFpgaSeconds);

  default void reset(double nowFpgaSeconds) {}

  default void setTagSelection(TagSelection selection) {}
}
