package com.team581.trailblazer3.model;

import edu.wpi.first.math.geometry.Pose2d;
import java.util.Objects;

public sealed interface TargetLossPolicy {
  record ControlledStop() implements TargetLossPolicy {}

  record FallbackNominal(Pose2d nominalPose) implements TargetLossPolicy {
    public FallbackNominal {
      Objects.requireNonNull(nominalPose);
    }
  }

  record HoldLastValid(double graceSeconds) implements TargetLossPolicy {
    public HoldLastValid {
      if (!Double.isFinite(graceSeconds) || graceSeconds < 0) {
        throw new IllegalArgumentException("Invalid target-loss grace period");
      }
    }
  }

  static TargetLossPolicy controlledStop() {
    return new ControlledStop();
  }

  static TargetLossPolicy fallbackNominal(Pose2d pose) {
    return new FallbackNominal(pose);
  }

  static TargetLossPolicy holdLastValid(double seconds) {
    return new HoldLastValid(seconds);
  }
}
