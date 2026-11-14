package com.team581.trailblazer3.model;

/** A speed ceiling between two distances along the active stop-bounded horizon. */
public record SpeedZone(double startMeters, double endMeters, double maxSpeed) {
  public SpeedZone {
    if (!Double.isFinite(startMeters)
        || !Double.isFinite(endMeters)
        || !Double.isFinite(maxSpeed)
        || startMeters < 0
        || endMeters <= startMeters
        || maxSpeed <= 0) {
      throw new IllegalArgumentException("Invalid speed zone");
    }
  }
}
