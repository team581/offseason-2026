package com.team581.trailblazer3.model;

public record TimestampedTarget<T>(T value, double timestampSeconds, boolean valid) {
  public boolean fresh(double now) {
    return valid
        && value != null
        && Double.isFinite(timestampSeconds)
        && timestampSeconds <= now + 1e-6
        && now - timestampSeconds <= 0.10;
  }
}
