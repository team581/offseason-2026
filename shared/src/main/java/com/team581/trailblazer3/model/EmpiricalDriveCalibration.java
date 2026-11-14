package com.team581.trailblazer3.model;

public record EmpiricalDriveCalibration(
    double frictionLimit, double maxBrakingDeceleration, double driveResponseLagSeconds) {
  public EmpiricalDriveCalibration {
    if (!Double.isFinite(frictionLimit)
        || frictionLimit <= 0
        || !Double.isFinite(maxBrakingDeceleration)
        || maxBrakingDeceleration <= 0
        || !Double.isFinite(driveResponseLagSeconds)
        || driveResponseLagSeconds < 0.010
        || driveResponseLagSeconds > 0.10) {
      throw new IllegalArgumentException("Invalid drive calibration");
    }
  }
}
