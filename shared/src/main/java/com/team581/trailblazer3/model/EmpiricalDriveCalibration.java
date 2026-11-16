package com.team581.trailblazer3.model;

import static com.google.common.base.Preconditions.checkArgument;

public record EmpiricalDriveCalibration(
    double frictionLimit, double maxBrakingDeceleration, double driveResponseLagSeconds) {
  public EmpiricalDriveCalibration {
    checkArgument(
        Double.isFinite(frictionLimit)
            && frictionLimit > 0
            && Double.isFinite(maxBrakingDeceleration)
            && maxBrakingDeceleration > 0
            && Double.isFinite(driveResponseLagSeconds)
            && driveResponseLagSeconds >= 0.010
            && driveResponseLagSeconds <= 0.10,
        "Invalid drive calibration");
  }
}
