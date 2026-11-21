package com.team581.trailblazer3.model;

import static com.google.common.base.Preconditions.checkArgument;

/**
 * Carpet measurements and heading feedback gains; use the same drive request as the autos.
 *
 * @param frictionLimit acceleration before wheel slip, in m/s²; estimate from measured speed change
 *     divided by elapsed time during an acceleration test
 * @param maxBrakingDeceleration braking magnitude in m/s²; measure speed decrease / elapsed time
 * @param driveResponseLagSeconds response time in seconds; fit the speed-step response with the
 *     preview plant's first-order lag model
 * @param headingP heading-error gain in 1/s; tune tracking on carpet
 * @param headingD angular-speed-error gain, dimensionless; tune damping on carpet
 */
public record EmpiricalDriveCalibration(
    double frictionLimit,
    double maxBrakingDeceleration,
    double driveResponseLagSeconds,
    double headingP,
    double headingD) {
  /** Keeps the existing heading gains when only carpet motion measurements are supplied. */
  public EmpiricalDriveCalibration(
      double frictionLimit, double maxBrakingDeceleration, double driveResponseLagSeconds) {
    this(frictionLimit, maxBrakingDeceleration, driveResponseLagSeconds, 8, 0.4);
  }

  public EmpiricalDriveCalibration {
    checkArgument(
        Double.isFinite(frictionLimit)
            && frictionLimit > 0
            && Double.isFinite(maxBrakingDeceleration)
            && maxBrakingDeceleration > 0
            && Double.isFinite(driveResponseLagSeconds)
            && driveResponseLagSeconds >= 0.010
            && driveResponseLagSeconds <= 0.10
            && Double.isFinite(headingP)
            && headingP >= 0
            && Double.isFinite(headingD)
            && headingD >= 0,
        "Invalid drive calibration");
  }
}
