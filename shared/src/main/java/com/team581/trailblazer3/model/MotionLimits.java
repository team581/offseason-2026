package com.team581.trailblazer3.model;

/** Optional ceilings in SI units. Positive infinity delegates a ceiling to the robot model. */
public record MotionLimits(
    double maxVelocityMetersPerSecond,
    double maxAccelerationMetersPerSecondSquared,
    double maxBrakingMetersPerSecondSquared,
    double maxAngularVelocityRadiansPerSecond,
    double maxAngularAccelerationRadiansPerSecondSquared) {
  public MotionLimits {
    for (double cap :
        new double[] {
          maxVelocityMetersPerSecond,
          maxAccelerationMetersPerSecondSquared,
          maxBrakingMetersPerSecondSquared,
          maxAngularVelocityRadiansPerSecond,
          maxAngularAccelerationRadiansPerSecondSquared
        }) {
      if (!(cap > 0)) {
        throw new IllegalArgumentException("Motion ceilings must be positive");
      }
    }
  }

  public static MotionLimits robotMaximum() {
    double unlimited = Double.POSITIVE_INFINITY;
    return new MotionLimits(unlimited, unlimited, unlimited, unlimited, unlimited);
  }
}
