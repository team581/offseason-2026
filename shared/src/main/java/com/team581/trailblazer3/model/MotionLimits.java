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

  public static MotionLimits maxSpeed(double maxVelocityMetersPerSecond) {
    return robotMaximum().withMaxSpeed(maxVelocityMetersPerSecond);
  }

  public static MotionLimits robotMaximum() {
    double unlimited = Double.POSITIVE_INFINITY;
    return new MotionLimits(unlimited, unlimited, unlimited, unlimited, unlimited);
  }

  public MotionLimits withMaxAcceleration(double maxAccelerationMetersPerSecondSquared) {
    return new MotionLimits(
        maxVelocityMetersPerSecond,
        maxAccelerationMetersPerSecondSquared,
        maxBrakingMetersPerSecondSquared,
        maxAngularVelocityRadiansPerSecond,
        maxAngularAccelerationRadiansPerSecondSquared);
  }

  public MotionLimits withMaxAngularAcceleration(
      double maxAngularAccelerationRadiansPerSecondSquared) {
    return new MotionLimits(
        maxVelocityMetersPerSecond,
        maxAccelerationMetersPerSecondSquared,
        maxBrakingMetersPerSecondSquared,
        maxAngularVelocityRadiansPerSecond,
        maxAngularAccelerationRadiansPerSecondSquared);
  }

  public MotionLimits withMaxAngularAccelerationDegrees(
      double maxAngularAccelerationDegreesPerSecondSquared) {
    return withMaxAngularAcceleration(
        Math.toRadians(maxAngularAccelerationDegreesPerSecondSquared));
  }

  public MotionLimits withMaxAngularSpeed(double maxAngularVelocityRadiansPerSecond) {
    return new MotionLimits(
        maxVelocityMetersPerSecond,
        maxAccelerationMetersPerSecondSquared,
        maxBrakingMetersPerSecondSquared,
        maxAngularVelocityRadiansPerSecond,
        maxAngularAccelerationRadiansPerSecondSquared);
  }

  public MotionLimits withMaxAngularSpeedDegrees(double maxAngularVelocityDegreesPerSecond) {
    return withMaxAngularSpeed(Math.toRadians(maxAngularVelocityDegreesPerSecond));
  }

  public MotionLimits withMaxBraking(double maxBrakingMetersPerSecondSquared) {
    return new MotionLimits(
        maxVelocityMetersPerSecond,
        maxAccelerationMetersPerSecondSquared,
        maxBrakingMetersPerSecondSquared,
        maxAngularVelocityRadiansPerSecond,
        maxAngularAccelerationRadiansPerSecondSquared);
  }

  public MotionLimits withMaxSpeed(double maxVelocityMetersPerSecond) {
    return new MotionLimits(
        maxVelocityMetersPerSecond,
        maxAccelerationMetersPerSecondSquared,
        maxBrakingMetersPerSecondSquared,
        maxAngularVelocityRadiansPerSecond,
        maxAngularAccelerationRadiansPerSecondSquared);
  }
}
