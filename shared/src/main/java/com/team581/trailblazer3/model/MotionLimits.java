package com.team581.trailblazer3.model;

import static com.google.common.base.Preconditions.checkArgument;

/** Optional ceilings in SI units. Positive infinity delegates a ceiling to the robot model. */
public record MotionLimits(
    double maxVelocityMetersPerSecond,
    double maxAccelerationMetersPerSecondSquared,
    double maxBrakingMetersPerSecondSquared,
    double maxAngularVelocityRadiansPerSecond,
    double maxAngularAccelerationRadiansPerSecondSquared) {
  /** Requires positive ceilings, allowing positive infinity to defer to the robot model. */
  public MotionLimits {
    for (double cap :
        new double[] {
          maxVelocityMetersPerSecond,
          maxAccelerationMetersPerSecondSquared,
          maxBrakingMetersPerSecondSquared,
          maxAngularVelocityRadiansPerSecond,
          maxAngularAccelerationRadiansPerSecondSquared
        }) {
      checkArgument((cap > 0), "Motion ceilings must be positive");
    }
  }

  /** Creates a profile with only a translational speed ceiling in meters per second. */
  public static MotionLimits maxSpeed(double maxVelocityMetersPerSecond) {
    return robotMaximum().withMaxSpeed(maxVelocityMetersPerSecond);
  }

  /** Creates a profile that uses the robot model for every ceiling. */
  public static MotionLimits robotMaximum() {
    double unlimited = Double.POSITIVE_INFINITY;
    return new MotionLimits(unlimited, unlimited, unlimited, unlimited, unlimited);
  }

  /** Returns a copy with a translational acceleration ceiling in meters per second squared. */
  public MotionLimits withMaxAcceleration(double maxAccelerationMetersPerSecondSquared) {
    return new MotionLimits(
        maxVelocityMetersPerSecond,
        maxAccelerationMetersPerSecondSquared,
        maxBrakingMetersPerSecondSquared,
        maxAngularVelocityRadiansPerSecond,
        maxAngularAccelerationRadiansPerSecondSquared);
  }

  /** Returns a copy with an angular acceleration ceiling in radians per second squared. */
  public MotionLimits withMaxAngularAcceleration(
      double maxAngularAccelerationRadiansPerSecondSquared) {
    return new MotionLimits(
        maxVelocityMetersPerSecond,
        maxAccelerationMetersPerSecondSquared,
        maxBrakingMetersPerSecondSquared,
        maxAngularVelocityRadiansPerSecond,
        maxAngularAccelerationRadiansPerSecondSquared);
  }

  /**
   * Returns a copy with an angular acceleration ceiling specified in degrees per second squared.
   */
  public MotionLimits withMaxAngularAccelerationDegrees(
      double maxAngularAccelerationDegreesPerSecondSquared) {
    return withMaxAngularAcceleration(
        Math.toRadians(maxAngularAccelerationDegreesPerSecondSquared));
  }

  /** Returns a copy with a yaw rate ceiling in radians per second. */
  public MotionLimits withMaxAngularSpeed(double maxAngularVelocityRadiansPerSecond) {
    return new MotionLimits(
        maxVelocityMetersPerSecond,
        maxAccelerationMetersPerSecondSquared,
        maxBrakingMetersPerSecondSquared,
        maxAngularVelocityRadiansPerSecond,
        maxAngularAccelerationRadiansPerSecondSquared);
  }

  /** Returns a copy with a yaw rate ceiling specified in degrees per second. */
  public MotionLimits withMaxAngularSpeedDegrees(double maxAngularVelocityDegreesPerSecond) {
    return withMaxAngularSpeed(Math.toRadians(maxAngularVelocityDegreesPerSecond));
  }

  /** Returns a copy with a braking magnitude ceiling in meters per second squared. */
  public MotionLimits withMaxBraking(double maxBrakingMetersPerSecondSquared) {
    return new MotionLimits(
        maxVelocityMetersPerSecond,
        maxAccelerationMetersPerSecondSquared,
        maxBrakingMetersPerSecondSquared,
        maxAngularVelocityRadiansPerSecond,
        maxAngularAccelerationRadiansPerSecondSquared);
  }

  /** Returns a copy with a translational speed ceiling in meters per second. */
  public MotionLimits withMaxSpeed(double maxVelocityMetersPerSecond) {
    return new MotionLimits(
        maxVelocityMetersPerSecond,
        maxAccelerationMetersPerSecondSquared,
        maxBrakingMetersPerSecondSquared,
        maxAngularVelocityRadiansPerSecond,
        maxAngularAccelerationRadiansPerSecondSquared);
  }
}
