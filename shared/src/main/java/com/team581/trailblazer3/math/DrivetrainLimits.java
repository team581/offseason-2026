package com.team581.trailblazer3.math;

import com.team581.trailblazer3.config.TrailblazerConfig;
import com.team581.trailblazer3.model.EmpiricalDriveCalibration;
import com.team581.trailblazer3.model.MotionLimits;
import com.team581.trailblazer3.model.RobotDrivetrainSpec;
import edu.wpi.first.math.MathUtil;

/**
 * Calculated drivetrain limits with optional segment caps.
 *
 * @param robot source drivetrain dimensions and motor data
 * @param maxSpeed wheel-speed limit capped by the segment, in m/s
 * @param maxWheelSpeed 0.91 times calculated free speed, in m/s
 * @param freeSpeed (nominal voltage - static friction voltage) * motor Kv * wheel radius / drive
 *     ratio
 * @param braking measured deceleration capped by the segment, in m/s²
 * @param friction measured traction acceleration, in m/s²
 * @param lateral 0.85 times traction for cornering, in m/s²
 * @param maxOmega 0.85 * wheel-speed limit / largest module radius, capped by the segment, in rad/s
 * @param maxAlpha 0.85 * mass * min(motor launch acceleration / 0.90, traction) * radius / yaw
 *     inertia, capped by the segment, in rad/s²
 * @param turnRate min(half modeled steer free speed, 0.50 / (lag + period / 2)), in rad/s
 * @param lag measured drive response lag, in seconds
 * @param moduleRadius largest distance from robot center to a module, in meters
 * @param inertiaShare min(1, 4 * yaw inertia / (mass * sum(module radius²)))
 * @param accelerationCeiling segment acceleration cap, in m/s²; infinity uses the motor model
 * @param headingP heading-error gain from empirical calibration, in 1/s
 * @param headingD angular-speed-error gain from empirical calibration, dimensionless
 */
public record DrivetrainLimits(
    RobotDrivetrainSpec robot,
    double maxSpeed,
    double maxWheelSpeed,
    double freeSpeed,
    double braking,
    double friction,
    double lateral,
    double maxOmega,
    double maxAlpha,
    double turnRate,
    double lag,
    double moduleRadius,
    double inertiaShare,
    double accelerationCeiling,
    double headingP,
    double headingD) {
  public static DrivetrainLimits derive(
      RobotDrivetrainSpec robot,
      EmpiricalDriveCalibration calibration,
      double nominalDt,
      MotionLimits caps) {
    return TrailblazerConfig.deriveLimits(robot, calibration, nominalDt, caps);
  }

  /** Applies segment caps to limits derived with the robot maximum profile. */
  public DrivetrainLimits withCaps(MotionLimits caps) {
    return new DrivetrainLimits(
        robot,
        Math.min(maxSpeed, caps.maxVelocityMetersPerSecond()),
        maxWheelSpeed,
        freeSpeed,
        Math.min(braking, caps.maxBrakingMetersPerSecondSquared()),
        friction,
        lateral,
        Math.min(maxOmega, caps.maxAngularVelocityRadiansPerSecond()),
        Math.min(maxAlpha, caps.maxAngularAccelerationRadiansPerSecondSquared()),
        turnRate,
        lag,
        moduleRadius,
        inertiaShare,
        Math.min(accelerationCeiling, caps.maxAccelerationMetersPerSecondSquared()),
        headingP,
        headingD);
  }

  public double acceleration(double speed) {
    return Math.min(
        accelerationCeiling, TrailblazerConfig.motorAcceleration(robot, freeSpeed, speed));
  }

  public double availableTraction(double omega, double alpha) {
    return Math.max(
        0, friction - moduleRadius * Math.hypot(inertiaShare * alpha, 0.12 * omega * omega));
  }

  public double rotationSpeed(double velocity) {
    double share = MathUtil.clamp(1 - velocity / 0.50, 0, 1);
    return Math.min(
        maxOmega,
        Math.min(
            (0.45 + 0.55 * share) * maxWheelSpeed / moduleRadius,
            Math.sqrt(0.8 * (0.40 + 0.60 * share) * friction / (0.12 * moduleRadius))));
  }

  public double rotationAccel(double velocity) {
    double share = MathUtil.clamp(1 - velocity / 0.50, 0, 1);
    return Math.min(maxAlpha, (0.40 + 0.60 * share) * friction / (inertiaShare * moduleRadius));
  }
}
