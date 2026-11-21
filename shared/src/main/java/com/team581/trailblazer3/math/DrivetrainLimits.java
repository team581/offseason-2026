package com.team581.trailblazer3.math;

import com.team581.trailblazer3.config.TrailblazerConfig;
import com.team581.trailblazer3.model.EmpiricalDriveCalibration;
import com.team581.trailblazer3.model.MotionLimits;
import com.team581.trailblazer3.model.RobotDrivetrainSpec;
import edu.wpi.first.math.MathUtil;

/**
 * Calculated drivetrain limits with optional segment caps.
 *
 * @param robot source physical drivetrain specs (dimensions, mass, inertia, gear ratios, and motor
 *     models); not a derived scalar limit
 * @param maxSpeed maximum chassis translation speed allowed on this segment, in m/s; {@code
 *     min(maxWheelSpeed, segment cap)}, bounding how fast the robot center can translate across the
 *     field
 * @param maxWheelSpeed usable top wheel surface speed, in m/s; {@code 0.91 * freeSpeed}, reserving
 *     a 9% voltage/load margin so swerve modules can still steer and combine translation with
 *     rotation without saturating
 * @param freeSpeed theoretical unloaded top wheel surface speed, in m/s; {@code (nominal voltage -
 *     static friction voltage) * motor Kv * wheel radius / drive ratio}, representing the drive
 *     motors' zero-load speed at nominal battery voltage and setting the back-EMF scale for motor
 *     acceleration
 * @param braking maximum linear deceleration magnitude, in m/s²; {@code min(measured braking,
 *     segment cap)}, bounding how hard the robot can slow down for corners, goals, or barriers
 *     without skidding or overshooting
 * @param friction carpet traction acceleration envelope, in m/s²; measured acceleration limit
 *     before the tires break traction and slip (shared across translation, cornering, and rotation)
 * @param lateral maximum cornering (centripetal) acceleration, in m/s²; {@code 0.85 * friction},
 *     capping path corner speed ({@code v² * curvature <= lateral}) with a 15% traction margin so
 *     the robot does not slide wide in turns
 * @param maxOmega maximum chassis yaw rotation speed around the robot center, in rad/s; {@code
 *     min(0.85 * maxWheelSpeed / moduleRadius, segment cap)}, keeping the outermost module within
 *     85% of {@code maxWheelSpeed} during pure spinning
 * @param maxAlpha maximum chassis yaw angular acceleration, in rad/s²; {@code min(0.85 * mass *
 *     min(motor launch acceleration / 0.90, friction) * moduleRadius / yaw inertia, segment cap)},
 *     bounding how quickly the robot can spin up or slow its rotation given its yaw inertia, module
 *     lever arm, and available motor/tire force
 * @param turnRate maximum path-direction change rate for curved paths, in rad/s; {@code min(0.5 *
 *     modeled steer free speed, 0.50 / (lag + period / 2))}, capping corner speed ({@code v *
 *     curvature <= turnRate}) so the swerve steer modules and drive response lag can keep up with
 *     tight turns
 * @param lag measured first-order drive response time constant, in seconds; characterizes how long
 *     wheel speed takes to respond to a velocity command, used for lookahead prediction and
 *     feedback gain sizing
 * @param moduleRadius largest distance from the robot center to any swerve module center, in
 *     meters; the longest rotational lever arm, converting between chassis yaw rate ({@code rad/s})
 *     and outermost module linear speed ({@code omega * moduleRadius} in {@code m/s})
 * @param inertiaShare yaw-inertia-to-module-inertia ratio, dimensionless ({@code 0} to {@code 1});
 *     {@code min(1, 4 * yaw inertia / (mass * sum(module radius²)))}, scaling how much tire
 *     traction is consumed by angular acceleration ({@code inertiaShare * moduleRadius * alpha})
 *     based on how concentrated the robot's mass is relative to its module footprint
 * @param accelerationCeiling optional segment cap on forward linear acceleration, in m/s²; {@code
 *     Double.POSITIVE_INFINITY} defers to the speed-dependent drive motor torque model and carpet
 *     traction
 * @param headingP proportional heading-error feedback gain, in 1/s ({@code (rad/s) / rad});
 *     commands corrective chassis angular velocity per radian of heading error
 * @param headingD derivative angular-velocity-error damping gain, dimensionless ({@code (rad/s) /
 *     (rad/s)}); scales corrective spin opposing angular-rate error to damp heading overshoot
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
