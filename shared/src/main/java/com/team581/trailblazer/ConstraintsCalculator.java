package com.team581.trailblazer;

import com.team581.math.MathHelpers;
import com.team581.math.SlewRateLimiterStateless;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * Helper class for applying velocity constraints to path following outputs.
 *
 * <p>Handles both linear velocity constraints (using slew rate limiting) and angular velocity
 * constraints (using a profiled PID controller).
 */
public class ConstraintsCalculator {
  // Assume we can decelerate at 5m/s/s to allow stopping quickly
  private static final double MAX_DECELERATION = -5;

  private final ProfiledPIDController profiledAngularController;

  private double lastLinearVelocity = 0;
  private double lastLinearVelocityTimestamp = MathSharedStore.getTimestamp();

  /**
   * Creates a new ConstraintsCalculator.
   *
   * @param angularPidConfig PID controller to copy gains from for angular velocity profiling
   */
  public ConstraintsCalculator(PIDController angularPidConfig) {
    this.profiledAngularController =
        new ProfiledPIDController(
            angularPidConfig.getP(),
            angularPidConfig.getI(),
            angularPidConfig.getD(),
            new AutoConstraintOptions().getAngularConstraints());

    this.profiledAngularController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Applies angular velocity constraints using a profiled PID controller.
   *
   * @param desiredAngularVelocity The desired angular velocity in rad/s (used when constraints are
   *     disabled)
   * @param currentAngleRadians The current heading in radians
   * @param targetAngleRadians The target heading in radians
   * @param currentSpeeds The current chassis speeds (used to reset profiled controller state)
   * @param constraints The constraint options to apply
   * @return The constrained angular velocity in rad/s
   */
  public double constrainAngularVelocity(
      double desiredAngularVelocity,
      double currentAngleRadians,
      double targetAngleRadians,
      ChassisSpeeds currentSpeeds,
      AutoConstraintOptions constraints) {
    if (constraints.maxAngularVelocity() > 0) {
      return profiledAngularController.calculate(
          currentAngleRadians,
          new TrapezoidProfile.State(targetAngleRadians, 0),
          constraints.getAngularConstraints());
    }

    // Reset the profiled controller when constraints are not active
    profiledAngularController.reset(currentAngleRadians, currentSpeeds.omegaRadiansPerSecond);
    return desiredAngularVelocity;
  }

  /**
   * Applies linear velocity constraints using slew rate limiting.
   *
   * @param desiredVelocity The desired linear velocity in m/s
   * @param currentSpeeds The current chassis speeds (used to reset state when constraints are
   *     disabled)
   * @param constraints The constraint options to apply
   * @return The constrained linear velocity in m/s
   */
  public double constrainLinearVelocity(
      double desiredVelocity, ChassisSpeeds currentSpeeds, AutoConstraintOptions constraints) {
    double constrainedVelocity;

    if (constraints.maxLinearVelocity() > 0) {
      // First, ensure that the requested velocity goal is within the max velocity
      var clampedVelocity =
          MathUtil.clamp(
              desiredVelocity, -constraints.maxLinearVelocity(), constraints.maxLinearVelocity());

      // Apply acceleration constraints only if acceleration limiting is enabled
      if (constraints.maxLinearAcceleration() > 0) {
        // Slew rate limiter to apply acceleration constraints
        constrainedVelocity =
            SlewRateLimiterStateless.calculate(
                clampedVelocity,
                lastLinearVelocity,
                lastLinearVelocityTimestamp,
                constraints.maxLinearAcceleration(),
                MAX_DECELERATION);
      } else {
        // No acceleration limiting, use clamped velocity directly
        constrainedVelocity = clampedVelocity;
      }

      lastLinearVelocity = constrainedVelocity;
    } else {
      // Reset the state when constraints are not active
      lastLinearVelocity = MathHelpers.getLinearVelocity(currentSpeeds);
      constrainedVelocity = desiredVelocity;
    }

    lastLinearVelocityTimestamp = MathSharedStore.getTimestamp();

    return constrainedVelocity;
  }

  /** Resets the internal state of the constrainer. */
  public void reset() {
    lastLinearVelocity = 0;
    lastLinearVelocityTimestamp = MathSharedStore.getTimestamp();
    profiledAngularController.reset(0, 0);
  }

  /**
   * Resets the internal state of the constrainer to match current robot state.
   *
   * @param currentSpeeds The current chassis speeds
   * @param currentAngleRadians The current heading in radians
   */
  public void reset(ChassisSpeeds currentSpeeds, double currentAngleRadians) {
    lastLinearVelocity = MathHelpers.getLinearVelocity(currentSpeeds);
    lastLinearVelocityTimestamp = MathSharedStore.getTimestamp();
    profiledAngularController.reset(currentAngleRadians, currentSpeeds.omegaRadiansPerSecond);
  }
}
