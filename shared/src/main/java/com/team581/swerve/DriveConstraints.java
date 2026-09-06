package com.team581.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DriveConstraints {

  private static final double MIN_DISTANCE_SQUARED_METERS = 1e-12;

  /**
   * Scales the driver's requested field-relative speeds to respect radial and tangential limits
   *
   * @param requestedSpeeds Field-relative requested speeds
   * @param robotTranslation Current field-relative position of the robot
   * @param goalTranslation Field-relative position of the goal
   * @return A scaled requested chassis speeds
   */
  public static ChassisSpeeds getGoalCentricConstrainedSpeeds(
      ChassisSpeeds requestedSpeeds,
      Translation2d robotTranslation,
      Translation2d goalTranslation,
      double maxRadialVelocity,
      double maxTangentialVelocity) {

    Rotation2d angleToGoal = goalTranslation.minus(robotTranslation).getAngle();

    Translation2d requestedVelocity =
        new Translation2d(requestedSpeeds.vxMetersPerSecond, requestedSpeeds.vyMetersPerSecond);

    // X becomes radial velocity (towards/away), Y becomes tangential (strafing)
    Translation2d goalCentricVelocity = requestedVelocity.rotateBy(angleToGoal.unaryMinus());

    // Calculate how much we are exceeding the radial limit
    double radialScale = 1.0;
    if (Math.abs(goalCentricVelocity.getX()) > maxRadialVelocity) {
      radialScale = maxRadialVelocity / Math.abs(goalCentricVelocity.getX());
    }

    // Calculate how much we are exceeding the tangential limit
    double tangentialScale = 1.0;
    if (Math.abs(goalCentricVelocity.getY()) > maxTangentialVelocity) {
      tangentialScale = maxTangentialVelocity / Math.abs(goalCentricVelocity.getY());
    }

    // Find the most restrictive scale factor
    double desaturationFactor = Math.min(radialScale, tangentialScale);

    // Scale the requested velocity by the desaturation factor
    Translation2d safeVelocity = requestedVelocity.times(desaturationFactor);

    return new ChassisSpeeds(
        safeVelocity.getX(), safeVelocity.getY(), requestedSpeeds.omegaRadiansPerSecond);
  }

  /** Limits a turret velocity target to the mechanism's velocity and one-cycle acceleration. */
  public static double getLimitedTurretTrackingAngularVelocity(
      double requestedTurretVelocityRadiansPerSecond,
      double measuredTurretVelocityRadiansPerSecond,
      double maxTurretVelocityRadiansPerSecond,
      double maxTurretAccelerationRadiansPerSecondSquared,
      double periodSeconds) {
    double velocityLimit = Math.abs(maxTurretVelocityRadiansPerSecond);
    double accelerationDelta =
        Math.abs(maxTurretAccelerationRadiansPerSecondSquared) * Math.max(0.0, periodSeconds);
    double minimumAllowedVelocity =
        Math.max(-velocityLimit, measuredTurretVelocityRadiansPerSecond - accelerationDelta);
    double maximumAllowedVelocity =
        Math.min(velocityLimit, measuredTurretVelocityRadiansPerSecond + accelerationDelta);

    if (minimumAllowedVelocity > maximumAllowedVelocity) {
      return Math.copySign(velocityLimit, measuredTurretVelocityRadiansPerSecond);
    }

    return MathUtil.clamp(
        requestedTurretVelocityRadiansPerSecond, minimumAllowedVelocity, maximumAllowedVelocity);
  }

  /**
   * Scales a requested field-relative chassis command so tracking the target remains within the
   * turret's velocity and acceleration capabilities.
   *
   * <p>All three chassis components are scaled together, preserving the driver's requested blend of
   * translation and rotation. Acceleration is limited relative to the measured turret velocity,
   * which also handles cases where the drivetrain or turret has not yet caught up to a prior
   * request.
   */
  public static ChassisSpeeds getTurretConstrainedSpeeds(
      ChassisSpeeds requestedFieldRelativeSpeeds,
      Pose2d robotPose,
      Translation2d goalTranslation,
      Translation2d robotToTurret,
      double measuredTurretVelocityRadiansPerSecond,
      double maxTurretVelocityRadiansPerSecond,
      double maxTurretAccelerationRadiansPerSecondSquared,
      double periodSeconds) {
    double requestedTurretVelocity =
        getTurretTrackingAngularVelocity(
            requestedFieldRelativeSpeeds, robotPose, goalTranslation, robotToTurret);
    double limitedTurretVelocity =
        getLimitedTurretTrackingAngularVelocity(
            requestedTurretVelocity,
            measuredTurretVelocityRadiansPerSecond,
            maxTurretVelocityRadiansPerSecond,
            maxTurretAccelerationRadiansPerSecondSquared,
            periodSeconds);

    if (limitedTurretVelocity == requestedTurretVelocity) {
      return requestedFieldRelativeSpeeds;
    }

    if (Math.abs(requestedTurretVelocity) < 1e-12) {
      return new ChassisSpeeds();
    }

    double scale = limitedTurretVelocity / requestedTurretVelocity;

    // Scaling toward zero can only produce turret rates between zero and the requested rate. When
    // the allowed window is outside that interval, use whichever endpoint is closest.
    if (!Double.isFinite(scale)) {
      scale = 0.0;
    } else {
      scale = MathUtil.clamp(scale, 0.0, 1.0);
    }

    return new ChassisSpeeds(
        requestedFieldRelativeSpeeds.vxMetersPerSecond * scale,
        requestedFieldRelativeSpeeds.vyMetersPerSecond * scale,
        requestedFieldRelativeSpeeds.omegaRadiansPerSecond * scale);
  }

  /**
   * Calculates the velocity the turret must have relative to the robot to keep pointing at a
   * stationary field target.
   *
   * <p>This uses the velocity of the turret pivot, not the velocity of the robot center. The pivot
   * velocity includes the tangential velocity caused by the chassis rotating about its center.
   * Positive velocity is counterclockwise, matching WPILib's angular convention.
   *
   * @param fieldRelativeSpeeds Field-relative chassis speeds at the robot center
   * @param robotPose Current field-relative robot pose
   * @param goalTranslation Field-relative position of the target
   * @param robotToTurret Translation from the robot center to the turret pivot, in robot
   *     coordinates
   * @return Required turret velocity relative to the robot in radians per second
   */
  public static double getTurretTrackingAngularVelocity(
      ChassisSpeeds fieldRelativeSpeeds,
      Pose2d robotPose,
      Translation2d goalTranslation,
      Translation2d robotToTurret) {
    Translation2d fieldRelativeOffset = robotToTurret.rotateBy(robotPose.getRotation());
    Translation2d turretTranslation = robotPose.getTranslation().plus(fieldRelativeOffset);
    Translation2d turretToGoal = goalTranslation.minus(turretTranslation);
    double distanceSquared = turretToGoal.getNorm() * turretToGoal.getNorm();

    if (distanceSquared < MIN_DISTANCE_SQUARED_METERS) {
      return -fieldRelativeSpeeds.omegaRadiansPerSecond;
    }

    double omega = fieldRelativeSpeeds.omegaRadiansPerSecond;
    double turretVelocityX =
        fieldRelativeSpeeds.vxMetersPerSecond - omega * fieldRelativeOffset.getY();
    double turretVelocityY =
        fieldRelativeSpeeds.vyMetersPerSecond + omega * fieldRelativeOffset.getX();

    // The bearing from the moving turret pivot to a stationary goal changes at
    // -cross(turretToGoal, turretVelocity) / distance^2. The turret is robot-relative, so chassis
    // yaw must also be subtracted.
    double fieldBearingRate =
        -(turretToGoal.getX() * turretVelocityY - turretToGoal.getY() * turretVelocityX)
            / distanceSquared;
    return fieldBearingRate - omega;
  }
}
