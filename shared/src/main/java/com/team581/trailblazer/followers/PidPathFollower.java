package com.team581.trailblazer.followers;

import com.team581.math.MathHelpers;
import com.team581.math.PolarChassisSpeeds;
import com.team581.math.SlewRateLimiterStateless;
import com.team581.trailblazer.AutoConstraintOptions;
import com.team581.trailblazer.AutoPoint;
import com.team581.trailblazer.segments.AutoSegment;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;

public class PidPathFollower implements PathFollower {
  // Assume we can decelerate at 5m/s/s to allow stopping quickly
  private static final double MAX_DECELERATION = -5;

  private final PIDController translationController;
  private final PIDController rotationController;
  private final ProfiledPIDController profiledRotationController;

  private double lastLinearVelocity = 0;
  private double lastLinearVelocityTimestamp = Timer.getFPGATimestamp();

  public PidPathFollower(PIDController translationController, PIDController rotationController) {
    this.translationController = translationController;
    this.rotationController = rotationController;
    this.profiledRotationController =
        new ProfiledPIDController(
            rotationController.getP(),
            rotationController.getI(),
            rotationController.getD(),
            new AutoConstraintOptions().getAngularConstraints());

    this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
    this.profiledRotationController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public ChassisSpeeds calculateSpeeds(
      ChassisSpeeds currentSpeeds,
      Pose2d currentPose,
      Pose2d targetPose,
      AutoPoint currentPoint,
      AutoSegment segment,
      int currentPointIndex) {
    // Get constraints for the current point
    var constraints = segment.getConstraints(currentPoint).orElseGet(AutoConstraintOptions::new);

    var distanceToGoalMeters =
        currentPose.getTranslation().getDistance(targetPose.getTranslation());

    var driveVelocity = Math.abs(translationController.calculate(distanceToGoalMeters, 0));

    var angularVelocity =
        rotationController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    var driveDirection = MathHelpers.getDriveDirection(currentPose, targetPose);

    // Apply velocity constraints
    if (constraints.maxLinearVelocity() > 0) {
      // Use the stateless slew rate limiter for linear velocity
      driveVelocity =
          SlewRateLimiterStateless.calculate(
              driveVelocity,
              lastLinearVelocity,
              lastLinearVelocityTimestamp,
              constraints.maxLinearVelocity(),
              MAX_DECELERATION);

      lastLinearVelocity = driveVelocity;
    } else {
      // Reset the state when constraints are not active
      lastLinearVelocity = MathHelpers.getLinearVelocity(currentSpeeds);
    }
    lastLinearVelocityTimestamp = MathSharedStore.getTimestamp();

    // Cap max angular velocity based on constraints
    if (constraints.maxAngularVelocity() > 0) {
      angularVelocity =
          profiledRotationController.calculate(
              currentPose.getRotation().getRadians(),
              new TrapezoidProfile.State(targetPose.getRotation().getRadians(), 0),
              constraints.getAngularConstraints());
    } else {
      profiledRotationController.reset(
          currentPose.getRotation().getRadians(), currentSpeeds.omegaRadiansPerSecond);
    }

    return new PolarChassisSpeeds(driveVelocity, driveDirection, angularVelocity);
  }
}
