package com.team581.trailblazer.followers;

import com.team581.math.MathHelpers;
import com.team581.math.PolarChassisSpeeds;
import com.team581.trailblazer.AutoConstraintOptions;
import com.team581.trailblazer.AutoPoint;
import com.team581.trailblazer.ConstraintsCalculator;
import com.team581.trailblazer.segments.AutoSegment;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class PidPathFollower implements PathFollower {
  private final PIDController translationController;
  private final PIDController rotationController;
  private final ConstraintsCalculator velocityConstrainer;

  public PidPathFollower(PIDController translationController, PIDController rotationController) {
    this.translationController = translationController;
    this.rotationController = rotationController;
    this.velocityConstrainer = new ConstraintsCalculator(rotationController);

    this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
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
    driveVelocity =
        velocityConstrainer.constrainLinearVelocity(driveVelocity, currentSpeeds, constraints);

    angularVelocity =
        velocityConstrainer.constrainAngularVelocity(
            angularVelocity,
            currentPose.getRotation().getRadians(),
            targetPose.getRotation().getRadians(),
            currentSpeeds,
            constraints);

    return new PolarChassisSpeeds(driveVelocity, driveDirection, angularVelocity);
  }

  @Override
  public void reset(ChassisSpeeds currentSpeeds, double currentAngleRadians) {
    velocityConstrainer.reset(currentSpeeds, currentAngleRadians);
  }
}
