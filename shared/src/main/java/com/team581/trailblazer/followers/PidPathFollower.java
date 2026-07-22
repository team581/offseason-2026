package com.team581.trailblazer.followers;

import com.team581.math.MathHelpers;
import com.team581.math.PolarChassisSpeeds;
import com.team581.trailblazer.AngularConstraintOptions;
import com.team581.trailblazer.AutoPoint;
import com.team581.trailblazer.ConstraintsCalculator;
import com.team581.trailblazer.LinearConstraintOptions;
import com.team581.trailblazer.segments.AutoSegment;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
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
      AutoPoint<?> currentPoint,
      AutoSegment segment,
      int currentPointIndex) {
    // Get constraints for the current point
    var linearConstraints =
        segment.getLinearConstraints(currentPoint).orElseGet(LinearConstraintOptions::new);
    var angularConstraints =
        segment.getAngularConstraints(currentPoint).orElseGet(AngularConstraintOptions::new);

    var distanceToCurrentPoint =
        currentPose.getTranslation().getDistance(targetPose.getTranslation());
    var distanceToEnd = distanceToCurrentPoint;

    // Sum the total linear distance and total angular work remaining through all waypoints in the
    // segment. These are used together to estimate how long the robot has to complete its rotation,
    // which drives a dynamic linear velocity cap to ensure we don't arrive translationally
    // before we finish rotating.
    //
    // For arc segments with a midpoint rotation, we compute angular work through the midpoint
    // (current -> midpoint + midpoint -> final) instead of using shortest-path angleModulus
    // directly to the final rotation. This ensures the velocity cap accounts for the full
    // intended rotation path and prevents the robot from trying to take a shortcut.
    var angularWorkToCurrentPoint = 0.0;
    var currentPointFinalRotation = currentPoint.getPose().getRotation();

    if (currentPoint.arcMidpoint().isPresent()) {
      var midRotation = currentPoint.arcMidpoint().orElseThrow().getRotation();
      // Leg 1: current rotation -> midpoint rotation
      angularWorkToCurrentPoint +=
          Math.abs(
              MathUtil.angleModulus(
                  midRotation.getRadians() - currentPose.getRotation().getRadians()));
      // Leg 2: midpoint rotation -> current waypoint's final rotation
      angularWorkToCurrentPoint +=
          Math.abs(
              MathUtil.angleModulus(
                  currentPointFinalRotation.getRadians() - midRotation.getRadians()));
    } else {
      angularWorkToCurrentPoint +=
          Math.abs(
              MathUtil.angleModulus(
                  targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians()));
    }

    var totalAngularWorkRadians = angularWorkToCurrentPoint;

    // Find total distance and angular work to end of segment
    for (int i = currentPointIndex; i < segment.points.size() - 1; i++) {
      var thisPoint = segment.points.get(i).poseSupplier().get().getPose();
      var nextPoint = segment.points.get(i + 1).getPose();

      distanceToEnd += thisPoint.getTranslation().getDistance(nextPoint.getTranslation());
      totalAngularWorkRadians +=
          Math.abs(
              MathUtil.angleModulus(
                  nextPoint.getRotation().getRadians() - thisPoint.getRotation().getRadians()));
    }

    // Calculate velocities with PID controller
    var linearVelocity = Math.abs(translationController.calculate(distanceToEnd, 0));
    var angularVelocity =
        rotationController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    // Scale down translational speed if we have more rotational work to do than translational work.
    // This ensures we arrive within rotational tolerance before or at the same time as we arrive
    // translationally, preventing the robot from oscillating.
    if (angularConstraints.maxVelocity() > 0) {
      // 1. Cap based on the current point (prevents oscillating at intermediate points with tight
      // tolerances)
      var currentPointToleranceRadians =
          Math.toRadians(
              currentPoint.transitionTolerance().map(t -> t.angularErrorTolerance()).orElse(360.0));
      var currentPointWorkOutsideTolerance =
          Math.max(0, angularWorkToCurrentPoint - currentPointToleranceRadians);

      if (currentPointWorkOutsideTolerance > 0) {
        var expectedRotationTime =
            currentPointWorkOutsideTolerance / angularConstraints.maxVelocity();
        var maxLinearVelocityForRotation = distanceToCurrentPoint / expectedRotationTime;

        // Ensure the cap doesn't drop so low that the robot stalls due to friction.
        // A minimum of 0.4 m/s ensures it keeps moving towards the target before anchoring.
        maxLinearVelocityForRotation = Math.max(maxLinearVelocityForRotation, 0.4);
        linearVelocity = Math.min(linearVelocity, maxLinearVelocityForRotation);

        // If we are waiting for rotation and are already within the linear tolerance,
        // command 0 translational velocity to prevent oscillating around the target point.
        var linearTolerance =
            currentPoint.transitionTolerance().map(t -> t.linearErrorTolerance()).orElse(0.05);
        if (distanceToCurrentPoint <= linearTolerance) {
          linearVelocity = 0;
        }
      }

      // 2. Cap based on the entire segment (paces the segment as a whole)
      var finalPoint = segment.points.get(segment.points.size() - 1);
      var segmentEndToleranceRadians =
          Math.toRadians(
              finalPoint.transitionTolerance().map(t -> t.angularErrorTolerance()).orElse(360.0));
      var segmentWorkOutsideTolerance =
          Math.max(0, totalAngularWorkRadians - segmentEndToleranceRadians);

      if (segmentWorkOutsideTolerance > 0) {
        var expectedRotationTime = segmentWorkOutsideTolerance / angularConstraints.maxVelocity();
        var maxLinearVelocityForRotation = distanceToEnd / expectedRotationTime;

        maxLinearVelocityForRotation = Math.max(maxLinearVelocityForRotation, 0.4);
        linearVelocity = Math.min(linearVelocity, maxLinearVelocityForRotation);

        // Prevent oscillation at the end of the segment while waiting for rotation
        var segmentLinearTolerance =
            finalPoint.transitionTolerance().map(t -> t.linearErrorTolerance()).orElse(0.05);
        if (distanceToEnd <= segmentLinearTolerance) {
          linearVelocity = 0;
        }
      }
    }

    DogLog.log("Trailblazer/Follower/AngularVelocity", angularVelocity);

    // Constrain velocities
    linearVelocity =
        velocityConstrainer.constrainLinearVelocity(
            linearVelocity, currentSpeeds, distanceToEnd, linearConstraints);

    angularVelocity =
        velocityConstrainer.constrainAngularVelocity(
            angularVelocity,
            currentPose.getRotation().getRadians(),
            targetPose.getRotation().getRadians(),
            currentSpeeds,
            angularConstraints);

    DogLog.log("Trailblazer/Follower/AngularLimit", angularConstraints.maxVelocity());
    DogLog.log("Trailblazer/Follower/AngularACCLimit", angularConstraints.maxAcceleration());

    DogLog.log("Trailblazer/Follower/AngularVelocityAfterConstraint", angularVelocity);

    var driveDirection = MathHelpers.getDriveDirection(currentPose, targetPose);

    DogLog.log("Trailblazer/Follower/DriveDirectionDegrees", driveDirection.getDegrees());

    return new PolarChassisSpeeds(linearVelocity, driveDirection, angularVelocity);
  }

  @Override
  public void reset(Pose2d currentPose, ChassisSpeeds currentSpeeds) {
    velocityConstrainer.reset(currentSpeeds, currentPose.getRotation().getRadians());
  }
}
