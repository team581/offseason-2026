package com.team581.trailblazer.followers;

import com.team581.math.MathHelpers;
import com.team581.math.PolarChassisSpeeds;
import com.team581.trailblazer.AutoConstraintOptions;
import com.team581.trailblazer.AutoPoint;
import com.team581.trailblazer.ConstraintsCalculator;
import com.team581.trailblazer.segments.AutoSegment;
import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

public class PidPathFollower implements PathFollower {
  // 0-1 scalar
  private static final double AGGRESSIVENESS_FACTOR = 0.5;
  private final PIDController translationController;
  private final PIDController rotationController;
  private final ConstraintsCalculator velocityConstrainer;

  private double lastTimestamp = 0.0;
  private double lastVelocity = 0.0;
  private double lastCommandedVelocity = 0.0;

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

    for (int i = currentPointIndex; i < segment.points.size() - 1; i++) {
      distanceToGoalMeters +=
          segment
              .points
              .get(i)
              .poseSupplier()
              .get()
              .getPose()
              .getTranslation()
              .getDistance(
                  segment.points.get(i + 1).poseSupplier().get().getPose().getTranslation());
    }

    var wantedDriveVelocity = Math.abs(translationController.calculate(distanceToGoalMeters, 0));
    DogLog.log("Trailblazer/Follower/WantedVelocity", wantedDriveVelocity);
    var currentVelocity =
        Math.hypot(currentSpeeds.vxMetersPerSecond, currentSpeeds.vyMetersPerSecond);
    DogLog.log("Trailblazer/Follower/CurrentVelocity", currentVelocity);

    // Calculate how much we can accelerate this loop
    // New Velocity = Old Velocity + (Acceleration * dt)
    var kDt = Math.min(Timer.getFPGATimestamp() - lastTimestamp, 0.04);
    var currentAccel = (currentVelocity - lastVelocity) / kDt;
    DogLog.log("Trailblazer/Follower/CurrentAccel", currentAccel);

    lastTimestamp = Timer.getFPGATimestamp();

    var maxReachableVelocity =
        Math.max((lastCommandedVelocity + (constraints.maxLinearAcceleration() * kDt)), 0.5);

    // Calculate velocity needed to stop in time
    var maxStoppingVelocity =
        Math.sqrt(2 * constraints.maxLinearAcceleration() * distanceToGoalMeters);

    // Apply constraints
    var maxVelocityLimitedWantedVelocity =
        Math.min(wantedDriveVelocity, constraints.maxLinearVelocity());

    var linearVelocity = Math.min(maxVelocityLimitedWantedVelocity, maxReachableVelocity);
    linearVelocity = Math.min(linearVelocity, maxStoppingVelocity);

    var angularVelocity =
        rotationController.calculate(
            currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians());

    angularVelocity =
        velocityConstrainer.constrainAngularVelocity(
            angularVelocity,
            currentPose.getRotation().getRadians(),
            targetPose.getRotation().getRadians(),
            currentSpeeds,
            constraints);

    var driveDirection = MathHelpers.getDriveDirection(currentPose, targetPose);

    if (currentVelocity > 0.2 && distanceToGoalMeters > 0.1) {
      var currentDirection = MathHelpers.getDriveDirection(currentSpeeds);
      driveDirection = currentDirection.interpolate(driveDirection, AGGRESSIVENESS_FACTOR);
    }

    lastVelocity = currentVelocity;
    lastCommandedVelocity = linearVelocity;
    return new PolarChassisSpeeds(linearVelocity, driveDirection, angularVelocity);
  }

  @Override
  public void reset(ChassisSpeeds currentSpeeds, double currentAngleRadians) {
    velocityConstrainer.reset(currentSpeeds, currentAngleRadians);
    lastCommandedVelocity = MathHelpers.getLinearVelocity(currentSpeeds);
    lastVelocity = lastCommandedVelocity;
    lastTimestamp = Timer.getFPGATimestamp();
  }
}
