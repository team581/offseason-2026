package com.team581.math;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class IntakeAssistCalculator {

  public static PolarChassisSpeeds getAssistSpeedsFromPose(
      Pose2d target, Pose2d robotPose, PIDController controller, double maxSpeed) {
    var robotRelativePose =
        target
            .getTranslation()
            .minus(robotPose.getTranslation())
            .rotateBy(Rotation2d.fromDegrees(360 - robotPose.getRotation().getDegrees()));
    var sidewaysSpeed = MathUtil.clamp(controller.calculate(robotRelativePose.getY(), 0.0), -maxSpeed, maxSpeed);
    var robotRelativeError = new Translation2d(0, sidewaysSpeed);
    var fieldRelativeError = robotRelativeError.rotateBy(robotPose.getRotation());
    var assistSpeeds = new PolarChassisSpeeds(fieldRelativeError.getX(), fieldRelativeError.getY(), 0.0);

    return assistSpeeds;
  }
}
