package com.team581.autos;

import com.team581.math.PoseErrorTolerance;
import com.team581.trailblazer.AutoPoint;
import com.team581.util.FieldUtil;
import com.team581.util.FmsUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public record Point(Pose2d redPose, Pose2d bluePose) {
  public static Point ofRed(Pose2d redPose) {
    return new Point(redPose, FieldUtil.pathflip(redPose));
  }

  public static Point ofBlue(Pose2d bluePose) {
    return new Point(FieldUtil.pathflip(bluePose), bluePose);
  }

  public Pose2d getPose() {
    return FmsUtil.isRedAlliance() ? redPose : bluePose;
  }

  public Translation2d getTranslation() {
    return getPose().getTranslation();
  }

  public double getX() {
    return getPose().getX();
  }

  public double getY() {
    return getPose().getY();
  }

  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public Point plus(Transform2d other) {
    return new Point(redPose.plus(other), bluePose.plus(other));
  }

  public Point transformBy(Transform2d other) {
    return new Point(redPose.transformBy(other), bluePose.transformBy(other));
  }

  public Point rotateBy(Rotation2d other) {
    return new Point(redPose.rotateBy(other), bluePose.rotateBy(other));
  }

  public AutoPoint withLinearConstraints(double maxVelocity, double maxAcceleration) {
    return AutoPoint.of(this).withLinearConstraints(maxVelocity, maxAcceleration);
  }

  public AutoPoint withAngularConstraints(
      double maxAngularVelocity, double maxAngularAcceleration) {
    return AutoPoint.of(this).withAngularConstraints(maxAngularVelocity, maxAngularAcceleration);
  }

  public AutoPoint withTransitionTolerance(PoseErrorTolerance transitionTolerance) {
    return AutoPoint.of(this).withTransitionTolerance(transitionTolerance);
  }
}
