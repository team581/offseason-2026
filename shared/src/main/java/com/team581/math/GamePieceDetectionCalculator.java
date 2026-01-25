package com.team581.math;

import com.team581.config.CameraConfig;
import com.team581.vision.results.GamePieceResult;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class GamePieceDetectionCalculator {

  public static Translation2d calculateFieldRelativeTranslationFromCamera(
      Pose2d robotPose,
      com.team581.vision.results.GamePieceResult visionResult,
      CameraConfig cameraConfig) {
    var robotRelativeTranslation =
        calculateRobotRelativeTranslationFromCamera(visionResult, cameraConfig);
    return robotRelativeToFieldRelativeGamePiecePose(robotPose, robotRelativeTranslation);
  }

  public static Translation2d calculateRobotRelativeTranslationFromCamera(
      GamePieceResult visionResult, CameraConfig cameraConfig) {
    double thetaX = Units.degreesToRadians(visionResult.tx());
    double thetaY = Units.degreesToRadians(visionResult.ty());
    double hypot = Math.copySign(Math.hypot(thetaX, thetaY), thetaX);
    double thetaRelativeToCenter = Math.atan(thetaY / thetaX);
    double adjustedRelativeToCenter =
        thetaRelativeToCenter + Units.degreesToRadians(cameraConfig.roll());
    double newThetaX = -1 * (hypot * Math.cos(adjustedRelativeToCenter));
    double newThetaY = hypot * Math.sin(adjustedRelativeToCenter);

    double adjustedThetaY = -Units.degreesToRadians(cameraConfig.pitch()) - newThetaY;

    double forwardOffset;
    if (adjustedThetaY == 0) {
      forwardOffset = Math.abs(-cameraConfig.right());
    } else {
      forwardOffset = (cameraConfig.up() / Math.tan(adjustedThetaY));
    }

    double sidewaysOffset = forwardOffset * Math.tan(newThetaX);

    var cameraRelativeTranslation = new Translation2d(forwardOffset, sidewaysOffset);
    return cameraRelativeTranslation
        .rotateBy(Rotation2d.fromDegrees(cameraConfig.yaw()))
        .plus(new Translation2d(cameraConfig.forward(), -cameraConfig.right()));
  }

  public static Translation2d robotRelativeToFieldRelativeGamePiecePose(
      Pose2d robotPose, Translation2d robotRelativeGamePiecePose) {
    return robotRelativeGamePiecePose
        .rotateBy(robotPose.getRotation())
        .plus(robotPose.getTranslation());
  }

  private GamePieceDetectionCalculator() {}
}
