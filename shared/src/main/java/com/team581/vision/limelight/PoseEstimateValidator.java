package com.team581.vision.limelight;

import com.team581.vision.limelight.LimelightHelpers.PoseEstimate;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import org.jspecify.annotations.Nullable;

public class PoseEstimateValidator {
  // Degrees per second
  public static final double MAX_ANGULAR_VELOCITY = 100;

  private final String name;
  private @Nullable Pose2d previousPose = null;

  public PoseEstimateValidator(String name) {
    this.name = name;
  }

  public boolean shouldTrust(PoseEstimate poseEstimate, double angularVelocity) {
    if (poseEstimate == null) {
      return false;
    }

    if (Math.abs(angularVelocity) > MAX_ANGULAR_VELOCITY) {
      return false;
    }
    if (poseEstimate.tagCount == 0) {
      DogLog.log("Vision/" + name + "/Tags/RawLimelightPose", Pose2d.kZero);
      return false;
    }
    if (poseEstimate.rawFiducials.length == 1) {
      double ambiguity = poseEstimate.rawFiducials[0].ambiguity;
      if (ambiguity >= 0.7) {
        DogLog.timestamp("Vision/" + name + "/Tags/AmbiguityFilter");
        return false;
      }
    }

    var mtPose = poseEstimate.pose;

    // This prevents pose estimator from having crazy poses if the Limelight loses power
    if (mtPose.getX() == 0.0 && mtPose.getY() == 0.0) {
      DogLog.log("Vision/" + name + "/Tags/RawLimelightPose", Pose2d.kZero);
      return false;
    }

    // Limelights sometimes get stuck returning the same pose repeatedly, ignore duplicates
    if (mtPose.equals(previousPose)) {
      DogLog.timestamp("Vision/" + name + "/Tags/DuplicatePoseFilter");
      return false;
    }
    previousPose = mtPose;

    return true;
  }
}
