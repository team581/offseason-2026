package com.team581.vision.limelight;

import com.team581.vision.limelight.LimelightHelpers.PoseEstimate;
import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public class PoseEstimateValidator {
  // Degrees per second
  public static final DoubleSubscriber MAX_ANGULAR_VELOCITY =
      DogLog.tunable("Limelight/MaxTagAngularRate", 100.0);

  private final String name;

  // Duplicate-frame detection is tracked per pose source (MT1 vs MT2). The two
  // sources produce different poses from the same underlying frame, so sharing
  // a single previous-pose field would let stale frames slip through the
  // duplicate filter for whichever source was checked second.
  private double previousTimestampMt1 = Double.NEGATIVE_INFINITY;
  private double previousTimestampMt2 = Double.NEGATIVE_INFINITY;

  public PoseEstimateValidator(String name) {
    this.name = name;
  }

  public boolean shouldTrust(PoseEstimate poseEstimate, double angularVelocity) {
    if (poseEstimate == null) {
      return false;
    }

    if (Math.abs(angularVelocity) > MAX_ANGULAR_VELOCITY.getAsDouble()) {
      return false;
    }
    if (poseEstimate.tagCount == 0) {
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
      return false;
    }

    // Limelights sometimes get stuck returning the same frame repeatedly. The
    // frame timestamp is the most reliable signal that we're looking at the
    // same data we already consumed.
    double timestamp = poseEstimate.timestampSeconds;
    if (poseEstimate.isMegaTag2) {
      if (timestamp <= previousTimestampMt2) {
        DogLog.timestamp("Vision/" + name + "/Tags/DuplicatePoseFilter");
        return false;
      }
      previousTimestampMt2 = timestamp;
    } else {
      if (timestamp <= previousTimestampMt1) {
        DogLog.timestamp("Vision/" + name + "/Tags/DuplicatePoseFilter");
        return false;
      }
      previousTimestampMt1 = timestamp;
    }

    return true;
  }
}
