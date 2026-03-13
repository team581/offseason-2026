package com.team581.trailblazer.trackers;

import com.google.common.collect.ImmutableList;
import com.team581.math.PoseErrorTolerance;
import com.team581.trailblazer.AutoPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.List;
import org.jspecify.annotations.Nullable;

public class HeuristicPathTracker implements PathTracker {
  private final PoseErrorTolerance defaultTransitionTolerance;
  private List<? extends AutoPoint<?>> points = ImmutableList.of();
  private Pose2d currentPose = Pose2d.kZero;
  private int currentPointIndex = 0;

  public HeuristicPathTracker(PoseErrorTolerance defaultTransitionTolerance) {
    this.defaultTransitionTolerance = defaultTransitionTolerance;
  }

  @Override
  public int getCurrentPointIndex() {
    return currentPointIndex;
  }

  @Override
  public Pose2d getTargetPose(@Nullable Rotation2d trackerRotationOverride) {
    if (currentPointIndex < points.size() - 1) {
      var currentPoint = points.get(currentPointIndex);
      var toleranceCheckPose = currentPoint.getPose();

      if (trackerRotationOverride != null) {
        toleranceCheckPose =
            new Pose2d(toleranceCheckPose.getTranslation(), trackerRotationOverride);
      }

      if (currentPoint
          .transitionTolerance()
          .orElse(defaultTransitionTolerance)
          .atPose(toleranceCheckPose, currentPose)) {
        currentPointIndex++;
      }
    }

    var targetPose = points.get(currentPointIndex).getPose();

    if (currentPointIndex > 0) {
      var previousPose = points.get(currentPointIndex - 1).getPose();
      double totalDistance = previousPose.getTranslation().getDistance(targetPose.getTranslation());
      double distanceToTarget =
          currentPose.getTranslation().getDistance(targetPose.getTranslation());

      if (totalDistance > 0) {
        double t = distanceToTarget / totalDistance;
        var interpolatedRotation =
            targetPose.getRotation().interpolate(previousPose.getRotation(), t);
        targetPose = new Pose2d(targetPose.getTranslation(), interpolatedRotation);
      }
    }

    return targetPose;
  }

  @Override
  public void resetAndSetPoints(List<? extends AutoPoint<?>> points) {
    this.points = points;
    this.currentPointIndex = 0;
  }

  @Override
  public void updateRobotState(Pose2d currentPose, ChassisSpeeds currentFieldRelativeRobotSpeeds) {
    this.currentPose = currentPose;
  }
}
