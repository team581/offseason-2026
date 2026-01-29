package com.team581.trailblazer;

import com.team581.trailblazer.followers.PathFollower;
import com.team581.trailblazer.segments.AutoSegment;
import com.team581.trailblazer.trackers.PathTracker;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.List;
import java.util.Optional;
import org.jspecify.annotations.Nullable;

/**
 * Trailblazer is Team 581's custom path following library. We built Trailblazer to give us end to
 * end control over every aspect of how our autos execute.
 *
 * <p>Trailblazer is made up of a few components:
 *
 * <ol>
 *   <li>Path segments, which are a list of points to follow with the robot
 *   <li>Can include constraints on robot motion per point or per segment
 *   <li>Path trackers, which determine the pose setpoint for the robot
 *   <li>Path followers, which calculate a velocity setpoint to reach the pose setpoint
 * </ol>
 */
public class Trailblazer {
  public static AutoSegmentBuilder segment(AutoPoint... waypoints) {
    return new AutoSegmentBuilder(List.of(waypoints));
  }

  private final PathTracker pathTracker;
  private final PathFollower pathFollower;
  private int currentIndex = -1;
  private Optional<AutoSegment> currentSegment = Optional.empty();

  public Trailblazer(PathTracker pathTracker, PathFollower pathFollower) {
    this.pathTracker = pathTracker;
    this.pathFollower = pathFollower;
  }

  public boolean atGoal(Pose2d currentPose) {
    return currentSegment.filter(segment -> segment.atGoal(currentPose, currentIndex)).isPresent();
  }

  public boolean atGoal(Translation2d currentTranslation) {
    return currentSegment
        .filter(segment -> segment.atGoal(currentTranslation, currentIndex))
        .isPresent();
  }

  public ChassisSpeeds getFieldRelativeSetpoint(
      Pose2d currentPose, ChassisSpeeds currentFieldRelativeSpeeds) {
    return this.getFieldRelativeSetpoint(currentPose, currentFieldRelativeSpeeds, null);
  }

  /**
   * @deprecated I (Jonah) want to delete this. It only really makes sense for robots where swerve
   *     is aware of auto vs teleop, but we migrated off of that via {@link
   *     com.team581.swerve.DriveSource}. Please remind me to delete this in 2027.
   */
  @Deprecated(since = "2026-01-28", forRemoval = true)
  public ChassisSpeeds getFieldRelativeSetpoint(
      Pose2d currentPose,
      ChassisSpeeds currentFieldRelativeSpeeds,
      @Nullable Rotation2d rotationOverride) {
    if (currentSegment.isEmpty()) {
      return new ChassisSpeeds();
    }

    var segment = currentSegment.orElseThrow();

    // Update tracker with current robot state
    pathTracker.updateRobotState(currentPose, currentFieldRelativeSpeeds);

    // Update current index from tracker
    currentIndex = pathTracker.getCurrentPointIndex();
    DogLog.log("Trailblazer/Tracker/CurrentIndex", currentIndex);

    var targetPose = pathTracker.getTargetPose();
    DogLog.log("Trailblazer/Tracker/TargetPose", targetPose);

    if (rotationOverride != null) {
      targetPose = new Pose2d(targetPose.getTranslation(), rotationOverride);
    }

    // Calculate speeds using follower
    return pathFollower.calculateSpeeds(
        currentFieldRelativeSpeeds,
        currentPose,
        targetPose,
        segment.points.get(currentIndex),
        segment,
        currentIndex);
  }

  public void setActiveSegment(AutoSegment segment) {
    if (currentSegment.isPresent() && currentSegment.orElseThrow().equals(segment)) {
      return;
    }

    currentSegment = Optional.of(segment);
    pathTracker.resetAndSetPoints(segment.points);
    currentIndex = pathTracker.getCurrentPointIndex();
  }
}
