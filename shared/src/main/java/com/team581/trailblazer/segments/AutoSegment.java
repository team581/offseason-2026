package com.team581.trailblazer.segments;

import com.team581.trailblazer.AngularConstraintOptions;
import com.team581.trailblazer.AutoPoint;
import com.team581.trailblazer.LinearConstraintOptions;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;

public abstract class AutoSegment {
  public final List<AutoPoint<?>> points;
  protected final Optional<LinearConstraintOptions> linearConstraints;
  protected final Optional<AngularConstraintOptions> angularConstraints;
  private final List<HashSet<Enum<?>>> pointToPassedMarkers;

  protected AutoSegment(
      List<AutoPoint<?>> points,
      Optional<LinearConstraintOptions> linearConstraints,
      Optional<AngularConstraintOptions> angularConstraints) {
    this.points = points;
    this.linearConstraints = linearConstraints;
    this.angularConstraints = angularConstraints;
    this.pointToPassedMarkers = new ArrayList<>(points.size());

    var tmp = new HashSet<Enum<?>>();
    for (int i = 0; i < points.size(); i++) {
      if (points.get(i).marker().isPresent()) {
        tmp.add(points.get(i).marker().get());
      }

      pointToPassedMarkers.add(new HashSet<>(tmp));
    }
  }

  /**
   * Check if the robot is done following this segment.
   *
   * @param robotPose The current pose of the robot.
   * @param currentIndex The current index of the point being tracked.
   * @return Whether the robot is done following this segment.
   */
  public abstract boolean atGoal(Pose2d robotPose, int currentIndex);

  /**
   * Check if the robot is done following this segment, ignoring rotation.
   *
   * @param robotTranslation The current translation of the robot.
   * @param currentIndex The current index of the point being tracked.
   * @return Whether the robot is done following this segment.
   */
  public abstract boolean atGoal(Translation2d robotTranslation, int currentIndex);

  /**
   * Resolve the angular constraints for a point belonging to this segment.
   *
   * @param point The point to resolve the constraints for.
   * @return The angular constraints for the point.
   */
  public Optional<AngularConstraintOptions> getAngularConstraints(AutoPoint<?> point) {
    return point.angularConstraints().or(() -> this.angularConstraints);
  }

  /**
   * Resolve the linear constraints for a point belonging to this segment.
   *
   * @param point The point to resolve the constraints for.
   * @return The linear constraints for the point.
   */
  public Optional<LinearConstraintOptions> getLinearConstraints(AutoPoint<?> point) {
    return point.linearConstraints().or(() -> this.linearConstraints);
  }

  public Optional<AutoPoint<?>> lastPoint() {
    if (points.isEmpty()) {
      return Optional.empty();
    }

    return Optional.of(points.get(points.size() - 1));
  }

  public boolean passedMarker(int index, Enum<?> marker) {
    return pointToPassedMarkers.get(index).contains(marker);
  }
}
