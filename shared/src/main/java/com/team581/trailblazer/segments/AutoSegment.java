package com.team581.trailblazer.segments;

import com.team581.trailblazer.AutoConstraintOptions;
import com.team581.trailblazer.AutoPoint;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.List;
import java.util.Optional;

public abstract class AutoSegment {
  public final List<AutoPoint> points;
  protected final Optional<AutoConstraintOptions> constraints;

  protected AutoSegment(List<AutoPoint> points, Optional<AutoConstraintOptions> constraints) {
    this.points = points;
    this.constraints = constraints;
  }

  public Optional<AutoPoint> lastPoint() {
    if (points.isEmpty()) {
      return Optional.empty();
    }

    return Optional.of(points.get(points.size() - 1));
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
   * Resolve the constraints for a point belonging to this segment.
   *
   * @param point The point to resolve the constraints for.
   * @return The constraints for the point.
   */
  public Optional<AutoConstraintOptions> getConstraints(AutoPoint point) {
    return point.constraints().or(() -> this.constraints);
  }
}
