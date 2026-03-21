package com.team581.trailblazer;

import com.team581.math.PoseErrorTolerance;
import com.team581.trailblazer.segments.AutoSegment;
import com.team581.trailblazer.segments.AutoSegmentCustomEnd;
import com.team581.trailblazer.segments.AutoSegmentForever;
import com.team581.trailblazer.segments.AutoSegmentLastPointEnd;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.Optional;

public class AutoSegmentBuilder {
  private final List<AutoPoint<?>> points;
  private Optional<LinearConstraintOptions> linearConstraints = Optional.empty();
  private Optional<AngularConstraintOptions> angularConstraints = Optional.empty();

  AutoSegmentBuilder(List<AutoPoint<?>> waypoints) {
    this.points = waypoints;
  }

  /**
   * Builds the segment, which will be followed indefinitely.
   *
   * @return The segment.
   */
  public AutoSegment forever() {
    return new AutoSegmentForever(points, linearConstraints, angularConstraints);
  }

  /**
   * Builds the segment, which will be followed until the robot is within the tolerance of the last
   * point.
   *
   * @return The segment.
   */
  public AutoSegment untilFinished() {
    if (points.isEmpty()) {
      return forever();
    }

    var lastPoint = points.get(points.size() - 1);

    if (lastPoint.transitionTolerance().isEmpty()) {
      throw new NoSuchElementException(
          "Last point is missing a transition tolerance, but segment is trying to be built with untilFinished()");
    }

    return new AutoSegmentLastPointEnd(points, linearConstraints, angularConstraints);
  }

  /**
   * Builds the segment, which will be followed until the robot is within the tolerance of the last
   * point.
   *
   * @param finishedTolerance The tolerance for the segment to be considered finished.
   * @return The segment.
   */
  public AutoSegment untilFinished(PoseErrorTolerance finishedTolerance) {
    if (points.isEmpty()) {
      return forever();
    }

    return new AutoSegmentCustomEnd(
        points, linearConstraints, angularConstraints, finishedTolerance);
  }

  /**
   * Set the angular constraints for the segment.
   *
   * @param maxAngularVelocity The maximum angular velocity in radians per second.
   * @param maxAngularAcceleration The maximum angular acceleration in radians per second squared.
   * @return This builder.
   */
  public AutoSegmentBuilder withAngularConstraints(
      double maxAngularVelocity, double maxAngularAcceleration) {
    this.angularConstraints =
        Optional.of(new AngularConstraintOptions(maxAngularVelocity, maxAngularAcceleration));
    return this;
  }

  /**
   * Set the linear constraints for the segment.
   *
   * @param maxVelocity The maximum velocity in meters per second.
   * @param maxAcceleration The maximum acceleration in meters per second squared.
   * @return This builder.
   */
  public AutoSegmentBuilder withLinearConstraints(double maxVelocity, double maxAcceleration) {
    this.linearConstraints = Optional.of(new LinearConstraintOptions(maxVelocity, maxAcceleration));
    return this;
  }
}
