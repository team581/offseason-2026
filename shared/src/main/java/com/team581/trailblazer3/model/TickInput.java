package com.team581.trailblazer3.model;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Map;
import java.util.Optional;

public record TickInput(
    double timestampSeconds,
    RobotStateSnapshot robotState,
    Map<String, TimestampedTarget<Pose2d>> resolvedGoalTargets,
    Map<String, TimestampedTarget<Translation2d>> resolvedAimTargets,
    Optional<Translation2d> driverFieldVelocityMpS) {
  public TickInput {
    resolvedGoalTargets = Map.copyOf(resolvedGoalTargets);
    resolvedAimTargets = Map.copyOf(resolvedAimTargets);
  }
}
