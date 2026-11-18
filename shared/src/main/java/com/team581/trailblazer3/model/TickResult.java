package com.team581.trailblazer3.model;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.Optional;

public record TickResult(
    ChassisSpeeds commandedFieldSpeeds,
    RuntimeStatus status,
    boolean infeasible,
    LimitingFactor limitingFactor,
    long geometryGeneration,
    double progress,
    String diagnostic,
    Optional<VelocityStep> velocityStep) {
  public TickResult(
      ChassisSpeeds commandedFieldSpeeds,
      RuntimeStatus status,
      boolean infeasible,
      LimitingFactor limitingFactor,
      long geometryGeneration,
      double progress,
      String diagnostic) {
    this(
        commandedFieldSpeeds,
        status,
        infeasible,
        limitingFactor,
        geometryGeneration,
        progress,
        diagnostic,
        Optional.empty());
  }

  /** The interval over which the follower limited the change to commandedFieldSpeeds. */
  public record VelocityStep(
      ChassisSpeeds startingFieldSpeeds, double timestampSeconds, double durationSeconds) {}

  public enum RuntimeStatus {
    IDLE,
    RUNNING,
    AT_GOAL
  }

  public enum LimitingFactor {
    NONE,
    TOP_SPEED,
    CORNER_CENTRIPETAL,
    GOAL_DECELERATION,
    HEADING_HARD_GATE,
    MOTOR_ACCELERATION,
    MODULE_SPEED,
    VECTOR_SLEW_LIMIT,
    INFEASIBLE
  }
}
