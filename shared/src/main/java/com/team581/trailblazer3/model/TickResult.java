package com.team581.trailblazer3.model;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public record TickResult(
    ChassisSpeeds commandedFieldSpeeds,
    RuntimeStatus status,
    boolean infeasible,
    LimitingFactor limitingFactor,
    long geometryGeneration,
    double progress,
    String diagnostic) {
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
