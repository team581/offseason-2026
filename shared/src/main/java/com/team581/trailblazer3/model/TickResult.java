package com.team581.trailblazer3.model;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public record TickResult(
    ChassisSpeeds commandedFieldSpeeds,
    RuntimeStatus status,
    boolean infeasible,
    LimitingFactor limitingFactor,
    long geometryGeneration,
    double progress,
    String diagnostic,
    VelocityStep velocityStep) {

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
        new VelocityStep(commandedFieldSpeeds, 0));
  }

  /** Zero duration requests the endpoint immediately. */
  public record VelocityStep(
      double startVx,
      double startVy,
      double startOmega,
      double durationSeconds,
      double startTime,
      long sequence) {
    public VelocityStep {
      if (!Double.isFinite(durationSeconds)
          || durationSeconds < 0
          || (durationSeconds > 0 && (!Double.isFinite(startTime) || sequence <= 0))) {
        throw new IllegalArgumentException(
            "A ramp requires a duration, planner timestamp and sequence");
      }
    }

    public VelocityStep(ChassisSpeeds start, double durationSeconds) {
      this(start, durationSeconds, Double.NaN, 0);
    }

    public VelocityStep(
        ChassisSpeeds start, double durationSeconds, double startTime, long sequence) {
      this(
          start.vxMetersPerSecond,
          start.vyMetersPerSecond,
          start.omegaRadiansPerSecond,
          durationSeconds,
          startTime,
          sequence);
    }
  }

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
