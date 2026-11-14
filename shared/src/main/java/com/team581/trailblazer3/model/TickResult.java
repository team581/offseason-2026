package com.team581.trailblazer3.model;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.List;
import java.util.Map;
import java.util.Set;

public record TickResult(
    ChassisSpeeds commandedFieldSpeeds,
    ChassisAccelerations commandedFieldAccelerations,
    List<Translation2d> wheelForces,
    RuntimeStatus status,
    boolean infeasible,
    LimitingFactor profileLimitingFactor,
    LimitingFactor commandLimitingFactor,
    Set<Enum<?>> newlyPassedMarkers,
    Map<String, GoalPassageStatus> goalStatuses,
    long geometryGeneration,
    double progress,
    String diagnostic) {
  public TickResult {
    commandedFieldSpeeds = copy(commandedFieldSpeeds);
    wheelForces = List.copyOf(wheelForces);
    newlyPassedMarkers =
        java.util.Collections.unmodifiableSet(new java.util.LinkedHashSet<>(newlyPassedMarkers));
    goalStatuses = Map.copyOf(goalStatuses);
  }

  @Override
  public ChassisSpeeds commandedFieldSpeeds() {
    return copy(commandedFieldSpeeds);
  }

  private static ChassisSpeeds copy(ChassisSpeeds speeds) {
    return new ChassisSpeeds(
        speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
  }

  public enum RuntimeStatus {
    RUNNING,
    AT_GOAL,
    FAULT,
    CANCELLED,
    DISABLED
  }

  public enum GoalPassageStatus {
    PASSED_WITHIN_TOLERANCE,
    PASSED_OUTSIDE_TOLERANCE
  }

  public enum LimitingFactor {
    NONE,
    TOP_SPEED,
    CORNER_CENTRIPETAL,
    ZONE_SPEED_CAP,
    GOAL_DECELERATION,
    MOTOR_ACCELERATION,
    HEADING_HARD_GATE,
    MODULE_SPEED,
    VECTOR_SLEW_LIMIT,
    INFEASIBLE
  }

  public record ChassisAccelerations(double ax, double ay, double alpha) {}
}
