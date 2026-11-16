package com.team581.trailblazer3.model;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Objects;
import java.util.function.UnaryOperator;

public record AutoSegment(String id, MotionLimits limits, List<Goal> goals) {
  public AutoSegment {
    Objects.requireNonNull(limits);
    if (id == null || id.isBlank() || goals.isEmpty() || goals.size() > 128) {
      throw new IllegalArgumentException("Invalid plan ID or goal count");
    }
    var markers = new HashSet<Enum<?>>();
    for (int i = 0; i < goals.size(); i++) {
      var goal = goals.get(i);
      if (goal.marker() != null && !markers.add(goal.marker())) {
        throw new IllegalArgumentException("Repeated goal marker " + goal.marker());
      }
      if (i < goals.size() - 1 && goal.passType() != Goal.PassType.THROUGH) {
        throw new IllegalArgumentException("Intermediate goals must pass through");
      }
      if (i == goals.size() - 1 && goal.passType() == Goal.PassType.THROUGH) {
        throw new IllegalArgumentException("Final goal must use stop or finishThrough");
      }
    }
    goals = List.copyOf(goals);
  }

  public AutoSegment forAlliance(Alliance alliance, FieldGeometry field) {
    Objects.requireNonNull(alliance);
    Objects.requireNonNull(field);
    return alliance == Alliance.Blue
        ? this
        : new AutoSegment(id, limits, goals.stream().map(goal -> goal.flip(field)).toList());
  }

  public static final class Builder {
    private final String id;
    private MotionLimits limits = MotionLimits.robotMaximum();
    private final List<Goal> goals = new ArrayList<>();

    public Builder(String id) {
      this.id = id;
    }

    public Builder addGoal(Goal goal) {
      goals.add(goal);
      return this;
    }

    public AutoSegment build() {
      return new AutoSegment(id, limits, goals);
    }

    public Builder finishThrough(Pose2d pose, double speed) {
      return addGoal(Goal.finishThrough(pose, speed));
    }

    public Builder finishThrough(Translation2d point, double speed) {
      return addGoal(Goal.finishThrough(point, speed));
    }

    public Builder finishThrough(Translation2d point, double speed, UnaryOperator<Goal> configure) {
      return addGoal(Objects.requireNonNull(configure).apply(Goal.finishThrough(point, speed)));
    }

    public Builder maxAcceleration(double maxAccelerationMetersPerSecondSquared) {
      limits = limits.withMaxAcceleration(maxAccelerationMetersPerSecondSquared);
      return this;
    }

    public Builder maxBraking(double maxBrakingMetersPerSecondSquared) {
      limits = limits.withMaxBraking(maxBrakingMetersPerSecondSquared);
      return this;
    }

    public Builder maxSpeed(double maxVelocityMetersPerSecond) {
      limits = limits.withMaxSpeed(maxVelocityMetersPerSecond);
      return this;
    }

    public Builder stopAt(double xMeters, double yMeters, double headingDegrees) {
      return addGoal(
          Goal.stop(new Pose2d(xMeters, yMeters, Rotation2d.fromDegrees(headingDegrees))));
    }

    public Builder stopAt(double xMeters, double yMeters, UnaryOperator<Goal> configure) {
      return stopAt(new Translation2d(xMeters, yMeters), configure);
    }

    public Builder stopAt(Pose2d pose) {
      return addGoal(Goal.stop(pose));
    }

    public Builder stopAt(Pose2d pose, UnaryOperator<Goal> configure) {
      return addGoal(Objects.requireNonNull(configure).apply(Goal.stop(pose)));
    }

    public Builder stopAt(Translation2d point, UnaryOperator<Goal> configure) {
      return addGoal(
          Objects.requireNonNull(configure).apply(Goal.stop(new Pose2d(point, Rotation2d.kZero))));
    }

    public Builder through(double xMeters, double yMeters) {
      return addGoal(Goal.through(new Translation2d(xMeters, yMeters)));
    }

    public Builder through(double xMeters, double yMeters, double blendRadiusMeters) {
      return addGoal(Goal.through(new Translation2d(xMeters, yMeters)).within(blendRadiusMeters));
    }

    public Builder through(
        double xMeters, double yMeters, double blendRadiusMeters, UnaryOperator<Goal> configure) {
      return addGoal(
          Objects.requireNonNull(configure)
              .apply(Goal.through(new Translation2d(xMeters, yMeters)).within(blendRadiusMeters)));
    }

    public Builder through(double xMeters, double yMeters, UnaryOperator<Goal> configure) {
      return addGoal(
          Objects.requireNonNull(configure)
              .apply(Goal.through(new Translation2d(xMeters, yMeters))));
    }

    public Builder through(Pose2d pose) {
      return addGoal(Goal.through(pose));
    }

    public Builder through(Pose2d pose, double blendRadiusMeters) {
      return addGoal(Goal.through(pose).within(blendRadiusMeters));
    }

    public Builder through(Translation2d point) {
      return addGoal(Goal.through(point));
    }

    public Builder through(Translation2d point, double blendRadiusMeters) {
      return addGoal(Goal.through(point).within(blendRadiusMeters));
    }

    public Builder through(
        Translation2d point, double blendRadiusMeters, UnaryOperator<Goal> configure) {
      return addGoal(
          Objects.requireNonNull(configure).apply(Goal.through(point).within(blendRadiusMeters)));
    }

    public Builder through(Translation2d point, UnaryOperator<Goal> configure) {
      return addGoal(Objects.requireNonNull(configure).apply(Goal.through(point)));
    }

    public Builder withLimits(MotionLimits profile) {
      limits = profile;
      return this;
    }
  }
}
