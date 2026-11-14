package com.team581.trailblazer3.model;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Objects;

public record AutoSegment(String id, MotionLimits limits, List<Goal> goals, List<SpeedZone> zones) {
  public AutoSegment(String id, MotionLimits limits, List<Goal> goals) {
    this(id, limits, goals, List.of());
  }

  public AutoSegment {
    Objects.requireNonNull(limits);
    zones = List.copyOf(zones);
    if (zones.size() > 128) {
      throw new IllegalArgumentException("Too many speed zones");
    }
    if (id == null || id.isBlank() || goals.isEmpty() || goals.size() > 128) {
      throw new IllegalArgumentException("Invalid plan ID or goal count");
    }
    var ids = new HashSet<String>();
    var markers = new HashSet<Enum<?>>();
    var normalized = new ArrayList<Goal>();
    for (Goal goal : goals) {
      var named = goal.id() == null ? goal.withId("goal-" + normalized.size()) : goal;
      if (!ids.add(named.id()) || (named.marker() != null && !markers.add(named.marker()))) {
        throw new IllegalArgumentException("Repeated goal ID or marker at " + named.id());
      }
      normalized.add(named);
    }
    goals = List.copyOf(normalized);
    if (goals.get(goals.size() - 1).passType() == Goal.PassType.THROUGH) {
      throw new IllegalArgumentException("Final goal must stop or cruise through");
    }
    for (var goal : goals.subList(0, goals.size() - 1)) {
      if (goal.passType() == Goal.PassType.CRUISE_THROUGH) {
        throw new IllegalArgumentException("Cruise-through goal must be last");
      }
    }
  }

  public AutoSegment forAlliance(Alliance alliance, FieldGeometry field) {
    Objects.requireNonNull(alliance);
    Objects.requireNonNull(field);
    return alliance == Alliance.Blue
        ? this
        : new AutoSegment(id, limits, goals.stream().map(goal -> goal.flip(field)).toList(), zones);
  }

  public static final class Builder {
    private final String id;
    private MotionLimits limits = MotionLimits.robotMaximum();
    private final List<Goal> goals = new ArrayList<>();
    private final List<SpeedZone> zones = new ArrayList<>();

    public Builder(String id) {
      this.id = id;
    }

    public Builder addGoal(Goal goal) {
      goals.add(goal);
      return this;
    }

    public Builder addZone(SpeedZone zone) {
      zones.add(zone);
      return this;
    }

    public AutoSegment build() {
      return new AutoSegment(id, limits, goals, zones);
    }

    public Builder cruiseThrough(
        edu.wpi.first.math.geometry.Translation2d point, double speed, double overrun) {
      return addGoal(Goal.cruiseThrough(point, speed, overrun));
    }

    public Builder stopAt(edu.wpi.first.math.geometry.Pose2d pose) {
      return addGoal(Goal.stop(pose));
    }

    public Builder through(edu.wpi.first.math.geometry.Translation2d point) {
      return addGoal(Goal.through(point));
    }

    public Builder withLimits(MotionLimits profile) {
      limits = profile;
      return this;
    }
  }
}
