package com.team581.trailblazer3.model;

import static com.google.common.base.Preconditions.checkArgument;
import static com.google.common.base.Preconditions.checkState;
import static java.util.Objects.requireNonNull;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.function.UnaryOperator;

/** An immutable sequence of through goals followed by a stop or moving handoff. */
public record AutoSegment(String id, MotionLimits limits, List<Goal> goals) {
  /** Validates goal order, unique markers, and segment size, then snapshots the goals. */
  public AutoSegment {
    requireNonNull(limits);
    checkArgument(
        id != null && !id.isBlank() && !goals.isEmpty() && goals.size() <= 128,
        "Invalid plan ID or goal count");
    var markers = new HashSet<Enum<?>>();
    for (int i = 0; i < goals.size(); i++) {
      var goal = goals.get(i);
      checkArgument(
          goal.marker() == null || markers.add(goal.marker()),
          "Repeated goal marker %s",
          goal.marker());
      checkArgument(
          i >= goals.size() - 1 || goal.passType() == Goal.PassType.THROUGH,
          "Intermediate goals must pass through");
      checkArgument(
          i != goals.size() - 1 || goal.passType() != Goal.PassType.THROUGH,
          "Final goal must use stop or finishThrough");
    }
    goals = List.copyOf(goals);
  }

  /** Maps a blue-authored segment to the active alliance using the supplied field symmetry. */
  public AutoSegment forAlliance(Alliance alliance, FieldGeometry field) {
    requireNonNull(alliance);
    requireNonNull(field);
    return alliance == Alliance.Blue
        ? this
        : new AutoSegment(id, limits, goals.stream().map(goal -> goal.flip(field)).toList());
  }

  /**
   * Builds a segment in traversal order. Field coordinates are in meters. Author reusable autos in
   * the blue frame and apply {@link AutoSegment#forAlliance} before activation.
   *
   * <p>Use {@code through} for an intermediate point the robot should drive near while continuing
   * along the route. At a corner, the planner can round off the waypoint and slow the robot for the
   * turn. The positional tolerance gives the planner room to round the corner. It measures distance
   * from the waypoint in meters.
   *
   * <p>A coordinate or translation overload adds only a position. A pose overload also adds a
   * heading target, so the chassis can turn while driving independently of its direction of travel.
   * Use a callback to set a custom heading and chain {@code mark} to attach a marker. For example,
   * {@code through(x, y, 0.35, g -> g.faceDegrees(90)).mark(MyMarker.INTAKE)} lets the caller check
   * whether the robot has passed the waypoint with {@code passedMarker(MyMarker.INTAKE)}. The
   * caller decides what action to take when the marker passes.
   *
   * <p>Use {@code stopAt} when the segment should finish after settling at a pose. Use {@code
   * finishThrough} when the segment should finish at a moving handoff.
   */
  public static final class Builder {
    private final String id;
    private MotionLimits limits = MotionLimits.robotMaximum();
    private final List<Goal> goals = new ArrayList<>();

    /** Starts a builder with a nonblank segment ID and the robot’s default motion limits. */
    public Builder(String id) {
      this.id = id;
    }

    /**
     * Appends a preconfigured goal. Use this when a route helper returns a {@link Goal}, or when
     * configuring the goal separately makes the route easier to read. The goal's pass type controls
     * how the robot drives it. Appending a through goal here has the same behavior as adding it
     * with a {@code through} overload.
     */
    public Builder addGoal(Goal goal) {
      goals.add(goal);
      return this;
    }

    /** Snapshots the segment and validates that only its final goal is terminal. */
    public AutoSegment build() {
      return new AutoSegment(id, limits, goals);
    }

    /** Ends the segment at the given pose and handoff speed in meters per second. */
    public Builder finishThrough(Pose2d pose, double speed) {
      return addGoal(Goal.finishThrough(pose, speed));
    }

    /** Ends the segment at a position with the given handoff speed in meters per second. */
    public Builder finishThrough(Translation2d point, double speed) {
      return addGoal(Goal.finishThrough(point, speed));
    }

    /**
     * Configures and appends a moving handoff at the given position and speed in meters per second.
     */
    public Builder finishThrough(Translation2d point, double speed, UnaryOperator<Goal> configure) {
      return addGoal(requireNonNull(configure).apply(Goal.finishThrough(point, speed)));
    }

    /**
     * Sets the marker on the most recently added waypoint and returns this builder. A waypoint
     * holds one marker, so this replaces any marker already on that waypoint.
     *
     * @throws IllegalStateException if no waypoint has been added
     */
    public Builder mark(Enum<?> marker) {
      checkState(!goals.isEmpty(), "Add a waypoint before marking it");
      int lastIndex = goals.size() - 1;
      goals.set(lastIndex, goals.get(lastIndex).mark(marker));
      return this;
    }

    /** Caps forward translational acceleration in meters per second squared. */
    public Builder maxAcceleration(double maxAccelerationMetersPerSecondSquared) {
      limits = limits.withMaxAcceleration(maxAccelerationMetersPerSecondSquared);
      return this;
    }

    /** Caps translational braking magnitude in meters per second squared. */
    public Builder maxBraking(double maxBrakingMetersPerSecondSquared) {
      limits = limits.withMaxBraking(maxBrakingMetersPerSecondSquared);
      return this;
    }

    /** Caps translational speed in meters per second. */
    public Builder maxSpeed(double maxVelocityMetersPerSecond) {
      limits = limits.withMaxSpeed(maxVelocityMetersPerSecond);
      return this;
    }

    /** Ends at a stop with an absolute field heading in degrees, counterclockwise from +X. */
    public Builder stopAt(double xMeters, double yMeters, double headingDegrees) {
      return addGoal(
          Goal.stop(new Pose2d(xMeters, yMeters, Rotation2d.fromDegrees(headingDegrees))));
    }

    /** Configures and appends a stop at the given coordinates with a default heading of 0°. */
    public Builder stopAt(double xMeters, double yMeters, UnaryOperator<Goal> configure) {
      return stopAt(new Translation2d(xMeters, yMeters), configure);
    }

    /** Ends at a stop at the given position and heading. */
    public Builder stopAt(Pose2d pose) {
      return addGoal(Goal.stop(pose));
    }

    /** Configures and appends a stop initialized from the given pose. */
    public Builder stopAt(Pose2d pose, UnaryOperator<Goal> configure) {
      return addGoal(requireNonNull(configure).apply(Goal.stop(pose)));
    }

    /** Configures and appends a stop at the given point with a default heading of 0°. */
    public Builder stopAt(Translation2d point, UnaryOperator<Goal> configure) {
      return addGoal(
          requireNonNull(configure).apply(Goal.stop(new Pose2d(point, Rotation2d.kZero))));
    }

    /** Drives near these coordinates and continues, with 0.25 m of room to round the corner. */
    public Builder through(double xMeters, double yMeters) {
      return addGoal(Goal.through(new Translation2d(xMeters, yMeters)));
    }

    /**
     * Drives near these coordinates and continues. A larger tolerance can allow a wider turn when
     * adjacent waypoints and field clearance leave room.
     *
     * @param xMeters field X in meters
     * @param yMeters field Y in meters
     * @param blendRadiusMeters allowed distance from the waypoint in meters, used to size the
     *     fillet
     * @return this builder
     */
    public Builder through(double xMeters, double yMeters, double blendRadiusMeters) {
      return addGoal(Goal.through(new Translation2d(xMeters, yMeters)).within(blendRadiusMeters));
    }

    /**
     * Adds a through waypoint with the given positional tolerance in meters, then applies the
     * configurator.
     */
    public Builder through(
        double xMeters, double yMeters, double blendRadiusMeters, UnaryOperator<Goal> configure) {
      return addGoal(
          requireNonNull(configure)
              .apply(Goal.through(new Translation2d(xMeters, yMeters)).within(blendRadiusMeters)));
    }

    /** Configures a through waypoint initialized with the default 0.25 m positional tolerance. */
    public Builder through(double xMeters, double yMeters, UnaryOperator<Goal> configure) {
      return addGoal(
          requireNonNull(configure).apply(Goal.through(new Translation2d(xMeters, yMeters))));
    }

    /**
     * Drives near the pose's position with a 0.25 m tolerance and turns toward its field heading
     * while moving. By default, the robot can pass the waypoint before reaching the heading.
     */
    public Builder through(Pose2d pose) {
      return addGoal(Goal.through(pose));
    }

    /**
     * Adds a through waypoint with the given positional tolerance in meters and the pose’s heading.
     */
    public Builder through(Pose2d pose, double blendRadiusMeters) {
      return addGoal(Goal.through(pose).within(blendRadiusMeters));
    }

    /** Applies the configurator after setting the pose heading and given positional tolerance. */
    public Builder through(Pose2d pose, double blendRadiusMeters, UnaryOperator<Goal> configure) {
      return addGoal(requireNonNull(configure).apply(Goal.through(pose).within(blendRadiusMeters)));
    }

    /** Configures a through waypoint initialized with the pose heading and a 0.25 m tolerance. */
    public Builder through(Pose2d pose, UnaryOperator<Goal> configure) {
      return addGoal(requireNonNull(configure).apply(Goal.through(pose)));
    }

    /** Adds a through waypoint with the default 0.25 m positional tolerance. */
    public Builder through(Translation2d point) {
      return addGoal(Goal.through(point));
    }

    /** Adds a through waypoint with the given positional tolerance in meters. */
    public Builder through(Translation2d point, double blendRadiusMeters) {
      return addGoal(Goal.through(point).within(blendRadiusMeters));
    }

    /**
     * Adds a through waypoint with the given positional tolerance in meters, then applies the
     * configurator.
     */
    public Builder through(
        Translation2d point, double blendRadiusMeters, UnaryOperator<Goal> configure) {
      return addGoal(
          requireNonNull(configure).apply(Goal.through(point).within(blendRadiusMeters)));
    }

    /** Configures a through waypoint initialized with the default 0.25 m positional tolerance. */
    public Builder through(Translation2d point, UnaryOperator<Goal> configure) {
      return addGoal(requireNonNull(configure).apply(Goal.through(point)));
    }

    /**
     * Replaces the segment’s entire motion limit profile, including any previously set ceilings.
     */
    public Builder withLimits(MotionLimits profile) {
      limits = profile;
      return this;
    }
  }
}
