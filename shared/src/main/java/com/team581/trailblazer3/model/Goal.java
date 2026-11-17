package com.team581.trailblazer3.model;

import static com.google.common.base.Preconditions.checkArgument;
import static java.util.Objects.requireNonNull;

import com.team581.util.FieldUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * An immutable waypoint with passage rules, an optional heading plan, and an optional event marker.
 */
public record Goal(
    Pose2d pose,
    PassType passType,
    double withinMeters,
    double passSpeed,
    double maxOverrunMeters,
    FinishTolerance finishTolerance,
    Enum<?> marker,
    HeadingPlan heading) {
  /** Determines whether the waypoint is an intermediate pass, a stop, or a moving handoff. */
  public enum PassType {
    STOP,
    THROUGH,
    FINISH_THROUGH
  }

  /**
   * Limits on stop position error, heading error, linear and angular speed, and settling time in SI
   * units.
   */
  public record FinishTolerance(
      double positionMeters,
      double headingRadians,
      double linearSpeed,
      double angularSpeed,
      double dwellSeconds) {
    public FinishTolerance {
      nonnegative(positionMeters);
      nonnegative(headingRadians);
      nonnegative(linearSpeed);
      nonnegative(angularSpeed);
      nonnegative(dwellSeconds);
    }
  }

  /** Validates the waypoint coordinates, tolerance, speed, and overrun. */
  public Goal {
    checkArgument(
        Double.isFinite(pose.getX())
            && Double.isFinite(pose.getY())
            && Double.isFinite(pose.getRotation().getRadians()),
        "Nonfinite goal pose");
    requireNonNull(passType);
    requireNonNull(finishTolerance);
    nonnegative(withinMeters);
    checkArgument(
        !Double.isNaN(maxOverrunMeters) && maxOverrunMeters >= 0, "Expected a nonnegative overrun");
    checkArgument(
        passType == PassType.FINISH_THROUGH || Double.isFinite(maxOverrunMeters),
        "Expected a finite overrun");
    checkArgument(
        !Double.isNaN(passSpeed)
            && passSpeed >= 0
            && !(passType == PassType.FINISH_THROUGH
                && (!Double.isFinite(passSpeed) || passSpeed == 0 || maxOverrunMeters == 0)),
        "Invalid goal speed or overrun");
  }

  private static void nonnegative(double value) {
    checkArgument(Double.isFinite(value) && value >= 0, "Expected a finite nonnegative value");
  }

  /** Creates a terminal stop that settles at the given pose before completion. */
  public static Goal stop(Pose2d pose) {
    return create(pose, PassType.STOP, 0, 0);
  }

  /** Creates a through waypoint with a 0.25 m positional tolerance and no heading keyframe. */
  public static Goal through(Translation2d point) {
    return create(
        new Pose2d(point, Rotation2d.kZero), PassType.THROUGH, Double.POSITIVE_INFINITY, 0);
  }

  /**
   * Creates a through waypoint with a 0.25 m positional tolerance and a fixed heading from the
   * pose.
   */
  public static Goal through(Pose2d pose) {
    return through(pose.getTranslation()).withPose(pose).face(pose.getRotation());
  }

  /** Creates a moving handoff with the given target passage speed in meters per second. */
  public static Goal finishThrough(Translation2d point, double speed) {
    return finishThrough(point, speed, Double.POSITIVE_INFINITY);
  }

  /**
   * Creates a moving handoff with the pose’s field heading and a target speed in meters per second.
   */
  public static Goal finishThrough(Pose2d pose, double speed) {
    return create(pose, PassType.FINISH_THROUGH, speed, Double.POSITIVE_INFINITY)
        .face(pose.getRotation());
  }

  /**
   * Creates a moving handoff with an explicit maximum distance in meters for braking after passage.
   */
  public static Goal finishThrough(Translation2d point, double speed, double maxOverrunMeters) {
    return create(
        new Pose2d(point, Rotation2d.kZero), PassType.FINISH_THROUGH, speed, maxOverrunMeters);
  }

  private static Goal create(Pose2d pose, PassType type, double speed, double overrun) {
    return new Goal(
        pose,
        type,
        0.25,
        speed,
        overrun,
        new FinishTolerance(0.03, Math.toRadians(2), 0.10, 0.10, 0.10),
        null,
        null);
  }

  /** Returns a copy with a fixed field heading using the default soft, shortest-turn plan. */
  public Goal face(Rotation2d target) {
    return heading(HeadingPlan.to(target));
  }

  /**
   * Returns a copy with a fixed field heading in degrees, counterclockwise from field +X. The
   * follower ramps rotation toward the target within its angular limits. With the default soft
   * policy, passing the waypoint does not require the robot to have reached that heading.
   */
  public Goal faceDegrees(double degrees) {
    return heading(HeadingPlan.faceDegrees(degrees));
  }

  /** Returns a copy that aims the robot’s forward axis at a field point. */
  public Goal facePoint(Translation2d fieldTarget) {
    return heading(HeadingPlan.facePoint(fieldTarget));
  }

  /**
   * Returns a copy that aims the mechanism at a field point using its robot-relative angular
   * offset.
   */
  public Goal facePoint(Translation2d fieldTarget, Rotation2d mechanismOffset) {
    return heading(HeadingPlan.facePoint(fieldTarget, mechanismOffset));
  }

  /** Aims the rear-mounted shooter ({@code 180 deg} robot offset) at the alliance Hub. */
  public Goal faceHub() {
    return faceHub(Rotation2d.k180deg);
  }

  /** Aims a robot mechanism at {@code mechanismOffset} toward the alliance Hub. */
  public Goal faceHub(Rotation2d mechanismOffset) {
    return facePoint(FieldUtil.BLUE_HUB_TRANSLATION, mechanismOffset);
  }

  /** Returns a copy that points the chassis along the path tangent. */
  public Goal faceTravel() {
    return heading(HeadingPlan.faceTravel());
  }

  /** Returns a copy with the supplied heading plan, including its anchor offset and gate policy. */
  public Goal heading(HeadingPlan keyframe) {
    return new Goal(
        pose,
        passType,
        withinMeters,
        passSpeed,
        maxOverrunMeters,
        finishTolerance,
        marker,
        keyframe);
  }

  /** Returns a copy with a passage speed limit in meters per second at the waypoint anchor. */
  public Goal passSpeed(double metersPerSecond) {
    return new Goal(
        pose,
        passType,
        withinMeters,
        metersPerSecond,
        maxOverrunMeters,
        finishTolerance,
        marker,
        heading);
  }

  /** Transforms the waypoint position and heading plan using the field symmetry. */
  public Goal flip(FieldGeometry field) {
    var transformed = withPose(field.flipPose(pose));
    return heading == null ? transformed : transformed.heading(heading.flip(field));
  }

  /** Returns a copy with a replacement pose and the same explicit heading plan. */
  public Goal withPose(Pose2d target) {
    return new Goal(
        target,
        passType,
        withinMeters,
        passSpeed,
        maxOverrunMeters,
        finishTolerance,
        marker,
        heading);
  }

  /**
   * Returns a copy with a positional passage tolerance in meters. The planner uses this distance to
   * size a fillet around a through waypoint.
   */
  public Goal within(double meters) {
    return new Goal(
        pose, passType, meters, passSpeed, maxOverrunMeters, finishTolerance, marker, heading);
  }

  /** Sets an angular passage tolerance on a fixed-heading through point. */
  public Goal headingTolerance(Rotation2d tolerance) {
    checkArgument(
        passType == PassType.THROUGH
            && heading != null
            && heading.target() instanceof HeadingPlan.Target.Fixed,
        "Angular passage tolerance requires a fixed-heading through point");
    return heading(heading.tolerance(tolerance));
  }

  /** Returns a copy with an event marker that the follower records when the goal passes. */
  public Goal mark(Enum<?> event) {
    return new Goal(
        pose, passType, withinMeters, passSpeed, maxOverrunMeters, finishTolerance, event, heading);
  }

  /** Returns a copy that approaches its heading target with a clockwise turn. */
  public Goal clockwise() {
    return heading(baseHeadingPlan().clockwise());
  }

  /** Returns a copy that approaches its heading target with a counterclockwise turn. */
  public Goal counterClockwise() {
    return heading(baseHeadingPlan().counterClockwise());
  }

  /** Returns a copy whose heading readiness can hold translation at the heading anchor. */
  public Goal hardGate() {
    return heading(baseHeadingPlan().hardGate());
  }

  /**
   * Updates the stop’s position, heading, and linear speed tolerances and keeps its other settle
   * settings.
   */
  public Goal finish(double meters, Rotation2d headingTolerance, double speed) {
    return finish(
        new FinishTolerance(
            meters,
            headingTolerance.getRadians(),
            speed,
            finishTolerance.angularSpeed(),
            finishTolerance.dwellSeconds()));
  }

  /** Returns a copy with the full set of settling tolerances for a terminal stop. */
  public Goal finish(FinishTolerance tolerance) {
    return new Goal(
        pose, passType, withinMeters, passSpeed, maxOverrunMeters, tolerance, marker, heading);
  }

  private HeadingPlan baseHeadingPlan() {
    if (heading != null) {
      return heading;
    }
    if (passType == PassType.STOP) {
      return HeadingPlan.to(pose.getRotation());
    }
    throw new IllegalStateException("Set a heading before configuring turn direction or gate");
  }
}
