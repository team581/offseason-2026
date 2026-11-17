package com.team581.trailblazer3.model;

import static com.google.common.base.Preconditions.checkArgument;
import static java.util.Objects.requireNonNull;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** A chassis heading target, its anchor offset along the path, and the rules for reaching it. */
public record HeadingPlan(
    Target target,
    double offsetMeters,
    TurnDirection direction,
    GatePolicy gate,
    double toleranceRadians,
    double angularSpeedTolerance,
    double dwellSeconds,
    boolean explicitTolerance) {
  /** Chooses how the follower unwraps a heading target when the keyframe becomes active. */
  public enum TurnDirection {
    SHORTEST,
    CW,
    CCW
  }

  /** Determines whether heading readiness can hold translation at the anchor. */
  public enum GatePolicy {
    SOFT,
    HARD
  }

  /** Describes how to compute the field heading the robot should face. */
  public sealed interface Target {
    /** An absolute field heading. */
    record Fixed(Rotation2d angle) implements Target {
      public Fixed {
        checkArgument(Double.isFinite(angle.getRadians()), "Invalid fixed heading");
      }
    }

    /** A field point to aim at using a robot-relative mechanism angle. */
    record Point(Translation2d target, Rotation2d mechanismOffset) implements Target {
      public Point {
        checkArgument(
            Double.isFinite(target.getNorm()) && Double.isFinite(mechanismOffset.getRadians()),
            "Invalid aim point");
      }
    }

    /** A heading that follows the path tangent. */
    record Travel() implements Target {}
  }

  /** Validates the target and the heading readiness thresholds. */
  public HeadingPlan {
    requireNonNull(target);
    requireNonNull(direction);
    requireNonNull(gate);
    checkArgument(
        Double.isFinite(offsetMeters)
            && Double.isFinite(toleranceRadians)
            && toleranceRadians >= 0
            && Double.isFinite(angularSpeedTolerance)
            && angularSpeedTolerance >= 0
            && Double.isFinite(dwellSeconds)
            && dwellSeconds >= 0,
        "Invalid heading keyframe");
  }

  /** Creates a keyframe with gate readiness thresholds and legacy heading scheduling. */
  public HeadingPlan(
      Target target,
      double offsetMeters,
      TurnDirection direction,
      GatePolicy gate,
      double toleranceRadians,
      double angularSpeedTolerance,
      double dwellSeconds) {
    this(
        target,
        offsetMeters,
        direction,
        gate,
        toleranceRadians,
        angularSpeedTolerance,
        dwellSeconds,
        false);
  }

  /** Creates a fixed field heading in degrees, counterclockwise from +X. */
  public static HeadingPlan faceDegrees(double degrees) {
    return to(Rotation2d.fromDegrees(degrees));
  }

  /** Creates a soft fixed-heading plan that takes the shortest turn by default. */
  public static HeadingPlan to(Rotation2d angle) {
    return of(new Target.Fixed(angle));
  }

  /** Creates a heading plan that follows the path tangent. */
  public static HeadingPlan faceTravel() {
    return of(new Target.Travel());
  }

  /** Creates a heading plan that points the front of the robot at the given field point. */
  public static HeadingPlan facePoint(Translation2d point) {
    return facePoint(point, Rotation2d.kZero);
  }

  /**
   * Creates a heading plan that aims a mechanism with the given robot-relative offset at the point.
   */
  public static HeadingPlan facePoint(Translation2d point, Rotation2d mechanismOffset) {
    return of(new Target.Point(point, mechanismOffset));
  }

  private static HeadingPlan of(Target target) {
    return new HeadingPlan(
        target, 0, TurnDirection.SHORTEST, GatePolicy.SOFT, Math.toRadians(2.5), 0.10, 0.10);
  }

  /** Places the heading anchor this many meters before the waypoint along the path. */
  public HeadingPlan before(double meters) {
    return new HeadingPlan(
        target,
        -meters,
        direction,
        gate,
        toleranceRadians,
        angularSpeedTolerance,
        dwellSeconds,
        explicitTolerance);
  }

  /** Places the heading anchor this many meters after the waypoint along the path. */
  public HeadingPlan after(double meters) {
    return before(-meters);
  }

  /** Returns a copy with the given heading gate policy. */
  public HeadingPlan gate(GatePolicy policy) {
    return new HeadingPlan(
        target,
        offsetMeters,
        direction,
        policy,
        toleranceRadians,
        angularSpeedTolerance,
        dwellSeconds,
        explicitTolerance);
  }

  /** Returns a copy with the given turn direction. */
  public HeadingPlan direction(TurnDirection turn) {
    return new HeadingPlan(
        target,
        offsetMeters,
        turn,
        gate,
        toleranceRadians,
        angularSpeedTolerance,
        dwellSeconds,
        explicitTolerance);
  }

  /** Requests the clockwise branch when acquiring the target. */
  public HeadingPlan clockwise() {
    return direction(TurnDirection.CW);
  }

  /** Requests the counterclockwise branch when acquiring the target. */
  public HeadingPlan counterClockwise() {
    return direction(TurnDirection.CCW);
  }

  /** Allows the follower to brake and hold at the anchor for heading readiness. */
  public HeadingPlan hardGate() {
    return gate(GatePolicy.HARD);
  }

  /**
   * Returns a copy with the given maximum heading error for through-point planning and gate
   * readiness.
   */
  public HeadingPlan tolerance(Rotation2d tolerance) {
    return new HeadingPlan(
        target,
        offsetMeters,
        direction,
        gate,
        tolerance.getRadians(),
        angularSpeedTolerance,
        dwellSeconds,
        true);
  }

  /** Transforms the target and turn direction for the supplied field symmetry. */
  public HeadingPlan flip(FieldGeometry field) {
    Target flipped = target;
    if (target instanceof Target.Fixed fixed) {
      flipped =
          new Target.Fixed(
              field.flipPose(new Pose2d(Translation2d.kZero, fixed.angle())).getRotation());
    } else if (target instanceof Target.Point aim) {
      flipped = new Target.Point(field.flipPoint(aim.target()), aim.mechanismOffset());
    }
    TurnDirection mirrored = direction;
    if (field.symmetry() == FieldSymmetry.MIRROR_X) {
      mirrored =
          switch (direction) {
            case CW -> TurnDirection.CCW;
            case CCW -> TurnDirection.CW;
            case SHORTEST -> TurnDirection.SHORTEST;
          };
    }
    return new HeadingPlan(
        flipped,
        offsetMeters,
        mirrored,
        gate,
        toleranceRadians,
        angularSpeedTolerance,
        dwellSeconds,
        explicitTolerance);
  }
}
