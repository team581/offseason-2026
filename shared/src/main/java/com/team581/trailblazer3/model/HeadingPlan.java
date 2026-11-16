package com.team581.trailblazer3.model;

import static com.google.common.base.Preconditions.checkArgument;
import static java.util.Objects.requireNonNull;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public record HeadingPlan(
    Target target,
    double offsetMeters,
    TurnDirection direction,
    GatePolicy gate,
    double toleranceRadians,
    double angularSpeedTolerance,
    double dwellSeconds) {
  public enum TurnDirection {
    SHORTEST,
    CW,
    CCW
  }

  public enum GatePolicy {
    SOFT,
    HARD
  }

  public sealed interface Target {
    record Fixed(Rotation2d angle) implements Target {
      public Fixed {
        checkArgument(Double.isFinite(angle.getRadians()), "Invalid fixed heading");
      }
    }

    record Point(Translation2d target, Rotation2d mechanismOffset) implements Target {
      public Point {
        checkArgument(
            Double.isFinite(target.getNorm()) && Double.isFinite(mechanismOffset.getRadians()),
            "Invalid aim point");
      }
    }

    record Travel() implements Target {}
  }

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

  public static HeadingPlan faceDegrees(double degrees) {
    return to(Rotation2d.fromDegrees(degrees));
  }

  public static HeadingPlan to(Rotation2d angle) {
    return of(new Target.Fixed(angle));
  }

  public static HeadingPlan faceTravel() {
    return of(new Target.Travel());
  }

  public static HeadingPlan facePoint(Translation2d point) {
    return facePoint(point, Rotation2d.kZero);
  }

  public static HeadingPlan facePoint(Translation2d point, Rotation2d mechanismOffset) {
    return of(new Target.Point(point, mechanismOffset));
  }

  private static HeadingPlan of(Target target) {
    return new HeadingPlan(
        target, 0, TurnDirection.SHORTEST, GatePolicy.SOFT, Math.toRadians(2.5), 0.10, 0.10);
  }

  public HeadingPlan before(double meters) {
    return new HeadingPlan(
        target, -meters, direction, gate, toleranceRadians, angularSpeedTolerance, dwellSeconds);
  }

  public HeadingPlan after(double meters) {
    return before(-meters);
  }

  public HeadingPlan gate(GatePolicy policy) {
    return new HeadingPlan(
        target,
        offsetMeters,
        direction,
        policy,
        toleranceRadians,
        angularSpeedTolerance,
        dwellSeconds);
  }

  public HeadingPlan direction(TurnDirection turn) {
    return new HeadingPlan(
        target, offsetMeters, turn, gate, toleranceRadians, angularSpeedTolerance, dwellSeconds);
  }

  public HeadingPlan clockwise() {
    return direction(TurnDirection.CW);
  }

  public HeadingPlan counterClockwise() {
    return direction(TurnDirection.CCW);
  }

  public HeadingPlan hardGate() {
    return gate(GatePolicy.HARD);
  }

  public HeadingPlan tolerance(Rotation2d tolerance) {
    return new HeadingPlan(
        target,
        offsetMeters,
        direction,
        gate,
        tolerance.getRadians(),
        angularSpeedTolerance,
        dwellSeconds);
  }

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
        dwellSeconds);
  }
}
