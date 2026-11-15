package com.team581.trailblazer3.model;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Objects;
import java.util.function.Supplier;

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
    record DynamicPoint(
        Supplier<TimestampedTarget<Translation2d>> supplier, Rotation2d mechanismOffset)
        implements Target {
      public DynamicPoint {
        Objects.requireNonNull(supplier);
        if (!Double.isFinite(mechanismOffset.getRadians())) {
          throw new IllegalArgumentException("Invalid aim offset");
        }
      }
    }

    record Fixed(Rotation2d angle) implements Target {
      public Fixed {
        if (!Double.isFinite(angle.getRadians())) {
          throw new IllegalArgumentException("Invalid fixed heading");
        }
      }
    }

    record Hold() implements Target {}

    record Point(Translation2d target, Rotation2d mechanismOffset) implements Target {
      public Point {
        if (!Double.isFinite(target.getNorm()) || !Double.isFinite(mechanismOffset.getRadians())) {
          throw new IllegalArgumentException("Invalid aim point");
        }
      }
    }

    record Travel() implements Target {}
  }

  public HeadingPlan {
    Objects.requireNonNull(target);
    Objects.requireNonNull(direction);
    Objects.requireNonNull(gate);
    if (!Double.isFinite(offsetMeters)
        || !Double.isFinite(toleranceRadians)
        || toleranceRadians < 0
        || !Double.isFinite(angularSpeedTolerance)
        || angularSpeedTolerance < 0
        || !Double.isFinite(dwellSeconds)
        || dwellSeconds < 0) {
      throw new IllegalArgumentException("Invalid heading keyframe");
    }
  }

  public static HeadingPlan face(Rotation2d angle) {
    return to(angle);
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

  public static HeadingPlan hold() {
    return of(new Target.Hold());
  }

  public static HeadingPlan facePoint(Translation2d point) {
    return of(new Target.Point(point, Rotation2d.kZero));
  }

  public static HeadingPlan facePointDynamic(Supplier<TimestampedTarget<Translation2d>> supplier) {
    return of(new Target.DynamicPoint(supplier, Rotation2d.kZero));
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

  public HeadingPlan shortestTurn() {
    return direction(TurnDirection.SHORTEST);
  }

  public HeadingPlan softGate() {
    return gate(GatePolicy.SOFT);
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

  public HeadingPlan withinDegrees(double toleranceDegrees) {
    return tolerance(Rotation2d.fromDegrees(toleranceDegrees));
  }

  public HeadingPlan flip(FieldGeometry field) {
    Target flipped = target;
    if (target instanceof Target.Fixed fixed) {
      flipped =
          new Target.Fixed(
              field
                  .flipPose(
                      new edu.wpi.first.math.geometry.Pose2d(Translation2d.kZero, fixed.angle()))
                  .getRotation());
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
