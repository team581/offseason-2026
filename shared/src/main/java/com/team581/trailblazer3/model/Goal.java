package com.team581.trailblazer3.model;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Objects;
import java.util.function.Supplier;

public record Goal(
    String id,
    Pose2d pose,
    PassType passType,
    double withinMeters,
    double passSpeed,
    double maxOverrunMeters,
    FinishTolerance finishTolerance,
    Enum<?> marker,
    HeadingPlan heading,
    DynamicTarget dynamicTarget) {
  public record DynamicTarget(
      Supplier<TimestampedTarget<Pose2d>> supplier, TargetLossPolicy lossPolicy) {
    public DynamicTarget {
      Objects.requireNonNull(supplier);
      Objects.requireNonNull(lossPolicy);
    }
  }

  public enum PassType {
    STOP,
    THROUGH,
    CRUISE_THROUGH
  }

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

  public Goal {
    if (id != null && id.isBlank()) {
      throw new IllegalArgumentException("Blank goal ID");
    }
    if (!Double.isFinite(pose.getX())
        || !Double.isFinite(pose.getY())
        || !Double.isFinite(pose.getRotation().getRadians())) {
      throw new IllegalArgumentException("Nonfinite goal pose");
    }
    Objects.requireNonNull(passType);
    Objects.requireNonNull(finishTolerance);
    nonnegative(withinMeters);
    nonnegative(maxOverrunMeters);
    if (Double.isNaN(passSpeed)
        || passSpeed < 0
        || (passType == PassType.CRUISE_THROUGH
            && (!Double.isFinite(passSpeed) || passSpeed == 0 || maxOverrunMeters == 0))) {
      throw new IllegalArgumentException("Invalid goal speed or overrun");
    }
  }

  private static void nonnegative(double value) {
    if (!Double.isFinite(value) || value < 0) {
      throw new IllegalArgumentException("Expected a finite nonnegative value");
    }
  }

  public static Goal stop(Pose2d pose) {
    return create(pose, PassType.STOP, 0, 0);
  }

  public static Goal through(Translation2d point) {
    return create(
        new Pose2d(point, Rotation2d.kZero), PassType.THROUGH, Double.POSITIVE_INFINITY, 0);
  }

  public static Goal through(Pose2d pose) {
    return through(pose.getTranslation());
  }

  public static Goal cruiseThrough(Translation2d point, double speed, double maxOverrunMeters) {
    return create(
        new Pose2d(point, Rotation2d.kZero), PassType.CRUISE_THROUGH, speed, maxOverrunMeters);
  }

  private static Goal create(Pose2d pose, PassType type, double speed, double overrun) {
    return new Goal(
        null,
        pose,
        type,
        0.25,
        speed,
        overrun,
        new FinishTolerance(0.03, Math.toRadians(2), 0.10, 0.10, 0.10),
        null,
        null,
        null);
  }

  public static Goal through(
      Supplier<TimestampedTarget<Pose2d>> supplier, TargetLossPolicy policy) {
    return through(Pose2d.kZero).tracking(supplier, policy);
  }

  public static Goal stop(Supplier<TimestampedTarget<Pose2d>> supplier, TargetLossPolicy policy) {
    return stop(Pose2d.kZero).tracking(supplier, policy);
  }

  public Goal tracking(Supplier<TimestampedTarget<Pose2d>> supplier, TargetLossPolicy policy) {
    return new Goal(
        id,
        pose,
        passType,
        withinMeters,
        passSpeed,
        maxOverrunMeters,
        finishTolerance,
        marker,
        heading,
        new DynamicTarget(supplier, policy));
  }

  public Goal heading(HeadingPlan keyframe) {
    return new Goal(
        id,
        pose,
        passType,
        withinMeters,
        passSpeed,
        maxOverrunMeters,
        finishTolerance,
        marker,
        keyframe,
        dynamicTarget);
  }

  public Goal passSpeed(double metersPerSecond) {
    return new Goal(
        id,
        pose,
        passType,
        withinMeters,
        metersPerSecond,
        maxOverrunMeters,
        finishTolerance,
        marker,
        heading,
        dynamicTarget);
  }

  public Goal flip(FieldGeometry field) {
    var transformed = dynamicTarget == null ? withPose(field.flipPose(pose)) : this;
    if (dynamicTarget != null
        && dynamicTarget.lossPolicy() instanceof TargetLossPolicy.FallbackNominal fallback) {
      transformed =
          transformed.tracking(
              dynamicTarget.supplier(),
              TargetLossPolicy.fallbackNominal(field.flipPose(fallback.nominalPose())));
    }
    return heading == null ? transformed : transformed.heading(heading.flip(field));
  }

  public Goal withId(String goalId) {
    return new Goal(
        goalId,
        pose,
        passType,
        withinMeters,
        passSpeed,
        maxOverrunMeters,
        finishTolerance,
        marker,
        heading,
        dynamicTarget);
  }

  public Goal withPose(Pose2d target) {
    return new Goal(
        id,
        target,
        passType,
        withinMeters,
        passSpeed,
        maxOverrunMeters,
        finishTolerance,
        marker,
        heading,
        dynamicTarget);
  }

  public Goal within(double meters) {
    return new Goal(
        id,
        pose,
        passType,
        meters,
        passSpeed,
        maxOverrunMeters,
        finishTolerance,
        marker,
        heading,
        dynamicTarget);
  }

  public Goal marker(Enum<?> event) {
    return new Goal(
        id,
        pose,
        passType,
        withinMeters,
        passSpeed,
        maxOverrunMeters,
        finishTolerance,
        event,
        heading,
        dynamicTarget);
  }

  public Goal finish(double meters, Rotation2d headingTolerance, double speed) {
    return finish(
        new FinishTolerance(
            meters,
            headingTolerance.getRadians(),
            speed,
            finishTolerance.angularSpeed(),
            finishTolerance.dwellSeconds()));
  }

  public Goal finish(FinishTolerance tolerance) {
    return new Goal(
        id,
        pose,
        passType,
        withinMeters,
        passSpeed,
        maxOverrunMeters,
        tolerance,
        marker,
        heading,
        dynamicTarget);
  }
}
