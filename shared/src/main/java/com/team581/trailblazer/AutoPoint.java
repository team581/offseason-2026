package com.team581.trailblazer;

import com.team581.autos.Point;
import com.team581.math.PoseErrorTolerance;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.Optional;
import java.util.function.Supplier;

public record AutoPoint<T extends Enum<T>>(
    Supplier<Point> poseSupplier,
    Optional<LinearConstraintOptions> linearConstraints,
    Optional<AngularConstraintOptions> angularConstraints,
    Optional<PoseErrorTolerance> transitionTolerance,
    Optional<T> marker,
    Optional<Pose2d> arcMidpoint) {
  public static AutoPoint<EmptyMarker> of(Point pose) {
    return new AutoPoint<EmptyMarker>(
        () -> pose,
        Optional.empty(),
        Optional.empty(),
        Optional.empty(),
        Optional.empty(),
        Optional.empty());
  }

  public static AutoPoint<EmptyMarker> of(Supplier<Point> poseSupplier) {
    return new AutoPoint<EmptyMarker>(
        poseSupplier,
        Optional.empty(),
        Optional.empty(),
        Optional.empty(),
        Optional.empty(),
        Optional.empty());
  }

  public static AutoPoint<EmptyMarker> ofBlue(Pose2d pose) {
    return of(Point.ofBlue(pose));
  }

  public static AutoPoint<EmptyMarker> ofRed(Pose2d pose) {
    return of(Point.ofRed(pose));
  }

  public Pose2d getPose() {
    return poseSupplier.get().getPose();
  }

  public AutoPoint<T> withLinearConstraints(double maxVelocity, double maxAcceleration) {
    return new AutoPoint<T>(
        poseSupplier,
        Optional.of(new LinearConstraintOptions(maxVelocity, maxAcceleration)),
        angularConstraints,
        transitionTolerance,
        marker,
        arcMidpoint);
  }

  public AutoPoint<T> withAngularConstraints(
      double maxAngularVelocity, double maxAngularAcceleration) {
    return new AutoPoint<T>(
        poseSupplier,
        linearConstraints,
        Optional.of(new AngularConstraintOptions(maxAngularVelocity, maxAngularAcceleration)),
        transitionTolerance,
        marker,
        arcMidpoint);
  }

  public AutoPoint<T> withTransitionTolerance(PoseErrorTolerance transitionTolerance) {
    return new AutoPoint<T>(
        poseSupplier,
        linearConstraints,
        angularConstraints,
        Optional.of(transitionTolerance),
        marker,
        arcMidpoint);
  }

  public <E extends Enum<E>> AutoPoint<E> withMarker(E marker) {
    return new AutoPoint<E>(
        poseSupplier,
        linearConstraints,
        angularConstraints,
        transitionTolerance,
        Optional.of(marker),
        arcMidpoint);
  }

  public AutoPoint<T> withArcMidpoint(Pose2d arcMidpoint) {
    return new AutoPoint<T>(
        poseSupplier,
        linearConstraints,
        angularConstraints,
        transitionTolerance,
        marker,
        Optional.of(arcMidpoint));
  }
}
